// AMR firmware: one sketch for Arduino Uno, ESP32 and Arduino UNO Q.
//
// Serial protocol (all lines end in \n, *XX is the XOR checksum in hex of everything between $ and *):
//   host -> board  $V,fl,fr,rl,rr*XX         wheel speed, milli-rad/s
//                  $J,wa,sh,el,wr,gr*XX      joint target, milli-rad
//                  $Q*XX                     ask for the info line
//   board -> host  $S,ms,p1..p4,v1..v4,j1..j5,flags*XX     state at 25 Hz
//                  $I,board,encoders*XX
// flags: bit 0 = watchdog tripped (motors stopped), bit 1 = encoders in use.
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

static char rxLine[80];
static uint8_t rxLen = 0;
static float wheelCmd[4] = {0, 0, 0, 0};
static float wheelVel[4] = {0, 0, 0, 0};
static float wheelPos[4] = {0, 0, 0, 0};
static float integ[4] = {0, 0, 0, 0};
static float jointTarget[5] = {0, 0, 0, 0, 0};
static float jointPos[5] = {0, 0, 0, 0, 0};
static uint32_t lastCmdMs = 0, lastControlMs = 0, lastReportMs = 0;
static bool watchdog = true;

// ---------------------------------------- PCA9685, no library needed
static void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static void pcaInit() {
  pcaWrite(0x00, 0x10);                    // sleep, required to change the prescaler
  pcaWrite(0xFE, 121);                     // 25 MHz / (4096 * 50 Hz) - 1
  pcaWrite(0x00, 0xA0);                    // wake, auto-increment
  delay(2);
}
static void servoWriteUs(uint8_t ch, float us) {
  uint16_t ticks = (uint16_t)(us * 4096.0f / 20000.0f);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(0); Wire.write(0);
  Wire.write(ticks & 0xFF); Wire.write(ticks >> 8);
  Wire.endTransmission();
}
static void updateServo(uint8_t i) {
  float us = SERVO_CENTER_US[i] + SERVO_SIGN[i] * jointPos[i] * SERVO_US_PER_RAD;
  us = constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  servoWriteUs(SERVO_CH[i], us);
}

// ---------------------------------------- motors
static void motorWrite(uint8_t i, float pwm) {     // -255..255
  pwm *= MOTOR_SIGN[i];
  digitalWrite(PIN_IN1[i], pwm > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2[i], pwm < 0 ? HIGH : LOW);
  analogWrite(PIN_EN[i], (int)fabs(pwm));
}
static void motorsStop() {
  for (uint8_t i = 0; i < 4; i++) { motorWrite(i, 0); integ[i] = 0; }
}

// ---------------------------------------- encoders
#if USE_ENCODERS
static volatile long ticks[4] = {0, 0, 0, 0};
static long lastTicks[4] = {0, 0, 0, 0};
  #if defined(ARDUINO_AVR_UNO)
// One channel per wheel on pin-change interrupts A0..A3. The direction comes from the command.
static volatile uint8_t lastPinc = 0;
ISR(PCINT1_vect) {
  uint8_t now = PINC, changed = now ^ lastPinc; lastPinc = now;
  for (uint8_t i = 0; i < 4; i++)
    if ((changed & (1 << i)) && (now & (1 << i))) ticks[i] += wheelCmd[i] >= 0 ? 1 : -1;
}
static void encodersInit() {
  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_ENC_A[i], INPUT_PULLUP);
  lastPinc = PINC; PCICR |= (1 << PCIE1); PCMSK1 |= 0x0F;
}
  #elif defined(ESP32)
template <uint8_t I> void IRAM_ATTR isrEnc() { ticks[I] += digitalRead(PIN_ENC_B[I]) ? 1 : -1; }
static void encodersInit() {
  for (uint8_t i = 0; i < 4; i++) { pinMode(PIN_ENC_A[i], INPUT); pinMode(PIN_ENC_B[i], INPUT); }
  attachInterrupt(PIN_ENC_A[0], isrEnc<0>, RISING); attachInterrupt(PIN_ENC_A[1], isrEnc<1>, RISING);
  attachInterrupt(PIN_ENC_A[2], isrEnc<2>, RISING); attachInterrupt(PIN_ENC_A[3], isrEnc<3>, RISING);
}
  #else
template <uint8_t I> void isrEnc() { ticks[I] += wheelCmd[I] >= 0 ? 1 : -1; }
static void encodersInit() {
  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_ENC_A[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[0]), isrEnc<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[1]), isrEnc<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[2]), isrEnc<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[3]), isrEnc<3>, RISING);
}
  #endif
#endif

// ---------------------------------------- protocol
static uint8_t checksum(const char *s, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}
static void sendLine(char *body, uint8_t n) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  uint8_t c = checksum(body, n);
  HOST.write('$');
  HOST.write((const uint8_t *)body, n);
  HOST.write('*');
  HOST.write(HEX_DIGITS[c >> 4]);
  HOST.write(HEX_DIGITS[c & 0x0F]);
  HOST.write('\n');
}
static uint8_t appendLong(char *buf, uint8_t at, long v) {
  buf[at++] = ',';
  char tmp[12]; uint8_t t = 0;
  if (v < 0) { buf[at++] = '-'; v = -v; }
  do { tmp[t++] = '0' + (v % 10); v /= 10; } while (v);
  while (t) buf[at++] = tmp[--t];
  return at;
}
static void sendInfo() {
  char b[32]; uint8_t n = 0;
  b[n++] = 'I'; b[n++] = ',';
  for (const char *p = BOARD_NAME; *p; p++) b[n++] = *p;
  n = appendLong(b, n, USE_ENCODERS);
  sendLine(b, n);
}
static void sendState() {
  char b[140]; uint8_t n = 0;
  b[n++] = 'S';
  n = appendLong(b, n, (long)millis());
  for (uint8_t i = 0; i < 4; i++) n = appendLong(b, n, (long)(wheelPos[i] * 1000.0f));
  for (uint8_t i = 0; i < 4; i++) n = appendLong(b, n, (long)(wheelVel[i] * 1000.0f));
  for (uint8_t i = 0; i < 5; i++) n = appendLong(b, n, (long)(jointPos[i] * 1000.0f));
  n = appendLong(b, n, (watchdog ? 1 : 0) | (USE_ENCODERS ? 2 : 0));
  sendLine(b, n);
}
static int8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
// Reads up to `count` comma-separated integers after the first character of the body.
static uint8_t parseInts(const char *p, long *out, uint8_t count) {
  uint8_t got = 0;
  while (*p == ',' && got < count) {
    p++;
    bool neg = false; long v = 0; bool any = false;
    if (*p == '-') { neg = true; p++; }
    while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; any = true; }
    if (!any) return 0;
    out[got++] = neg ? -v : v;
  }
  return (*p == 0) ? got : 0;
}
static void handleLine(char *s, uint8_t n) {
  if (n < 4 || s[0] != '$' || s[n - 3] != '*') return;
  int8_t hi = hexVal(s[n - 2]), lo = hexVal(s[n - 1]);
  if (hi < 0 || lo < 0 || checksum(s + 1, n - 4) != (uint8_t)((hi << 4) | lo)) return;
  s[n - 3] = 0;
  long v[5];
  switch (s[1]) {
    case 'V':
      if (parseInts(s + 2, v, 4) == 4) {
        for (uint8_t i = 0; i < 4; i++) wheelCmd[i] = constrain(v[i] / 1000.0f, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
        lastCmdMs = millis();
        watchdog = false;
      }
      break;
    case 'J':
      if (parseInts(s + 2, v, 5) == 5)
        for (uint8_t i = 0; i < 5; i++) jointTarget[i] = constrain(v[i] / 1000.0f, Q_MIN[i], Q_MAX[i]);
      break;
    case 'Q':
      sendInfo();
      break;
  }
}
static void readHost() {
  while (HOST.available()) {
    char c = HOST.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxLine[rxLen] = 0; handleLine(rxLine, rxLen); }
      rxLen = 0;
    } else if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;     // too long: must be noise
    }
  }
}

// ---------------------------------------- control
static void control(float dt) {
  if (millis() - lastCmdMs > WATCHDOG_MS) {
    watchdog = true;
    for (uint8_t i = 0; i < 4; i++) wheelCmd[i] = 0;
  }
  for (uint8_t i = 0; i < 4; i++) {
    float pwm;
#if USE_ENCODERS
    noInterrupts(); long t = ticks[i]; interrupts();
    long d = t - lastTicks[i]; lastTicks[i] = t;
    float rad = d * 6.2831853f / TICKS_PER_WHEEL_REV;
    wheelPos[i] += rad;
    wheelVel[i] = 0.7f * wheelVel[i] + 0.3f * (rad / dt);
    float err = wheelCmd[i] - wheelVel[i];
    integ[i] = constrain(integ[i] + err * dt, -10.0f, 10.0f);
    pwm = wheelCmd[i] * 255.0f / MAX_WHEEL_RAD_S + KP * err + KI * integ[i];
#else
    // No feedback: report what was commanded, so odometry is an estimate only.
    wheelVel[i] = wheelCmd[i];
    wheelPos[i] += wheelVel[i] * dt;
    pwm = wheelCmd[i] * 255.0f / MAX_WHEEL_RAD_S;
#endif
    if (wheelCmd[i] == 0.0f) { pwm = 0; integ[i] = 0; }
    else if (fabs(pwm) < MIN_PWM) pwm = pwm > 0 ? MIN_PWM : -MIN_PWM;
    motorWrite(i, constrain(pwm, -255.0f, 255.0f));
  }
  float step = SERVO_MAX_RATE * dt;
  for (uint8_t i = 0; i < 5; i++) {
    float e = jointTarget[i] - jointPos[i];
    jointPos[i] += constrain(e, -step, step);
    updateServo(i);
  }
}

void setup() {
  HOST.begin(HOST_BAUD);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_EN[i], OUTPUT); pinMode(PIN_IN1[i], OUTPUT); pinMode(PIN_IN2[i], OUTPUT);
  }
  motorsStop();
  Wire.begin();
  pcaInit();
  for (uint8_t i = 0; i < 5; i++) updateServo(i);
#if USE_ENCODERS
  encodersInit();
#endif
  sendInfo();
}

void loop() {
  readHost();
  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_MS) {
    control((now - lastControlMs) * 0.001f);
    lastControlMs = now;
  }
  if (now - lastReportMs >= REPORT_MS) {
    sendState();
    lastReportMs = now;
  }
}
