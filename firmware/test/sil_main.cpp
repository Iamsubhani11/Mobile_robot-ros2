// Software-in-the-loop: compiles the real sketch for this PC. Serial is stdin/stdout, which the test
// connects to a pty. Pin writes and PCA9685 traffic go to the log file named in argv[1].
#define SIL_TEST
#include "Arduino.h"
#include "Wire.h"
#include "../amr_firmware/amr_firmware.ino"
SerialSim Serial; WireSim Wire;
static FILE *logFile; static int rxb = -1;
static timespec t0;
uint32_t millis() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (t.tv_sec - t0.tv_sec) * 1000 + (t.tv_nsec - t0.tv_nsec) / 1000000; }
void delay(uint32_t ms) { usleep(ms * 1000); }
void pinMode(uint8_t, uint8_t) {}
static int pins[32];
void digitalWrite(uint8_t p, uint8_t v) { pins[p] = v; }
void analogWrite(uint8_t p, int v) {
  for (int i = 0; i < 4; i++) if (PIN_EN[i] == p) {
    static int last[4] = {-999, -999, -999, -999};
    int s = v * (pins[PIN_IN1[i]] ? 1 : (pins[PIN_IN2[i]] ? -1 : 0));
    if (s != last[i]) { fprintf(logFile, "%u MOTOR %d %d\n", millis(), i, s); fflush(logFile); last[i] = s; }
  }
}
void SerialSim::begin(long) { fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK); }
int SerialSim::available() { if (rxb >= 0) return 1; unsigned char c; if (::read(0, &c, 1) == 1) { rxb = c; return 1; } return 0; }
int SerialSim::read() { int c = rxb; rxb = -1; return c; }
void SerialSim::write(uint8_t c) { ::write(1, &c, 1); }
void SerialSim::write(const uint8_t *b, size_t n) { ::write(1, b, n); }
static uint8_t wbuf[8]; static int wn;
void WireSim::begin() {}
void WireSim::beginTransmission(uint8_t) { wn = 0; }
void WireSim::write(uint8_t v) { if (wn < 8) wbuf[wn++] = v; }
void WireSim::endTransmission() {
  if (wn == 5) {
    static int last[16]; int ch = (wbuf[0] - 6) / 4, tk = wbuf[3] | (wbuf[4] << 8);
    if (ch >= 0 && ch < 16 && tk != last[ch]) { fprintf(logFile, "%u SERVO %d %d\n", millis(), ch, tk * 20000 / 4096); fflush(logFile); last[ch] = tk; }
  }
}
int main(int, char **argv) {
  clock_gettime(CLOCK_MONOTONIC, &t0); logFile = fopen(argv[1], "w");
  setup();
  for (;;) { loop(); usleep(500); }
}
