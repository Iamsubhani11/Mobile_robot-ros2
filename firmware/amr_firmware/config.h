// Everything you may need to change. See docs/WIRING.md for the wiring of each board.
#pragma once

// ---------- which board: picked automatically by the Arduino IDE ------------------
#if defined(ARDUINO_AVR_UNO)
  #define BOARD_NAME "uno"
  #define HOST Serial                // USB cable to the Pi or laptop
#elif defined(ESP32)
  #define BOARD_NAME "esp32"
  #define HOST Serial                // USB cable to the Pi or laptop
#elif defined(ARDUINO_UNO_Q)
  #define BOARD_NAME "uno_q"
  #define HOST Serial1               // D0/D1 header UART. A boot message appears on reset; the
                                     // ROS bridge drops it because it fails the checksum.
#elif defined(SIL_TEST)
  #define BOARD_NAME "sil"
  #define HOST Serial
#else
  #error "Add your board's pins to config.h"
#endif

#define HOST_BAUD 115200

// ---------- motors: two L298N. Order is FL, FR, RL, RR -----------------------------
#if defined(ESP32)
  const uint8_t PIN_EN[4]  = {25, 26, 27, 14};
  const uint8_t PIN_IN1[4] = {16, 18, 23, 32};
  const uint8_t PIN_IN2[4] = {17, 19, 13, 33};
  const uint8_t PIN_ENC_A[4] = {34, 35, 36, 39};   // input-only pins, 3.3 V max
  const uint8_t PIN_ENC_B[4] = {4, 5, 15, 2};
#else   // Uno, UNO Q: same header layout
  const uint8_t PIN_EN[4]  = {3, 5, 6, 9};         // PWM pins
  const uint8_t PIN_IN1[4] = {2, 7, 12, 10};
  const uint8_t PIN_IN2[4] = {4, 8, 13, 11};
  const uint8_t PIN_ENC_A[4] = {A0, A1, A2, A3};   // one channel per wheel: pins run out on an Uno
#endif

// Flip any wheel that spins backwards, rather than rewiring it.
const int8_t MOTOR_SIGN[4] = {1, 1, 1, 1};

const float MAX_WHEEL_RAD_S = 25.0f;   // wheel speed at full PWM: measure yours, and match robot.yaml
const uint8_t MIN_PWM = 60;            // below this the motor just hums. Raise it until the wheel starts.

// ---------- encoders: leave 0 until they are wired ---------------------------------
#define USE_ENCODERS 0
const float TICKS_PER_WHEEL_REV = 330.0f;   // encoder pulses x gear ratio, on the one edge counted
const float KP = 8.0f, KI = 20.0f;          // PWM per rad/s of error

// ---------- servos: PCA9685 on I2C, channels 0 to 4 ---------------------------------
#define PCA9685_ADDR 0x40
const uint8_t SERVO_CH[5] = {0, 1, 2, 3, 4};      // waist, shoulder, elbow, wrist, gripper
const int8_t SERVO_SIGN[5] = {1, 1, 1, 1, 1};     // set -1 if a joint moves the wrong way
const int16_t SERVO_CENTER_US[5] = {1500, 1500, 1500, 1500, 800};    // pulse at q = 0: CAD pose; gripper fully open
const float SERVO_US_PER_RAD = 636.6f;            // 2000 us across 180 degrees
const int16_t SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
const float SERVO_MAX_RATE = 1.5f;                // rad/s: limits the jump when a new target arrives
// Joint limits from the CAD sweep (rad). The same values are in the URDF.
const float Q_MIN[5] = {-1.57f, -1.05f, -1.57f, -1.57f, 0.00f};
const float Q_MAX[5] = { 1.57f,  0.59f,  1.01f,  1.10f, 2.20f};

// ---------- safety and timing -------------------------------------------------------
const uint16_t WATCHDOG_MS = 500;     // motors stop if no wheel command arrives for this long
const uint16_t CONTROL_MS = 20;
const uint16_t REPORT_MS = 40;
