// Host stand-in for the Arduino API: runs the real sketch on a PC for testing.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
uint32_t millis();
void delay(uint32_t);
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
void analogWrite(uint8_t, int);
inline void noInterrupts() {}
inline void interrupts() {}
template <class T, class L, class H> auto constrain(T v, L lo, H hi) -> decltype(v + lo + hi) { return v < lo ? lo : (v > hi ? hi : v); }
struct SerialSim {
  void begin(long);
  int available();
  int read();
  void write(uint8_t);
  void write(const uint8_t *, size_t);
};
extern SerialSim Serial;
