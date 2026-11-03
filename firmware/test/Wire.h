#pragma once
#include <stdint.h>
struct WireSim { void begin(); void beginTransmission(uint8_t); void write(uint8_t); void endTransmission(); };
extern WireSim Wire;
