#pragma once

#include <Arduino.h>

namespace diagnostics {

void initializeStatusLeds();
void setLed(uint8_t pin, bool on);
void turnLedsOff();
void blinkLed(uint8_t pin, uint8_t pulseCount);
void printHexByte(uint8_t value);
[[noreturn]] void fatalError(const __FlashStringHelper *message);

} // namespace diagnostics
