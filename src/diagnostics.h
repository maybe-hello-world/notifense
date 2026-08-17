#pragma once

#include <Arduino.h>

namespace diagnostics {

enum class LedFeedback : uint8_t {
    Optional,
    Essential,
};

void initializeStatusLeds(bool optionalFeedbackEnabled);
void setLed(
    uint8_t pin,
    bool on,
    LedFeedback feedback = LedFeedback::Optional
);
void turnLedsOff();
void blinkLed(
    uint8_t pin,
    uint8_t pulseCount,
    LedFeedback feedback = LedFeedback::Optional
);
void printHexByte(uint8_t value);
[[noreturn]] void fatalError(const __FlashStringHelper *message);

} // namespace diagnostics
