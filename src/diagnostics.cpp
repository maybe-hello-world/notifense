#include "diagnostics.h"

namespace diagnostics {
namespace {

constexpr unsigned long LED_PULSE_MS = 90;
bool optionalLedFeedbackEnabled = false;

void writeLed(uint8_t pin, bool on)
{
    // The XIAO RGB LED channels are active-low.
    digitalWrite(pin, on ? LOW : HIGH);
}

bool feedbackEnabled(LedFeedback feedback)
{
    return feedback == LedFeedback::Essential || optionalLedFeedbackEnabled;
}

} // namespace

void initializeStatusLeds(bool optionalFeedbackEnabled)
{
    optionalLedFeedbackEnabled = optionalFeedbackEnabled;

    // Preload the inactive level before switching the active-low pins to
    // outputs, avoiding a brief boot flash.
    writeLed(LED_RED, false);
    writeLed(LED_GREEN, false);
    writeLed(LED_BLUE, false);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
}

void setLed(uint8_t pin, bool on, LedFeedback feedback)
{
    // Turning a channel off is never suppressed, so no stale LED can remain on.
    writeLed(pin, on && feedbackEnabled(feedback));
}

void turnLedsOff()
{
    writeLed(LED_RED, false);
    writeLed(LED_GREEN, false);
    writeLed(LED_BLUE, false);
}

void blinkLed(uint8_t pin, uint8_t pulseCount, LedFeedback feedback)
{
    if (!feedbackEnabled(feedback)) {
        return;
    }

    for (uint8_t pulse = 0; pulse < pulseCount; ++pulse) {
        writeLed(pin, true);
        delay(LED_PULSE_MS);
        writeLed(pin, false);

        if (pulse + 1 < pulseCount) {
            delay(LED_PULSE_MS);
        }
    }
}

void printHexByte(uint8_t value)
{
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

[[noreturn]] void fatalError(const __FlashStringHelper *message)
{
    Serial.print(F("[FATAL] "));
    Serial.println(message);
    blinkLed(LED_RED, 1, LedFeedback::Essential);

    while (true) {
        delay(1000);
    }
}

} // namespace diagnostics
