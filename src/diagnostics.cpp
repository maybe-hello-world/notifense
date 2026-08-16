#include "diagnostics.h"

namespace diagnostics {
namespace {

constexpr unsigned long LED_PULSE_MS = 90;

} // namespace

void initializeStatusLeds()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    turnLedsOff();
}

void setLed(uint8_t pin, bool on)
{
    // The XIAO RGB LED channels are active-low.
    digitalWrite(pin, on ? LOW : HIGH);
}

void turnLedsOff()
{
    setLed(LED_RED, false);
    setLed(LED_GREEN, false);
    setLed(LED_BLUE, false);
}

void blinkLed(uint8_t pin, uint8_t pulseCount)
{
    for (uint8_t pulse = 0; pulse < pulseCount; ++pulse) {
        setLed(pin, true);
        delay(LED_PULSE_MS);
        setLed(pin, false);
        delay(LED_PULSE_MS);
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

    while (true) {
        blinkLed(LED_RED, 3);
        delay(1000);
    }
}

} // namespace diagnostics
