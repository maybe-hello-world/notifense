#include <Arduino.h>

#include "ble_manager.h"
#include "diagnostics.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 9600;
constexpr unsigned long SERIAL_WAIT_MS = 2000;

void handleSerialCommands()
{
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'c' || command == 'C') {
            bleManager::clearBonds();
        }
    }
}

} // namespace

void setup()
{
    diagnostics::initializeStatusLeds();

    Serial.begin(SERIAL_BAUD);
    const unsigned long serialWaitStartedAt = millis();
    while (!Serial && millis() - serialWaitStartedAt < SERIAL_WAIT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("========== Notifense ANCS v0 =========="));
    Serial.println(F("Adafruit Bluefruit + Nordic S140 SoftDevice"));
    Serial.println(F("Serial diagnostics and status LEDs only; haptics are disabled"));
    diagnostics::blinkLed(LED_BLUE, 1);

    bleManager::begin();
}

void loop()
{
    handleSerialCommands();
    bleManager::update();
    delay(10);
}
