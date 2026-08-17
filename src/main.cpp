#include <Arduino.h>

#include "battery_manager.h"
#include "ble_manager.h"
#include "diagnostics.h"
#include "haptic_manager.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 9600;
constexpr unsigned long SERIAL_WAIT_MS = 2000;

// Essential LEDs (searching, an error, and low battery) remain enabled.
// Set this to true to restore all connection and notification feedback.
constexpr bool ENABLE_OPTIONAL_LED_FEEDBACK = false;

void restartDevice()
{
    Serial.println(F("[SYSTEM] Restarting..."));
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
}

void handleSerialCommands()
{
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'c' || command == 'C') {
            bleManager::clearBonds();
        } else if (command == 'b' || command == 'B') {
            batteryManager::reportStatus();
        } else if (command == 'r' || command == 'R') {
            restartDevice();
        }
    }
}

} // namespace

void setup()
{
    diagnostics::initializeStatusLeds(ENABLE_OPTIONAL_LED_FEEDBACK);

    Serial.begin(SERIAL_BAUD);
    const unsigned long serialWaitStartedAt = millis();
    while (!Serial && millis() - serialWaitStartedAt < SERIAL_WAIT_MS) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("========== Notifense ANCS v0 =========="));
    Serial.println(F("Adafruit Bluefruit + Nordic S140 SoftDevice"));
    Serial.println(F("ANCS notifications with DRV2605 haptic feedback"));
    diagnostics::blinkLed(LED_BLUE, 1);

    batteryManager::begin();
    hapticManager::begin();
    bleManager::begin();
}

void loop()
{
    handleSerialCommands();
    hapticManager::update();
    bleManager::update();
    delay(100);
}
