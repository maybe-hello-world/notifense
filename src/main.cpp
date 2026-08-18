#include <Arduino.h>

#include "battery_manager.h"
#include "ble_manager.h"
#include "diagnostics.h"
#include "haptic_manager.h"
#include "power_manager.h"
#include "voltage_history.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 9600;
constexpr unsigned long SERIAL_WAIT_MS = 2000;
constexpr size_t SERIAL_COMMAND_CAPACITY = 5;

char serialCommand[SERIAL_COMMAND_CAPACITY] = {};
size_t serialCommandLength = 0;
bool discardSerialCommand = false;

// Essential LEDs (searching, an error, and low battery) remain enabled.
// Set this to true to restore all connection and notification feedback.
constexpr bool ENABLE_OPTIONAL_LED_FEEDBACK = false;

void dispatchSerialCommand()
{
    if (serialCommandLength == 0) {
        return;
    }

    // P, P1 ... P123
    if (serialCommand[0] == 'p' || serialCommand[0] == 'P') {
        uint8_t effectId = 1;

        if (serialCommandLength > 1) {
            // Maximum valid command is P123.
            if (serialCommandLength > 4) {
                return;
            }

            unsigned int parsedEffectId = 0;

            for (size_t i = 1; i < serialCommandLength; ++i) {
                const char c = serialCommand[i];

                if (c < '0' || c > '9') {
                    return;
                }

                parsedEffectId =
                    parsedEffectId * 10 + static_cast<unsigned int>(c - '0');
            }

            if (parsedEffectId < 1 || parsedEffectId > 123) {
                Serial.println(F("[SERIAL] Haptic effect must be 1-123"));
                return;
            }

            effectId = static_cast<uint8_t>(parsedEffectId);
        }

        Serial.print(F("[SERIAL] Playing test haptic effect "));
        Serial.println(effectId);
        hapticManager::playEffect(effectId);
        return;
    }

    // All other commands are still exactly one character.
    if (serialCommandLength != 1) {
        return;
    }

    switch (serialCommand[0]) {
        case 'b':
        case 'B':
            batteryManager::reportStatus();
            break;

        case 'c':
        case 'C':
            bleManager::clearBonds();
            break;

        case 'r':
        case 'R':
            bleManager::retryAncsConnection();
            break;

        case 'v':
        case 'V':
            voltageHistory::report();
            break;

        case 'h':
        case 'H':
            Serial.println(
                F("[SERIAL] Commands: "
                  "B=battery, "
                  "V=voltage history, "
                  "C=clear bonds, "
                  "R=retry BLE, "
                  "P[1-123]=play haptic, "
                  "H=help")
            );
            break;

        default:
            break;
    }
}

void finishSerialCommand()
{
    if (!discardSerialCommand && serialCommandLength > 0) {
        dispatchSerialCommand();
    }

    serialCommandLength = 0;
    discardSerialCommand = false;
}

void handleSerialCommands()
{
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            return;
        }

        const char character = static_cast<char>(value);
        if (character == '\r' || character == '\n') {
            finishSerialCommand();
            continue;
        }

        if (discardSerialCommand) {
            continue;
        }

        const bool printable = character >= ' ' && character <= '~';
        if (
            !printable
            || serialCommandLength + 1 >= SERIAL_COMMAND_CAPACITY
        ) {
            serialCommandLength = 0;
            discardSerialCommand = true;
            continue;
        }

        serialCommand[serialCommandLength++] = character;
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
    Serial.println(F("[SERIAL] Commands require Enter: B=battery, C=clear bonds, R=retry BLE"));
    diagnostics::blinkLed(LED_BLUE, 1);

    batteryManager::begin();
    hapticManager::begin();
    bleManager::begin();

    // Bluefruit must already be running before voltageHistory::begin(),
    // because it uses the SoftDevice API to query VBUS.
    powerManager::begin();
    voltageHistory::begin();

    // Also fixes our delightful mystery-red-LED situation after all
    // third-party initialization is complete.
    diagnostics::turnLedsOff();
}

void loop()
{
    handleSerialCommands();

    hapticManager::update();
    bleManager::update();
    voltageHistory::update();

    delay(100);
}
