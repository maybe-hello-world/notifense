#include "haptic_manager.h"

#include <Arduino.h>
#include <HapticDrivers.hpp>

#include "diagnostics.h"

namespace hapticManager {
namespace {

constexpr int SENSOR_SDA_PIN = 4;
constexpr int SENSOR_SCL_PIN = 5;

HapticDriver_DRV2605 haptic;

} // namespace

void begin()
{
    Serial.print(F("[HAPTIC] Initializing DRV2605... "));
    if (!haptic.begin(
        Wire,
        DRV2605_SLAVE_ADDRESS,
        SENSOR_SDA_PIN,
        SENSOR_SCL_PIN
    )) {
        Serial.println(F("failed"));
        diagnostics::fatalError(F("DRV2605 initialization failed"));
    }

    haptic.setActuatorType(HapticActuatorType::LRA);
    Serial.println(F("ready (LRA)"));
}

void playEffect(uint8_t effectId)
{
    Serial.print(F("[HAPTIC] Playing effect "));
    Serial.println(effectId);

    if (!haptic.playEffect(effectId)) {
        Serial.println(F("[HAPTIC] Effect playback failed"));
        diagnostics::blinkLed(LED_RED, 2);
    }
}

} // namespace hapticManager
