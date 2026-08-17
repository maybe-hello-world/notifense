#include "haptic_manager.h"

#include <atomic>

#include <Arduino.h>
#include <HapticDrivers.hpp>

#include "diagnostics.h"

namespace hapticManager {
namespace {

constexpr int SENSOR_SDA_PIN = 4;
constexpr int SENSOR_SCL_PIN = 5;
constexpr unsigned long PLAYBACK_TIMEOUT_MS = 1000;
constexpr unsigned long STANDBY_RETRY_MS = 100;
constexpr uint8_t PROGRAMMATIC_STOP_EFFECT_ID = 118;

enum class HapticState : uint8_t {
    Standby,
    Playing,
    AwaitingStandby,
};

HapticDriver_DRV2605 haptic;
SemaphoreHandle_t hapticMutex = nullptr;
HapticState hapticState = HapticState::Standby;
std::atomic<bool> hapticNeedsService(false);
unsigned long playbackStartedAt = 0;
unsigned long lastStandbyAttemptAt = 0;
uint8_t activeEffectId = 0;
bool standbyFailureReported = false;

void signalHapticError()
{
    diagnostics::blinkLed(
        LED_RED,
        1,
        diagnostics::LedFeedback::Essential
    );
}

void awaitStandby(unsigned long now)
{
    activeEffectId = 0;
    hapticState = HapticState::AwaitingStandby;
    hapticNeedsService.store(true, std::memory_order_release);
    lastStandbyAttemptAt = now - STANDBY_RETRY_MS;
}

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

    hapticMutex = xSemaphoreCreateMutex();
    if (hapticMutex == nullptr) {
        Serial.println(F("failed"));
        diagnostics::fatalError(F("DRV2605 mutex initialization failed"));
    }

    if (!haptic.setActuatorType(HapticActuatorType::LRA)) {
        Serial.println(F("failed"));
        diagnostics::fatalError(F("DRV2605 LRA configuration failed"));
    }

    if (!haptic.setMode(HapticMode::STANDBY)) {
        Serial.println(F("failed"));
        diagnostics::fatalError(F("DRV2605 standby initialization failed"));
    }

    hapticState = HapticState::Standby;
    hapticNeedsService.store(false, std::memory_order_release);
    Serial.println(F("ready (LRA, standby)"));
}

void playEffect(uint8_t effectId)
{
    Serial.print(F("[HAPTIC] Playing effect "));
    Serial.println(effectId);

    bool playbackFailed = false;
    if (xSemaphoreTake(hapticMutex, portMAX_DELAY) != pdTRUE) {
        Serial.println(F("[HAPTIC] Could not lock the driver"));
        signalHapticError();
        return;
    }

    if (hapticState == HapticState::Playing && !haptic.stop()) {
        Serial.println(F("[HAPTIC] Could not stop the previous effect"));
        playbackFailed = true;
    }

    if (!playbackFailed && hapticState == HapticState::Standby) {
        if (!haptic.setMode(HapticMode::INTERNAL_TRIGGER)) {
            Serial.println(F("[HAPTIC] Could not leave standby"));
            playbackFailed = true;
        }
    }

    if (!playbackFailed && haptic.playEffect(effectId)) {
        playbackStartedAt = millis();
        activeEffectId = effectId;
        standbyFailureReported = false;
        hapticState = HapticState::Playing;
        hapticNeedsService.store(true, std::memory_order_release);
    } else {
        Serial.println(F("[HAPTIC] Effect playback failed"));
        awaitStandby(millis());
        playbackFailed = true;
    }

    xSemaphoreGive(hapticMutex);

    if (playbackFailed) {
        signalHapticError();
    }
}

void update()
{
    if (!hapticNeedsService.load(std::memory_order_acquire)) {
        return;
    }

    if (xSemaphoreTake(hapticMutex, 0) != pdTRUE) {
        return;
    }

    const unsigned long now = millis();
    bool signalError = false;

    if (hapticState == HapticState::Playing) {
        if (haptic.isDone()) {
            awaitStandby(now);
        } else if (now - playbackStartedAt >= PLAYBACK_TIMEOUT_MS) {
            Serial.println(F("[HAPTIC] Playback timed out; stopping"));
            if (activeEffectId != PROGRAMMATIC_STOP_EFFECT_ID) {
                signalError = true;
            }
            if (!haptic.stop()) {
                Serial.println(F("[HAPTIC] Stop command failed"));
                signalError = true;
            }
            awaitStandby(now);
        }
    }

    if (
        hapticState == HapticState::AwaitingStandby
        && now - lastStandbyAttemptAt >= STANDBY_RETRY_MS
    ) {
        lastStandbyAttemptAt = now;
        if (haptic.setMode(HapticMode::STANDBY)) {
            hapticState = HapticState::Standby;
            hapticNeedsService.store(false, std::memory_order_release);
            standbyFailureReported = false;
            Serial.println(F("[HAPTIC] Standby"));
        } else if (!standbyFailureReported) {
            Serial.println(F("[HAPTIC] Standby command failed; retrying"));
            standbyFailureReported = true;
            signalError = true;
        }
    }

    xSemaphoreGive(hapticMutex);

    if (signalError) {
        signalHapticError();
    }
}

} // namespace hapticManager
