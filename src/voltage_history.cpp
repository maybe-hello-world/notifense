#include "voltage_history.h"

#include <Arduino.h>
#include <bluefruit.h>

#include "battery_manager.h"

namespace voltageHistory {
namespace {

constexpr unsigned long USB_POLL_INTERVAL_MS = 1000UL;

// One voltage measurement every 5 minutes.
constexpr unsigned long SAMPLE_INTERVAL_MS =
    5UL * 60UL * 1000UL;

// 576 * 5 minutes = 2880 minutes = 48 hours.
//
// Each sample is only 4 bytes, so this costs ~2.3 KB RAM.
constexpr size_t SAMPLE_CAPACITY = 576;

struct VoltageSample {
    uint16_t elapsedMinutes;
    uint16_t millivolts;
};

VoltageSample samples[SAMPLE_CAPACITY] = {};

size_t sampleHead = 0;
size_t sampleCount = 0;

unsigned long sessionStartedAt = 0;
unsigned long nextSampleAt = 0;
unsigned long nextUsbPollAt = 0;

bool batterySessionActive = false;
bool lastUsbPowered = true;

bool usbPowerPresent()
{
    // voltageHistory::begin() is called after Bluefruit.begin(), so the
    // SoftDevice owns the POWER peripheral. Ask it for USB status instead
    // of directly poking NRF_POWER.
    uint32_t usbStatus = 0;

    if (sd_power_usbregstatus_get(&usbStatus) != NRF_SUCCESS) {
        // Fail safe: if we can't determine USB state, don't record a
        // potentially charger-contaminated voltage.
        return true;
    }

    return (
        usbStatus & POWER_USBREGSTATUS_VBUSDETECT_Msk
    ) != 0;
}

void clearSamples()
{
    sampleHead = 0;
    sampleCount = 0;
}

void addSample(unsigned long now)
{
    const float voltage = batteryManager::readVoltage();

    const unsigned long elapsedMs = now - sessionStartedAt;
    const unsigned long elapsedMinutes =
        elapsedMs / (60UL * 1000UL);

    VoltageSample &sample = samples[sampleHead];

    sample.elapsedMinutes = static_cast<uint16_t>(
        min(elapsedMinutes, 65535UL)
    );

    sample.millivolts = static_cast<uint16_t>(
        voltage * 1000.0F + 0.5F
    );

    sampleHead = (sampleHead + 1) % SAMPLE_CAPACITY;

    if (sampleCount < SAMPLE_CAPACITY) {
        ++sampleCount;
    }
}

void startBatterySession(unsigned long now)
{
    clearSamples();

    batterySessionActive = true;
    sessionStartedAt = now;

    // Record the voltage immediately after USB disappears.
    addSample(now);

    nextSampleAt = now + SAMPLE_INTERVAL_MS;
}

} // namespace

void begin()
{
    const unsigned long now = millis();

    lastUsbPowered = usbPowerPresent();
    nextUsbPollAt = now + USB_POLL_INTERVAL_MS;

    if (!lastUsbPowered) {
        startBatterySession(now);
    }

    Serial.println(
        F("[VLOG] Battery-only voltage logger ready "
          "(5 min interval, 48 h history)")
    );
}

void update()
{
    const unsigned long now = millis();

    // Checking once per second is plenty. More GPIO bureaucracy would
    // accomplish remarkably little.
    if (
        static_cast<long>(now - nextUsbPollAt) < 0
    ) {
        return;
    }

    nextUsbPollAt = now + USB_POLL_INTERVAL_MS;

    const bool usbPowered = usbPowerPresent();

    if (usbPowered != lastUsbPowered) {
        lastUsbPowered = usbPowered;

        if (usbPowered) {
            // Freeze the history. Do NOT measure after USB insertion:
            // charging immediately alters battery terminal voltage.
            batterySessionActive = false;
        } else {
            // Every fresh USB disconnect begins a new experiment.
            startBatterySession(now);
        }
    }

    if (
        usbPowered
        || !batterySessionActive
        || static_cast<long>(now - nextSampleAt) < 0
    ) {
        return;
    }

    addSample(now);
    nextSampleAt = now + SAMPLE_INTERVAL_MS;
}

void report()
{
    Serial.println();
    Serial.println(F("========== Battery voltage history =========="));

    if (sampleCount == 0) {
        Serial.println(F("[VLOG] No battery-only samples recorded"));
        Serial.println(F("============================================="));
        return;
    }

    Serial.print(F("[VLOG] Samples: "));
    Serial.println(sampleCount);

    Serial.print(F("[VLOG] State: "));
    Serial.println(
        batterySessionActive
            ? F("currently running on battery")
            : F("frozen after USB connection")
    );

    if (sampleCount == SAMPLE_CAPACITY) {
        Serial.println(
            F("[VLOG] Buffer full; contains latest 48 hours")
        );
    }

    Serial.println();
    Serial.println(F("elapsed_min,voltage_v"));

    const size_t oldestIndex =
        (sampleHead + SAMPLE_CAPACITY - sampleCount)
        % SAMPLE_CAPACITY;

    for (size_t i = 0; i < sampleCount; ++i) {
        const size_t index =
            (oldestIndex + i) % SAMPLE_CAPACITY;

        const VoltageSample &sample = samples[index];

        Serial.print(sample.elapsedMinutes);
        Serial.print(',');

        Serial.println(
            static_cast<float>(sample.millivolts) / 1000.0F,
            3
        );
    }

    if (sampleCount >= 2) {
        const VoltageSample &first =
            samples[oldestIndex];

        const size_t lastIndex =
            (oldestIndex + sampleCount - 1)
            % SAMPLE_CAPACITY;

        const VoltageSample &last =
            samples[lastIndex];

        Serial.println();
        Serial.print(F("[VLOG] Duration: "));
        Serial.print(
            static_cast<unsigned long>(last.elapsedMinutes)
            - first.elapsedMinutes
        );
        Serial.println(F(" min"));

        Serial.print(F("[VLOG] Voltage change: "));
        Serial.print(
            (
                static_cast<int32_t>(last.millivolts)
                - static_cast<int32_t>(first.millivolts)
            ) / 1000.0F,
            3
        );
        Serial.println(F(" V"));
    }

    Serial.println(F("============================================="));
}

} // namespace voltageHistory