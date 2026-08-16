#include "battery_manager.h"

#include <Arduino.h>

namespace batteryManager {
namespace {

// The installed Seeed/Adafruit variant maps these Arduino pin numbers to the
// BQ25101 charger signals P0.13 (HICHG) and P0.17 (~CHG).
constexpr uint8_t CHARGING_CURRENT_PIN = 22;
constexpr uint8_t CHARGE_STATUS_PIN = 23;

constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_LEVELS = 1U << ADC_BITS;
constexpr float ADC_REFERENCE_V = 3.6F;
constexpr float VBAT_DIVIDER_RATIO = 1510.0F / 510.0F;
constexpr uint8_t VOLTAGE_SAMPLE_COUNT = 8;

struct ChargeCurvePoint {
    float voltage;
    uint8_t percentage;
};

// Generic approximation for a 1-cell 3.7 V Li-ion/LiPo battery. Voltage-based
// state of charge varies with the cell, load, temperature, and charging state.
constexpr ChargeCurvePoint CHARGE_CURVE[] = {
    {4.20F, 100},
    {4.11F, 90},
    {4.02F, 80},
    {3.95F, 70},
    {3.87F, 60},
    {3.84F, 50},
    {3.80F, 40},
    {3.77F, 30},
    {3.73F, 20},
    {3.69F, 10},
    {3.61F, 5},
    {3.30F, 0},
};

uint16_t lastRawReading = 0;

uint16_t readRawVoltage()
{
    uint32_t sampleTotal = 0;
    for (uint8_t sample = 0; sample < VOLTAGE_SAMPLE_COUNT; ++sample) {
        sampleTotal += analogRead(PIN_VBAT);
        delay(2);
    }

    return static_cast<uint16_t>(sampleTotal / VOLTAGE_SAMPLE_COUNT);
}

uint8_t estimateChargePercentage(float voltage)
{
    if (voltage >= CHARGE_CURVE[0].voltage) {
        return 100;
    }

    constexpr size_t pointCount = sizeof(CHARGE_CURVE) / sizeof(CHARGE_CURVE[0]);
    for (size_t index = 1; index < pointCount; ++index) {
        const ChargeCurvePoint &upper = CHARGE_CURVE[index - 1];
        const ChargeCurvePoint &lower = CHARGE_CURVE[index];

        if (voltage >= lower.voltage) {
            const float position = (
                voltage - lower.voltage
            ) / (
                upper.voltage - lower.voltage
            );
            const float percentage = lower.percentage + position * (
                upper.percentage - lower.percentage
            );
            return static_cast<uint8_t>(percentage + 0.5F);
        }
    }

    return 0;
}

} // namespace

void begin()
{
    // Leaving HICHG open selects the charger's low-current 50 mA mode.
    pinMode(CHARGING_CURRENT_PIN, INPUT);
    pinMode(CHARGE_STATUS_PIN, INPUT);

    pinMode(PIN_VBAT, INPUT);
    pinMode(VBAT_ENABLE, OUTPUT);

    // The voltage divider is active-low. Keep it enabled because Seeed warns
    // against driving this signal high while a battery is charging.
    digitalWrite(VBAT_ENABLE, LOW);

    analogReference(AR_DEFAULT);
    analogReadResolution(ADC_BITS);

    Serial.println(F("[BATTERY] Charger configured for 50 mA"));
    Serial.println(F("[BATTERY] Send B to report battery status"));
}

float readVoltage()
{
    lastRawReading = readRawVoltage();
    const float adcVoltage = (
        static_cast<float>(lastRawReading) * ADC_REFERENCE_V
    ) / ADC_LEVELS;

    return adcVoltage * VBAT_DIVIDER_RATIO;
}

bool isCharging()
{
    return digitalRead(CHARGE_STATUS_PIN) == LOW;
}

void reportStatus()
{
    const float voltage = readVoltage();
    const uint8_t percentage = estimateChargePercentage(voltage);

    Serial.println();
    Serial.println(F("========== Battery status =========="));
    Serial.print(F("[BATTERY] Voltage: "));
    Serial.print(voltage, 3);
    Serial.println(F(" V"));
    Serial.print(F("[BATTERY] Estimated charge: "));
    Serial.print(percentage);
    Serial.println('%');
    Serial.print(F("[BATTERY] ADC: "));
    Serial.print(lastRawReading);
    Serial.print(F(" / "));
    Serial.println(ADC_LEVELS - 1);
    Serial.println(F("[BATTERY] Charging current: 50 mA"));
    Serial.print(F("[BATTERY] Charger state: "));
    Serial.println(isCharging() ? F("charging") : F("not charging or full"));
    Serial.println(F("===================================="));
}

} // namespace batteryManager
