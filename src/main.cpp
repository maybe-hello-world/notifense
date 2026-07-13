#ifndef NOTIFENSE_DEBUG
#define NOTIFENSE_DEBUG 1 // Set to 0 for release builds.
#endif

#include <ArduinoBLE.h>
#include <HapticDrivers.hpp>

namespace {

constexpr char DEVICE_NAME[] = "Notifense";
constexpr char SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
constexpr char PATTERN_CHARACTERISTIC_UUID[] = "19B10001-E8F2-537E-4F6C-D104768A1214";

constexpr int BLUE_LED_PIN = LEDB;
constexpr int GREEN_LED_PIN = LEDG;
constexpr int RED_LED_PIN = LEDR;
constexpr int SENSOR_SDA_PIN = 4;
constexpr int SENSOR_SCL_PIN = 5;

// BLE interval units are 0.625 ms. A 250 ms interval balances discovery time
// and disconnected advertising power for the breadboard build.
constexpr uint16_t BLE_ADVERTISING_INTERVAL = 400;
constexpr unsigned long BLE_IDLE_POLL_TIMEOUT_MS = 1000;
constexpr unsigned long BLE_ACTIVE_POLL_TIMEOUT_MS = 10;

// The phone sends the DRV2605 ROM effect ID directly. Zero means no effect.
// SensorLib's 1..117 comment is stale; the TouchSense table contains 1..123.
constexpr uint8_t MAX_PATTERN_ID = 123;
constexpr uint8_t PROGRAMMATIC_STOP_PATTERN_ID = 118;
constexpr uint8_t PATTERN_QUEUE_CAPACITY = 4;
constexpr unsigned long HAPTIC_PLAYBACK_TIMEOUT_MS = 1000;
constexpr unsigned long HAPTIC_STANDBY_RETRY_MS = 100;
constexpr unsigned long DEBUG_SERIAL_WAIT_MS = 2000;
constexpr unsigned long LED_PULSE_MS = 80;
constexpr uint8_t ERROR_LED_PULSES = 2;

BLEService notificationService(SERVICE_UUID);
BLEByteCharacteristic patternCharacteristic(
    PATTERN_CHARACTERISTIC_UUID,
    BLEWrite
);
HapticDriver_DRV2605 haptic;

enum class HapticState : uint8_t {
    Standby,
    Playing,
    AwaitingStandby,
};

HapticState hapticState = HapticState::Standby;
uint8_t activePattern = 0;
unsigned long hapticStartedAt = 0;
unsigned long lastStandbyAttemptAt = 0;

uint8_t patternQueue[PATTERN_QUEUE_CAPACITY] = {};
uint8_t patternQueueHead = 0;
uint8_t patternQueueTail = 0;
uint8_t patternQueueCount = 0;

#if NOTIFENSE_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do { } while (0)
#define DEBUG_PRINTLN(...) do { } while (0)
#endif

void setStatusLed(int pin, bool on)
{
    digitalWrite(pin, on ? LOW : HIGH);
}

void blinkStatusLed(int pin, uint8_t pulseCount = 1)
{
    for (uint8_t pulse = 0; pulse < pulseCount; ++pulse) {
        setStatusLed(pin, true);
        delay(LED_PULSE_MS);
        setStatusLed(pin, false);

        if (pulse + 1 < pulseCount) {
            delay(LED_PULSE_MS);
        }
    }
}

void blinkDebugLed(int pin, uint8_t pulseCount = 1)
{
#if NOTIFENSE_DEBUG
    blinkStatusLed(pin, pulseCount);
#else
    (void)pin;
    (void)pulseCount;
#endif
}

void initializeDiagnostics()
{
    // The RGB LED is active-low. HIGH is the zero-current idle state.
    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

#if NOTIFENSE_DEBUG
    Serial.begin(9600);
    const unsigned long waitStartedAt = millis();
    while (!Serial && millis() - waitStartedAt < DEBUG_SERIAL_WAIT_MS) {
        delay(10);
    }

    DEBUG_PRINTLN(F("Notifense booting"));
#endif
}

bool enqueuePattern(uint8_t pattern)
{
    if (pattern == 0) {
        DEBUG_PRINTLN(F("Ignoring pattern 0 (no effect)"));
        return false;
    }

    if (pattern > MAX_PATTERN_ID) {
        DEBUG_PRINT(F("Ignoring invalid pattern: "));
        DEBUG_PRINTLN(pattern);
        blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
        return false;
    }

    if (patternQueueCount >= PATTERN_QUEUE_CAPACITY) {
        DEBUG_PRINTLN(F("Pattern queue full; dropping newest command"));
        blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
        return false;
    }

    patternQueue[patternQueueTail] = pattern;
    patternQueueTail = (patternQueueTail + 1) % PATTERN_QUEUE_CAPACITY;
    ++patternQueueCount;
    return true;
}

bool dequeuePattern(uint8_t &pattern)
{
    if (patternQueueCount == 0) {
        return false;
    }

    pattern = patternQueue[patternQueueHead];
    patternQueueHead = (patternQueueHead + 1) % PATTERN_QUEUE_CAPACITY;
    --patternQueueCount;
    return true;
}

void handleBleConnected(BLEDevice central)
{
    DEBUG_PRINT(F("Connected to: "));
    DEBUG_PRINTLN(central.address());
    blinkStatusLed(GREEN_LED_PIN);
    (void)central;
}

void handleBleDisconnected(BLEDevice central)
{
    DEBUG_PRINT(F("Disconnected from: "));
    DEBUG_PRINTLN(central.address());
    blinkDebugLed(BLUE_LED_PIN);
    (void)central;

    // ArduinoBLE retains the advertising configuration and automatically
    // resumes advertising after this callback, including after range loss.
}

void handlePatternWritten(BLEDevice central, BLECharacteristic characteristic)
{
    const uint8_t pattern = patternCharacteristic.value();
    if (enqueuePattern(pattern)) {
        DEBUG_PRINT(F("Queued pattern: "));
        DEBUG_PRINTLN(pattern);
    }

    (void)central;
    (void)characteristic;
}

void initializeHaptic()
{
    while (!haptic.begin(
        Wire,
        DRV2605_SLAVE_ADDRESS,
        SENSOR_SDA_PIN,
        SENSOR_SCL_PIN
    )) {
        DEBUG_PRINTLN(F("DRV2605 initialization failed; retrying"));
        delay(1000);
        blinkStatusLed(RED_LED_PIN, ERROR_LED_PULSES);
    }

    // SensorLib defaults to ERM. Select the LRA feedback mode and ROM library.
    haptic.setActuatorType(HapticActuatorType::LRA);

    while (!haptic.setMode(HapticMode::STANDBY)) {
        DEBUG_PRINTLN(F("DRV2605 standby failed; retrying"));
        delay(1000);
        blinkStatusLed(RED_LED_PIN, ERROR_LED_PULSES);
    }

    hapticState = HapticState::Standby;
    DEBUG_PRINTLN(F("DRV2605 initialized in LRA standby mode"));
}

void initializeBle()
{
    while (!BLE.begin()) {
        DEBUG_PRINTLN(F("BLE initialization failed; retrying"));
        delay(1000);
        blinkStatusLed(RED_LED_PIN, ERROR_LED_PULSES);
    }

    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);
    BLE.setAdvertisedService(notificationService);
    BLE.setAdvertisingInterval(BLE_ADVERTISING_INTERVAL);

    notificationService.addCharacteristic(patternCharacteristic);
    BLE.addService(notificationService);

    BLE.setEventHandler(BLEConnected, handleBleConnected);
    BLE.setEventHandler(BLEDisconnected, handleBleDisconnected);
    patternCharacteristic.setEventHandler(BLEWritten, handlePatternWritten);

    while (!BLE.advertise()) {
        DEBUG_PRINTLN(F("BLE advertising failed; retrying"));
        delay(1000);
        blinkStatusLed(RED_LED_PIN, ERROR_LED_PULSES);
    }

    blinkDebugLed(BLUE_LED_PIN);
    DEBUG_PRINTLN(F("BLE advertising as Notifense"));
}

void startPattern(uint8_t pattern)
{
    if (!haptic.setMode(HapticMode::INTERNAL_TRIGGER)) {
        DEBUG_PRINTLN(F("DRV2605 wake failed"));
        blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
        hapticState = HapticState::AwaitingStandby;
        return;
    }

    const HapticEffectId effect = static_cast<HapticEffectId>(pattern);
    if (!haptic.playEffect(effect)) {
        DEBUG_PRINTLN(F("DRV2605 playback start failed"));
        blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
        hapticState = HapticState::AwaitingStandby;
        return;
    }

    hapticStartedAt = millis();
    activePattern = pattern;
    hapticState = HapticState::Playing;
    DEBUG_PRINT(F("Playing DRV2605 effect: "));
    DEBUG_PRINTLN(effect);
}

void serviceHaptic()
{
    const unsigned long now = millis();

    if (hapticState == HapticState::Playing) {
        if (haptic.isDone()) {
            DEBUG_PRINTLN(F("Haptic playback complete"));
            activePattern = 0;
            hapticState = HapticState::AwaitingStandby;
            lastStandbyAttemptAt = now - HAPTIC_STANDBY_RETRY_MS;
        } else if (now - hapticStartedAt >= HAPTIC_PLAYBACK_TIMEOUT_MS) {
            if (activePattern == PROGRAMMATIC_STOP_PATTERN_ID) {
                // Effect 118 runs until stopped. The one-byte protocol has no
                // stop command, so bound it like a two-second notification.
                DEBUG_PRINTLN(F("Long buzz duration reached; stopping"));
            } else {
                DEBUG_PRINTLN(F("Haptic playback timed out; stopping"));
                blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
            }
            if (!haptic.stop()) {
                DEBUG_PRINTLN(F("DRV2605 stop command failed"));
            }
            activePattern = 0;
            hapticState = HapticState::AwaitingStandby;
            lastStandbyAttemptAt = now - HAPTIC_STANDBY_RETRY_MS;
        }
        return;
    }

    if (hapticState == HapticState::AwaitingStandby) {
        if (now - lastStandbyAttemptAt < HAPTIC_STANDBY_RETRY_MS) {
            return;
        }

        lastStandbyAttemptAt = now;
        if (haptic.setMode(HapticMode::STANDBY)) {
            hapticState = HapticState::Standby;
            DEBUG_PRINTLN(F("DRV2605 entered standby"));
        } else {
            DEBUG_PRINTLN(F("DRV2605 standby command failed; retrying"));
            blinkDebugLed(RED_LED_PIN, ERROR_LED_PULSES);
        }
        return;
    }

    uint8_t pattern = 0;
    if (dequeuePattern(pattern)) {
        startPattern(pattern);
    }
}

unsigned long blePollTimeout()
{
    return hapticState == HapticState::Standby && patternQueueCount == 0
        ? BLE_IDLE_POLL_TIMEOUT_MS
        : BLE_ACTIVE_POLL_TIMEOUT_MS;
}

} // namespace

void setup()
{
    initializeDiagnostics();
    initializeHaptic();
    initializeBle();
}

void loop()
{
    // Cordio wakes this timed poll immediately for connects, disconnects, and
    // characteristic writes; otherwise the RTOS can idle instead of busy-spin.
    BLE.poll(blePollTimeout());
    serviceHaptic();
}
