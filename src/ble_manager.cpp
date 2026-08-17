#include "ble_manager.h"

#include <atomic>

#include <Arduino.h>
#include <bluefruit.h>

#include "battery_manager.h"
#include "diagnostics.h"
#include "notification_processor.h"

namespace bleManager {
namespace {

constexpr char DEVICE_NAME[] = "Notifense";
constexpr int8_t TX_POWER_DBM = 0;
constexpr uint16_t CONNECTION_INTERVAL_MIN_MS = 150;
constexpr uint16_t CONNECTION_INTERVAL_MAX_MS = 165;
constexpr uint16_t CONNECTION_INTERVAL_MIN_UNITS =
    CONNECTION_INTERVAL_MIN_MS * 4 / 5;
constexpr uint16_t CONNECTION_INTERVAL_MAX_UNITS =
    CONNECTION_INTERVAL_MAX_MS * 4 / 5;
constexpr uint16_t CONNECTION_SLAVE_LATENCY = 4;
constexpr uint16_t CONNECTION_SUPERVISION_TIMEOUT_MS = 6000;
constexpr uint16_t CONNECTION_SUPERVISION_TIMEOUT_UNITS =
    CONNECTION_SUPERVISION_TIMEOUT_MS / 10;
constexpr unsigned long CONNECTION_PARAMETER_RETRY_DELAY_MS = 1000;
constexpr unsigned long CONNECTION_PARAMETER_REPORT_DELAY_MS = 3000;
constexpr uint16_t FAST_ADVERTISING_INTERVAL = 32;
constexpr uint16_t SLOW_ADVERTISING_INTERVAL = 1636;
constexpr uint8_t FAST_ADVERTISING_TIMEOUT_SECONDS = 30;
constexpr unsigned long SEARCH_LED_INTERVAL_MS = 10000;
constexpr unsigned long LOW_BATTERY_LED_INTERVAL_MS = 60000;
constexpr unsigned long BATTERY_LEVEL_INTERVAL_MS = 60000;
constexpr unsigned long CHARGING_STATUS_INTERVAL_MS = 1000;
constexpr uint8_t LOW_BATTERY_PERCENT = 20;

static_assert(
    CONNECTION_INTERVAL_MIN_MS % 15 == 0,
    "Apple BLE minimum interval must be a multiple of 15 ms"
);
static_assert(
    CONNECTION_INTERVAL_MAX_MS >= CONNECTION_INTERVAL_MIN_MS + 15,
    "Apple BLE interval range must span at least 15 ms"
);
static_assert(
    CONNECTION_INTERVAL_MAX_MS * (CONNECTION_SLAVE_LATENCY + 1) <= 2000,
    "Apple BLE interval and latency exceed the idle-event limit"
);
static_assert(
    CONNECTION_INTERVAL_MAX_MS * (CONNECTION_SLAVE_LATENCY + 1) * 3
        < CONNECTION_SUPERVISION_TIMEOUT_MS,
    "Apple BLE supervision timeout is too short"
);

// Bluetooth SIG Battery Service 1.1 UUIDs. Bluefruit's BLEBas supplies
// 0x180F/0x2A19; its installed version does not yet include 0x2BED.
constexpr uint16_t BATTERY_LEVEL_STATUS_UUID = 0x2BED;
constexpr uint8_t BATTERY_LEVEL_STATUS_LENGTH = 3;

constexpr uint8_t BATTERY_PRESENT_SHIFT = 0;
constexpr uint8_t WIRED_POWER_SHIFT = 1;
constexpr uint8_t CHARGE_STATE_SHIFT = 5;
constexpr uint8_t CHARGE_LEVEL_SHIFT = 7;

constexpr uint8_t WIRED_POWER_CONNECTED = 1;
constexpr uint8_t CHARGE_STATE_CHARGING = 1;
constexpr uint8_t CHARGE_STATE_DISCHARGING_ACTIVE = 2;
constexpr uint8_t CHARGE_LEVEL_GOOD = 1;
constexpr uint8_t CHARGE_LEVEL_LOW = 2;
constexpr uint8_t CHARGE_LEVEL_CRITICAL = 3;

BLEAncs ancsClient;
BLEBas batteryService;
BLECharacteristic batteryLevelStatus(BATTERY_LEVEL_STATUS_UUID);

unsigned long nextStatusAt = 0;
unsigned long nextLowBatteryLedAt = 0;
unsigned long nextBatteryLevelUpdateAt = 0;
unsigned long nextChargingStatusUpdateAt = 0;
uint8_t currentBatteryLevel = 0;
bool currentCharging = false;
bool lowPowerParameterRequestSent = false;
std::atomic<bool> connectionParameterRetryPending(false);
uint16_t connectionParameterRetryHandle = 0;
unsigned long connectionParameterRetryAt = 0;
std::atomic<bool> connectionParameterReportPending(false);
uint16_t connectionParameterReportHandle = 0;
unsigned long connectionParameterReportAt = 0;
std::atomic<bool> immediateSearchLedPending(false);

void connectCallback(uint16_t connectionHandle);
void disconnectCallback(uint16_t connectionHandle, uint8_t reason);
void pairingCompleteCallback(uint16_t connectionHandle, uint8_t authStatus);
void connectionSecuredCallback(uint16_t connectionHandle);
void notificationCallback(AncsNotification_t *notification);

void signalBleError()
{
    diagnostics::blinkLed(
        LED_RED,
        1,
        diagnostics::LedFeedback::Essential
    );
}

void printConnectionParameters(BLEConnection *connection)
{
    Serial.print(F("[BLE] Link interval: "));
    Serial.print(connection->getConnectionInterval() * 1.25F, 2);
    Serial.print(F(" ms, latency: "));
    Serial.print(connection->getSlaveLatency());
    Serial.print(F(", supervision timeout: "));
    Serial.print(connection->getSupervisionTimeout() * 10);
    Serial.println(F(" ms"));
}

bool connectionUsesLowPowerParameters(BLEConnection *connection)
{
    const uint16_t interval = connection->getConnectionInterval();
    return interval >= CONNECTION_INTERVAL_MIN_UNITS
        && interval <= CONNECTION_INTERVAL_MAX_UNITS
        && connection->getSlaveLatency() == CONNECTION_SLAVE_LATENCY
        && connection->getSupervisionTimeout()
            == CONNECTION_SUPERVISION_TIMEOUT_UNITS;
}

void submitLowPowerConnectionParameters(
    uint16_t connectionHandle,
    bool allowBusyRetry
)
{
    // Apple devices do not use the GAP Peripheral Preferred Connection
    // Parameters characteristic, so request the same Apple-valid range once
    // after ANCS setup. The phone remains free to reject or replace it.
    ble_gap_conn_params_t parameters = {};
    parameters.min_conn_interval = CONNECTION_INTERVAL_MIN_UNITS;
    parameters.max_conn_interval = CONNECTION_INTERVAL_MAX_UNITS;
    parameters.slave_latency = CONNECTION_SLAVE_LATENCY;
    parameters.conn_sup_timeout = CONNECTION_SUPERVISION_TIMEOUT_UNITS;

    const uint32_t result = sd_ble_gap_conn_param_update(
        connectionHandle,
        &parameters
    );
    if (result == NRF_ERROR_BUSY && allowBusyRetry) {
        Serial.println(F("[BLE] Link request busy; one retry scheduled"));
        connectionParameterRetryHandle = connectionHandle;
        connectionParameterRetryAt = millis()
            + CONNECTION_PARAMETER_RETRY_DELAY_MS;
        connectionParameterRetryPending.store(
            true,
            std::memory_order_release
        );
        return;
    }

    if (
        result == NRF_ERROR_INVALID_STATE
        || result == BLE_ERROR_INVALID_CONN_HANDLE
    ) {
        Serial.println(F("[BLE] Low-power link request canceled; disconnected"));
        return;
    }

    if (result != NRF_SUCCESS) {
        Serial.print(F("[BLE] Low-power link request failed: "));
        Serial.println(result);
        signalBleError();
        return;
    }

    Serial.println(F("[BLE] Low-power link parameters requested"));
    connectionParameterReportHandle = connectionHandle;
    connectionParameterReportAt = millis()
        + CONNECTION_PARAMETER_REPORT_DELAY_MS;
    connectionParameterReportPending.store(
        true,
        std::memory_order_release
    );
}

void requestLowPowerConnectionParameters(uint16_t connectionHandle)
{
    if (lowPowerParameterRequestSent) {
        return;
    }
    lowPowerParameterRequestSent = true;
    submitLowPowerConnectionParameters(connectionHandle, true);
}

uint8_t chargeLevelForPercentage(uint8_t percentage)
{
    if (percentage <= 10) {
        return CHARGE_LEVEL_CRITICAL;
    }
    if (percentage <= 20) {
        return CHARGE_LEVEL_LOW;
    }
    return CHARGE_LEVEL_GOOD;
}

void encodeBatteryLevelStatus(
    uint8_t percentage,
    bool charging,
    uint8_t status[BATTERY_LEVEL_STATUS_LENGTH]
)
{
    uint16_t powerState = 1U << BATTERY_PRESENT_SHIFT;

    if (charging) {
        powerState |= WIRED_POWER_CONNECTED << WIRED_POWER_SHIFT;
        powerState |= CHARGE_STATE_CHARGING << CHARGE_STATE_SHIFT;
    } else {
        // ~CHG is high both when unplugged and when charging has completed,
        // so only active charging can be positively identified.
        powerState |= CHARGE_STATE_DISCHARGING_ACTIVE << CHARGE_STATE_SHIFT;
    }

    powerState |= chargeLevelForPercentage(percentage) << CHARGE_LEVEL_SHIFT;

    // No optional Battery Level Status fields follow the mandatory Flags and
    // little-endian Power State fields. Percentage is exposed separately by
    // the mandatory 0x2A19 Battery Level characteristic.
    status[0] = 0;
    status[1] = static_cast<uint8_t>(powerState & 0xFF);
    status[2] = static_cast<uint8_t>(powerState >> 8);
}

void publishBatteryLevel(uint8_t percentage, bool sendNotification)
{
    if (!batteryService.write(percentage)) {
        diagnostics::fatalError(F("could not update BLE Battery Level"));
    }

    if (sendNotification && Bluefruit.connected()) {
        // A false result also means the phone has not subscribed, which is
        // valid; the freshly written value remains available via GATT Read.
        batteryService.notify(percentage);
    }
}

void publishBatteryLevelStatus(bool sendNotification)
{
    uint8_t status[BATTERY_LEVEL_STATUS_LENGTH];
    encodeBatteryLevelStatus(currentBatteryLevel, currentCharging, status);

    if (batteryLevelStatus.write(status, sizeof(status)) != sizeof(status)) {
        diagnostics::fatalError(F("could not update BLE Battery Level Status"));
    }

    if (sendNotification && Bluefruit.connected()) {
        batteryLevelStatus.notify(status, sizeof(status));
    }
}

void beginBatteryService()
{
    if (batteryService.begin() != ERROR_NONE) {
        diagnostics::fatalError(F("BLE Battery Service initialization failed"));
    }

    batteryLevelStatus.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    batteryLevelStatus.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    batteryLevelStatus.setFixedLen(BATTERY_LEVEL_STATUS_LENGTH);
    if (batteryLevelStatus.begin() != ERROR_NONE) {
        diagnostics::fatalError(F("BLE Battery Level Status initialization failed"));
    }

    currentBatteryLevel = batteryManager::readPercentage();
    currentCharging = batteryManager::isCharging();
    publishBatteryLevel(currentBatteryLevel, false);
    publishBatteryLevelStatus(false);

    const unsigned long now = millis();
    nextBatteryLevelUpdateAt = now + BATTERY_LEVEL_INTERVAL_MS;
    nextChargingStatusUpdateAt = now + CHARGING_STATUS_INTERVAL_MS;

    Serial.print(F("[BATTERY BLE] Initial state: "));
    Serial.print(currentBatteryLevel);
    Serial.print(F("%, "));
    Serial.println(currentCharging ? F("charging") : F("not charging"));
    Serial.println(F("[BATTERY BLE] Service 0x180F, Level 0x2A19, Status 0x2BED"));
}

void updateBatteryService(unsigned long now)
{
    if (static_cast<long>(now - nextChargingStatusUpdateAt) >= 0) {
        nextChargingStatusUpdateAt = now + CHARGING_STATUS_INTERVAL_MS;

        const bool charging = batteryManager::isCharging();
        if (charging != currentCharging) {
            currentCharging = charging;
            publishBatteryLevelStatus(true);

            Serial.print(F("[BATTERY BLE] Charging state changed: "));
            Serial.println(currentCharging ? F("charging") : F("not charging"));
        }
    }

    if (static_cast<long>(now - nextBatteryLevelUpdateAt) >= 0) {
        nextBatteryLevelUpdateAt = now + BATTERY_LEVEL_INTERVAL_MS;

        const uint8_t percentage = batteryManager::readPercentage();
        if (percentage != currentBatteryLevel) {
            currentBatteryLevel = percentage;
            publishBatteryLevel(currentBatteryLevel, true);
            publishBatteryLevelStatus(true);

            Serial.print(F("[BATTERY BLE] Level changed: "));
            Serial.print(currentBatteryLevel);
            Serial.println('%');
        }
    }
}

void startAdvertising()
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    if (!Bluefruit.Advertising.addService(batteryService)) {
        diagnostics::fatalError(F("could not advertise BLE Battery Service"));
    }

    // BLEAncs is a client service, so this emits the 128-bit ANCS
    // Service Solicitation rather than pretending the accessory hosts ANCS.
    if (!Bluefruit.Advertising.addService(ancsClient)) {
        diagnostics::fatalError(F("could not add ANCS Service Solicitation"));
    }

    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);

    // Apple recommends an exact 20 ms fast interval, followed by one of its
    // listed slow intervals. 1022.5 ms is the battery-constrained choice.
    Bluefruit.Advertising.setInterval(
        FAST_ADVERTISING_INTERVAL,
        SLOW_ADVERTISING_INTERVAL
    );
    Bluefruit.Advertising.setFastTimeout(FAST_ADVERTISING_TIMEOUT_SECONDS);

    if (!Bluefruit.Advertising.start(0)) {
        diagnostics::fatalError(F("advertising did not start"));
    }

    Serial.println(F("[BLE] Advertising started"));
    Serial.println(F("[BLE] Fast interval 20 ms for 30 s, then 1022.5 ms"));
    Serial.println(F("[ANCS] 128-bit Service Solicitation is present"));
    Serial.println(F("[BLE] Open iOS Settings > Bluetooth and select Notifense"));
}

void connectCallback(uint16_t connectionHandle)
{
    lowPowerParameterRequestSent = false;
    connectionParameterRetryPending.store(false, std::memory_order_release);
    connectionParameterReportPending.store(false, std::memory_order_release);

    Serial.println();
    Serial.print(F("[BLE] Connected on handle "));
    Serial.println(connectionHandle);
    diagnostics::setLed(LED_GREEN, true);

    BLEConnection *connection = Bluefruit.Connection(connectionHandle);
    if (connection == nullptr) {
        Serial.println(F("[BLE] Connection object is unavailable"));
        signalBleError();
        return;
    }

    if (!connection->setTxPower(TX_POWER_DBM)) {
        Serial.println(F("[BLE] Could not set connection TX power"));
        signalBleError();
    }
    printConnectionParameters(connection);

    Serial.print(F("[ANCS] Discovering iPhone ANCS service... "));
    if (ancsClient.discover(connectionHandle)) {
        Serial.println(F("found"));
    } else {
        Serial.println(F("not found yet"));
        Serial.println(F("[ANCS] Pairing anyway; discovery will be retried after encryption"));
    }

    Serial.println(F("[SEC] Requesting bonded Just Works pairing"));
    Serial.println(F("[SEC] Accept the Pair request on the iPhone"));
    if (!connection->requestPairing()) {
        Serial.println(F("[SEC] SoftDevice rejected the pairing request"));
        signalBleError();
    }
}

void disconnectCallback(uint16_t connectionHandle, uint8_t reason)
{
    (void) connectionHandle;

    Serial.println();
    Serial.print(F("[BLE] Disconnected; reason 0x"));
    diagnostics::printHexByte(reason);
    Serial.println();
    Serial.println(F("[BLE] Advertising will restart automatically"));
    diagnostics::setLed(LED_GREEN, false);
    lowPowerParameterRequestSent = false;
    connectionParameterRetryPending.store(false, std::memory_order_release);
    connectionParameterReportPending.store(false, std::memory_order_release);
    immediateSearchLedPending.store(true, std::memory_order_release);
}

void pairingCompleteCallback(uint16_t connectionHandle, uint8_t authStatus)
{
    Serial.print(F("[SEC] Pairing result on handle "));
    Serial.print(connectionHandle);
    Serial.print(F(": 0x"));
    diagnostics::printHexByte(authStatus);

    if (authStatus == BLE_GAP_SEC_STATUS_SUCCESS) {
        Serial.println(F(" (success; bond keys saved in internal flash)"));
        diagnostics::setLed(LED_GREEN, true);
    } else {
        Serial.println(F(" (failed)"));
        Serial.println(F("[SEC] Forget the device on iPhone, send C while disconnected, and retry"));
        signalBleError();
    }
}

void connectionSecuredCallback(uint16_t connectionHandle)
{
    BLEConnection *connection = Bluefruit.Connection(connectionHandle);
    if (connection == nullptr) {
        Serial.println(F("[SEC] Secured callback has no connection object"));
        signalBleError();
        return;
    }

    if (!connection->secured()) {
        Serial.println(F("[SEC] Stored key was rejected; requesting a fresh pairing"));
        if (!connection->requestPairing()) {
            Serial.println(F("[SEC] SoftDevice rejected the recovery pairing request"));
            signalBleError();
        }
        return;
    }

    Serial.println(F("[SEC] Link encrypted"));
    printConnectionParameters(connection);

    if (!ancsClient.discovered()) {
        Serial.print(F("[ANCS] Retrying service discovery... "));
        if (ancsClient.discover(connectionHandle)) {
            Serial.println(F("found"));
        } else {
            Serial.println(F("not found"));
            Serial.println(F("[ANCS] iOS did not publish ANCS on this connection"));
            signalBleError();
            return;
        }
    }

    Serial.print(F("[ANCS] Enabling Notification Source and Data Source... "));
    if (ancsClient.enableNotification()) {
        Serial.println(F("enabled"));
        Serial.println(F("[ANCS] Approve Share System Notifications on the iPhone if prompted"));
        Serial.println(F("[ANCS] Ready for iOS notifications"));
        requestLowPowerConnectionParameters(connectionHandle);
        diagnostics::blinkLed(LED_BLUE, 3);
    } else {
        Serial.println(F("failed"));
        signalBleError();
    }
}

void notificationCallback(AncsNotification_t *notification)
{
    if (notification == nullptr) {
        return;
    }

    diagnostics::blinkLed(LED_BLUE, 2);
    notificationProcessor::processNotification(ancsClient, notification);
}

} // namespace

void begin()
{
    // A large MTU moves occasional ANCS attributes in fewer packets. Idle
    // radio duty cycle is controlled by the GAP intervals configured below.
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    if (!Bluefruit.begin()) {
        diagnostics::fatalError(F("Bluefruit stack initialization failed"));
    }

    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(TX_POWER_DBM);
    Bluefruit.setName(DEVICE_NAME);

    if (
        !Bluefruit.Periph.setConnIntervalMS(
            CONNECTION_INTERVAL_MIN_MS,
            CONNECTION_INTERVAL_MAX_MS
        )
        || !Bluefruit.Periph.setConnSlaveLatency(CONNECTION_SLAVE_LATENCY)
        || !Bluefruit.Periph.setConnSupervisionTimeoutMS(
            CONNECTION_SUPERVISION_TIMEOUT_MS
        )
    ) {
        diagnostics::fatalError(F("BLE low-power connection setup failed"));
    }

    // Explicitly select Just Works capabilities. Bonding is enabled by default
    // and the Bluefruit stack stores the generated keys in InternalFS.
    Bluefruit.Security.setIOCaps(false, false, false);
    Bluefruit.Security.setPairCompleteCallback(pairingCompleteCallback);
    Bluefruit.Security.setSecuredCallback(connectionSecuredCallback);

    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    beginBatteryService();

    if (!ancsClient.begin()) {
        diagnostics::fatalError(F("ANCS client initialization failed"));
    }
    ancsClient.setNotificationCallback(notificationCallback);

    Serial.print(F("[BLE] Device name: "));
    Serial.println(DEVICE_NAME);
    Serial.println(F("[BLE] Preferred link: 150-165 ms, latency 4, timeout 6000 ms"));
    Serial.println(F("[BLE] TX power: 0 dBm"));
    Serial.println(F("[SEC] Bond keys persist across reset"));
    Serial.println(F("[SEC] Send C while disconnected to clear the device bond store"));

    startAdvertising();
    const unsigned long now = millis();
    nextStatusAt = now + SEARCH_LED_INTERVAL_MS;
    nextLowBatteryLedAt = now + LOW_BATTERY_LED_INTERVAL_MS;
}

void update()
{
    const unsigned long now = millis();
    updateBatteryService(now);

    if (
        connectionParameterRetryPending.load(std::memory_order_acquire)
        && static_cast<long>(now - connectionParameterRetryAt) >= 0
    ) {
        connectionParameterRetryPending.store(
            false,
            std::memory_order_release
        );
        submitLowPowerConnectionParameters(
            connectionParameterRetryHandle,
            false
        );
    }

    if (
        connectionParameterReportPending.load(std::memory_order_acquire)
        && static_cast<long>(now - connectionParameterReportAt) >= 0
    ) {
        connectionParameterReportPending.store(
            false,
            std::memory_order_release
        );
        BLEConnection *connection = Bluefruit.Connection(
            connectionParameterReportHandle
        );
        if (connection != nullptr && connection->connected()) {
            Serial.println(F("[BLE] Link state after low-power request:"));
            printConnectionParameters(connection);
            if (!connectionUsesLowPowerParameters(connection)) {
                Serial.println(F("[BLE] Phone selected different parameters; not retrying"));
            }
        }
    }

    if (static_cast<long>(now - nextLowBatteryLedAt) >= 0) {
        nextLowBatteryLedAt = now + LOW_BATTERY_LED_INTERVAL_MS;

        if (currentBatteryLevel < LOW_BATTERY_PERCENT) {
            Serial.print(F("[BATTERY] Low battery warning: "));
            Serial.print(currentBatteryLevel);
            Serial.println('%');
            diagnostics::blinkLed(
                LED_RED,
                1,
                diagnostics::LedFeedback::Essential
            );
        }
    }

    if (
        immediateSearchLedPending.exchange(
            false,
            std::memory_order_acq_rel
        )
        || static_cast<long>(now - nextStatusAt) >= 0
    ) {
        nextStatusAt = now + SEARCH_LED_INTERVAL_MS;

        if (!Bluefruit.connected()) {
            Serial.println(F("[BLE] Advertising; waiting for iPhone connection"));
            diagnostics::blinkLed(
                LED_BLUE,
                1,
                diagnostics::LedFeedback::Essential
            );
        }
    }
}

void clearBonds()
{
    if (Bluefruit.connected()) {
        Serial.println(F("[SEC] Disconnect before clearing bonds"));
        return;
    }

    Bluefruit.Periph.clearBonds();
    Serial.println(F("[SEC] Device bond storage cleared"));
    Serial.println(F("[SEC] Also use Forget This Device on the iPhone before pairing again"));
    diagnostics::blinkLed(LED_RED, 2);
}

} // namespace bleManager
