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
constexpr uint16_t CONNECTION_SLAVE_LATENCY = 4;
constexpr uint16_t CONNECTION_SUPERVISION_TIMEOUT_MS = 6000;
constexpr unsigned long ANCS_RECONNECT_DELAY_MS = 1000;
constexpr unsigned long ADVERTISING_RESTART_DELAY_MS = 1000;
constexpr unsigned long CLEANUP_DISCONNECT_RETRY_DELAY_MS = 1000;
constexpr uint8_t ANCS_MAX_SETUP_ATTEMPTS = 3;
constexpr uint8_t CLEANUP_DISCONNECT_MAX_ATTEMPTS = 3;
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

class CleanupAwareBLEAncs : public BLEAncs {
  public:
    void prepareForDisconnectCleanup(uint16_t connectionHandle)
    {
        // BLEAncs temporarily invalidates this parent handle during discovery.
        // Restoring it lets Bluefruit's normal disconnect path clear any
        // partially discovered child characteristics.
        _conn_hdl = connectionHandle;
    }
};

CleanupAwareBLEAncs ancsClient;
BLEBas batteryService;
BLECharacteristic batteryLevelStatus(BATTERY_LEVEL_STATUS_UUID);

enum class AncsSetupState : uint8_t {
    Idle,
    Waiting,
    Ready,
    Failed,
};

unsigned long nextStatusAt = 0;
unsigned long nextLowBatteryLedAt = 0;
unsigned long nextBatteryLevelUpdateAt = 0;
unsigned long nextChargingStatusUpdateAt = 0;
uint8_t currentBatteryLevel = 0;
bool currentCharging = false;
std::atomic<bool> immediateSearchLedPending(false);

constexpr uint32_t NO_CONNECTION_STATE = 0x0000FFFFUL;
std::atomic<uint32_t> activeConnectionState(NO_CONNECTION_STATE);
std::atomic<uint32_t> encryptedConnectionState(NO_CONNECTION_STATE);
std::atomic<uint32_t> readyConnectionState(NO_CONNECTION_STATE);
uint16_t connectionGeneration = 0;
uint32_t managedConnectionState = NO_CONNECTION_STATE;
AncsSetupState ancsSetupState = AncsSetupState::Idle;
unsigned long nextAncsSetupAt = 0;
uint8_t ancsSetupAttemptCount = 0;
bool cleanupOnNextConnection = false;
uint8_t cleanupDisconnectAttemptCount = 0;
unsigned long cleanupDisconnectRetryAt = 0;
bool recoveryDisconnectExpected = false;
bool advertisingRestartPending = false;
unsigned long advertisingRestartAt = 0;
std::atomic<bool> notificationProcessing(false);

void notificationCallback(AncsNotification_t *notification);
void bleEventCallback(ble_evt_t *event);

void signalBleError()
{
    diagnostics::blinkLed(
        LED_RED,
        1,
        diagnostics::LedFeedback::Essential
    );
}

uint32_t encodeConnectionState(
    uint16_t generation,
    uint16_t connectionHandle
)
{
    return static_cast<uint32_t>(generation) << 16 | connectionHandle;
}

uint16_t connectionHandleFromState(uint32_t state)
{
    return static_cast<uint16_t>(state & 0xFFFFU);
}

void beginConnectionSession(uint16_t connectionHandle)
{
    readyConnectionState.store(
        NO_CONNECTION_STATE,
        std::memory_order_relaxed
    );
    encryptedConnectionState.store(
        NO_CONNECTION_STATE,
        std::memory_order_relaxed
    );
    activeConnectionState.store(
        encodeConnectionState(++connectionGeneration, connectionHandle),
        std::memory_order_release
    );
}

void endConnectionSession(uint16_t connectionHandle)
{
    const uint32_t currentState = activeConnectionState.load(
        std::memory_order_acquire
    );
    if (connectionHandleFromState(currentState) != connectionHandle) {
        return;
    }

    readyConnectionState.store(
        NO_CONNECTION_STATE,
        std::memory_order_relaxed
    );
    encryptedConnectionState.store(
        NO_CONNECTION_STATE,
        std::memory_order_relaxed
    );
    activeConnectionState.store(
        encodeConnectionState(
            ++connectionGeneration,
            BLE_CONN_HANDLE_INVALID
        ),
        std::memory_order_release
    );
}

bool connectionStateMatches(uint32_t expectedState)
{
    return activeConnectionState.load(std::memory_order_acquire)
        == expectedState;
}

void requestAncsRecoveryReconnect(
    uint32_t connectionState,
    const __FlashStringHelper *reason
)
{
    ancsSetupState = AncsSetupState::Failed;
    Serial.print(F("[ANCS] Setup failed: "));
    Serial.println(reason);

    const uint16_t connectionHandle = connectionHandleFromState(
        connectionState
    );
    ancsClient.prepareForDisconnectCleanup(connectionHandle);
    if (ancsSetupAttemptCount >= ANCS_MAX_SETUP_ATTEMPTS) {
        if (!connectionStateMatches(connectionState)) {
            cleanupOnNextConnection = true;
            cleanupDisconnectAttemptCount = 0;
            cleanupDisconnectRetryAt = 0;
        }
        Serial.println(F("[ANCS] Retry limit reached; send R and Enter"));
        signalBleError();
        return;
    }

    Serial.print(F("[ANCS] Reconnecting for retry "));
    Serial.print(ancsSetupAttemptCount + 1);
    Serial.print('/');
    Serial.println(ANCS_MAX_SETUP_ATTEMPTS);
    cleanupOnNextConnection = true;
    cleanupDisconnectAttemptCount = 0;
    cleanupDisconnectRetryAt = 0;
    if (!connectionStateMatches(connectionState)) {
        // The disconnect already happened while discovery was blocked. Use
        // one short connection only to let Bluefruit clean stale child handles.
        Serial.println(F("[ANCS] Cleanup queued for the next connection"));
    }
}

void updateCleanupDisconnect(
    uint32_t connectionState,
    unsigned long now
)
{
    if (
        !cleanupOnNextConnection
        || recoveryDisconnectExpected
        || static_cast<long>(now - cleanupDisconnectRetryAt) < 0
        || notificationProcessing.load(std::memory_order_acquire)
    ) {
        return;
    }

    const uint16_t connectionHandle = connectionHandleFromState(
        connectionState
    );
    if (
        connectionHandle == BLE_CONN_HANDLE_INVALID
        || !connectionStateMatches(connectionState)
    ) {
        return;
    }
    if (
        cleanupDisconnectAttemptCount
            >= CLEANUP_DISCONNECT_MAX_ATTEMPTS
    ) {
        return;
    }

    ancsClient.prepareForDisconnectCleanup(connectionHandle);
    ++cleanupDisconnectAttemptCount;
    recoveryDisconnectExpected = true;
    const uint32_t result = sd_ble_gap_disconnect(
        connectionHandle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION
    );
    if (result == NRF_SUCCESS) {
        cleanupDisconnectAttemptCount = 0;
        return;
    }

    recoveryDisconnectExpected = false;
    cleanupDisconnectRetryAt = now + CLEANUP_DISCONNECT_RETRY_DELAY_MS;
    Serial.print(F("[ANCS] Cleanup disconnect attempt failed: "));
    Serial.println(result);
    if (
        cleanupDisconnectAttemptCount
            >= CLEANUP_DISCONNECT_MAX_ATTEMPTS
    ) {
        Serial.println(F("[ANCS] Disconnect retry limit reached; send R and Enter"));
        signalBleError();
    }
}

void startManagedConnection(
    uint32_t connectionState,
    unsigned long now
)
{
    managedConnectionState = connectionState;
    ancsSetupState = AncsSetupState::Idle;
    nextAncsSetupAt = 0;

    const uint16_t connectionHandle = connectionHandleFromState(
        connectionState
    );

    // Disconnected state.
    if (connectionHandle == BLE_CONN_HANDLE_INVALID) {
        if (recoveryDisconnectExpected) {
            cleanupOnNextConnection = false;
            cleanupDisconnectAttemptCount = 0;
        } else {
            ancsSetupAttemptCount = 0;
        }

        recoveryDisconnectExpected = false;
        diagnostics::setLed(LED_GREEN, false);
        immediateSearchLedPending.store(true, std::memory_order_release);

        advertisingRestartPending = true;
        advertisingRestartAt = now + ADVERTISING_RESTART_DELAY_MS;

        Serial.println();
        Serial.println(
            F("[BLE] Disconnected; advertising restarts in 2 seconds")
        );
        return;
    }

    // Connected state.
    advertisingRestartPending = false;
    diagnostics::setLed(LED_GREEN, true);

    Serial.println();
    Serial.print(F("[BLE] Connected on handle "));
    Serial.println(connectionHandle);

    // If ANCS recovery requested a cleanup-only connection, don't perform
    // normal setup. updateCleanupDisconnect() will terminate this connection.
    if (cleanupOnNextConnection) {
        Serial.println(F("[ANCS] Running one cleanup-only connection"));
        cleanupDisconnectAttemptCount = 0;
        cleanupDisconnectRetryAt = now;
        return;
    }

    // Set connection-specific TX power.
    const uint32_t txPowerResult = sd_ble_gap_tx_power_set(
        BLE_GAP_TX_POWER_ROLE_CONN,
        connectionHandle,
        TX_POWER_DBM
    );

    if (
        txPowerResult != NRF_SUCCESS
        && txPowerResult != BLE_ERROR_INVALID_CONN_HANDLE
    ) {
        Serial.println(F("[BLE] Could not set connection TX power"));
        signalBleError();
    }

    // Use Bluefruit's public BLEConnection API instead of calling the
    // internal Security._authenticate() method directly.
    BLEConnection *connection = Bluefruit.Connection(connectionHandle);

    if (connection == nullptr) {
        Serial.println(F("[SEC] Could not get BLE connection object"));
        signalBleError();
        return;
    }

    if (connection->secured()) {
        // This commonly happens on reconnect when the existing bond has
        // already been used to restore encryption.
        Serial.println(F("[SEC] Link already secured"));
    } else {
        Serial.println(F("[SEC] Requesting bonded Just Works pairing"));
        Serial.println(F("[SEC] Accept the Pair request on the iPhone"));

        if (!connection->requestPairing()) {
            Serial.println(F("[SEC] Could not start pairing"));
            signalBleError();
        }
    }
}
void updateAncsSetup(unsigned long now)
{
    const uint32_t connectionState = activeConnectionState.load(
        std::memory_order_acquire
    );
    if (connectionState != managedConnectionState) {
        startManagedConnection(connectionState, now);
    }

    const uint16_t connectionHandle = connectionHandleFromState(
        connectionState
    );
    if (connectionHandle == BLE_CONN_HANDLE_INVALID) {
        return;
    }
    if (cleanupOnNextConnection) {
        updateCleanupDisconnect(connectionState, now);
        return;
    }

    if (
        ancsSetupState == AncsSetupState::Idle
        && encryptedConnectionState.load(std::memory_order_acquire)
            == connectionState
    ) {
        if (ancsSetupAttemptCount >= ANCS_MAX_SETUP_ATTEMPTS) {
            ancsSetupState = AncsSetupState::Failed;
            Serial.println(F("[ANCS] Retry limit reached; send R and Enter"));
            signalBleError();
            return;
        }

        ancsSetupState = AncsSetupState::Waiting;
        nextAncsSetupAt = now + ANCS_RECONNECT_DELAY_MS;
        Serial.println(F("[SEC] Link encrypted"));
        Serial.println(F("[ANCS] Waiting 5 seconds before setup"));
    }

    if (
        ancsSetupState != AncsSetupState::Waiting
        || static_cast<long>(now - nextAncsSetupAt) < 0
        || notificationProcessing.load(std::memory_order_acquire)
        || !connectionStateMatches(connectionState)
    ) {
        return;
    }

    // Mark the attempt before entering blocking library calls. Recovery uses
    // a fresh BLE connection rather than retrying stale handles in place.
    ancsSetupState = AncsSetupState::Failed;
    ++ancsSetupAttemptCount;

    Serial.print(F("[ANCS] Discovering after reconnect delay... "));
    if (!ancsClient.discovered()) {
        const bool discovered = ancsClient.discover(connectionHandle);
        if (!connectionStateMatches(connectionState)) {
            requestAncsRecoveryReconnect(
                connectionState,
                F("discovery was interrupted")
            );
            return;
        }
        if (!discovered) {
            Serial.println(F("failed"));
            requestAncsRecoveryReconnect(
                connectionState,
                F("characteristic discovery failed")
            );
            return;
        }
    }
    if (!connectionStateMatches(connectionState)) {
        requestAncsRecoveryReconnect(
            connectionState,
            F("discovery was interrupted")
        );
        return;
    }
    Serial.println(F("found"));

    Serial.print(F("[ANCS] Enabling notification sources... "));
    if (!ancsClient.enableNotification()) {
        if (!connectionStateMatches(connectionState)) {
            return;
        }
        Serial.println(F("failed"));
        requestAncsRecoveryReconnect(
            connectionState,
            F("notification subscription failed")
        );
        return;
    }
    if (!connectionStateMatches(connectionState)) {
        return;
    }

    Serial.println(F("enabled"));
    Serial.print(F("[ANCS] Ready for iOS notifications at "));
    Serial.print(millis());
    Serial.println(F(" ms"));
    ancsSetupState = AncsSetupState::Ready;
    readyConnectionState.store(
        connectionState,
        std::memory_order_release
    );
    ancsSetupAttemptCount = 0;
    diagnostics::blinkLed(LED_BLUE, 3);
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
    Bluefruit.Advertising.restartOnDisconnect(false);

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

void bleEventCallback(ble_evt_t *event)
{
    if (event == nullptr) {
        return;
    }

    const uint16_t connectionHandle = event->evt.common_evt.conn_handle;
    switch (event->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            beginConnectionSession(connectionHandle);
            return;

        case BLE_GAP_EVT_DISCONNECTED: {
            const uint8_t reason =
                event->evt.gap_evt.params.disconnected.reason;

            Serial.print(F("[BLE] Disconnected at "));
            Serial.print(millis());
            Serial.print(F(" ms; reason = 0x"));
            if (reason < 0x10) {
                Serial.print('0');
            }
            Serial.println(reason, HEX);

            endConnectionSession(connectionHandle);
            immediateSearchLedPending.store(true, std::memory_order_release);
            return;
        }

        case BLE_GAP_EVT_CONN_SEC_UPDATE: {
            const uint32_t connectionState = activeConnectionState.load(
                std::memory_order_acquire
            );
            if (
                connectionHandleFromState(connectionState)
                    != connectionHandle
            ) {
                return;
            }

            const ble_gap_conn_sec_mode_t &securityMode =
                event->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode;
            const bool secured = !(
                securityMode.sm == 1
                && securityMode.lv == 1
            );
            encryptedConnectionState.store(
                secured ? connectionState : NO_CONNECTION_STATE,
                std::memory_order_release
            );
            return;
        }

        default:
            return;
    }
}

void updateAdvertisingRestart(unsigned long now)
{
    if (
        !advertisingRestartPending
        || static_cast<long>(now - advertisingRestartAt) < 0
    ) {
        return;
    }

    advertisingRestartPending = false;
    if (
        connectionHandleFromState(
            activeConnectionState.load(std::memory_order_acquire)
        ) != BLE_CONN_HANDLE_INVALID
    ) {
        return;
    }

    if (!Bluefruit.Advertising.start(0)) {
        advertisingRestartPending = true;
        advertisingRestartAt = now + ADVERTISING_RESTART_DELAY_MS;
        Serial.println(F("[BLE] Advertising restart failed; retrying in 2 seconds"));
        signalBleError();
        return;
    }

    Serial.println(F("[BLE] Advertising restarted"));
}

void notificationCallback(AncsNotification_t *notification)
{
    if (notification == nullptr) {
        return;
    }

    const uint32_t connectionState = activeConnectionState.load(
        std::memory_order_acquire
    );
    if (
        connectionHandleFromState(connectionState) == BLE_CONN_HANDLE_INVALID
        || readyConnectionState.load(std::memory_order_acquire)
            != connectionState
    ) {
        return;
    }

    notificationProcessing.store(true, std::memory_order_release);
    if (
        !connectionStateMatches(connectionState)
        || readyConnectionState.load(std::memory_order_acquire)
            != connectionState
    ) {
        notificationProcessing.store(false, std::memory_order_release);
        return;
    }
    diagnostics::blinkLed(LED_BLUE, 2);
    notificationProcessor::processNotification(ancsClient, notification);
    notificationProcessing.store(false, std::memory_order_release);
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
    Bluefruit.setEventCallback(bleEventCallback);

    beginBatteryService();

    if (!ancsClient.begin()) {
        diagnostics::fatalError(F("ANCS client initialization failed"));
    }
    ancsClient.setNotificationCallback(notificationCallback);

    Serial.print(F("[BLE] Device name: "));
    Serial.println(DEVICE_NAME);
    Serial.println(F("[BLE] Preferred link: 150-165 ms, latency 4, timeout 6000 ms"));
    Serial.println(F("[BLE] TX power: 0 dBm"));
    Serial.println(F("[SEC] Bond keys persist across BLE reconnects and power cycles"));
    Serial.println(F("[SEC] Send C then Enter while disconnected to clear bonds"));
    Serial.println(F("[ANCS] Send R then Enter for a clean BLE retry"));

    startAdvertising();
    const unsigned long now = millis();
    nextStatusAt = now + SEARCH_LED_INTERVAL_MS;
    nextLowBatteryLedAt = now + LOW_BATTERY_LED_INTERVAL_MS;
}

void update()
{
    const unsigned long now = millis();
    updateBatteryService(now);
    updateAncsSetup(now);
    updateAdvertisingRestart(now);

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

        if (
            connectionHandleFromState(
                activeConnectionState.load(std::memory_order_acquire)
            ) == BLE_CONN_HANDLE_INVALID
        ) {
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
    if (
        connectionHandleFromState(
            activeConnectionState.load(std::memory_order_acquire)
        ) != BLE_CONN_HANDLE_INVALID
    ) {
        Serial.println(F("[SEC] Disconnect before clearing bonds"));
        return;
    }

    Bluefruit.Periph.clearBonds();
    Serial.println(F("[SEC] Device bond storage cleared"));
    Serial.println(F("[SEC] Also use Forget This Device on the iPhone before pairing again"));
    diagnostics::blinkLed(LED_RED, 2);
}

void retryAncsConnection()
{
    readyConnectionState.store(
        NO_CONNECTION_STATE,
        std::memory_order_release
    );
    ancsSetupAttemptCount = 0;
    cleanupOnNextConnection = true;
    cleanupDisconnectAttemptCount = 0;
    cleanupDisconnectRetryAt = 0;
    const uint32_t connectionState = activeConnectionState.load(
        std::memory_order_acquire
    );
    const uint16_t connectionHandle = connectionHandleFromState(
        connectionState
    );
    if (connectionHandle == BLE_CONN_HANDLE_INVALID) {
        Serial.println(F("[ANCS] Cleanup armed; ANCS retries on the following connection"));
        return;
    }

    Serial.println(F("[ANCS] Manual BLE reconnect requested"));
    ancsSetupState = AncsSetupState::Failed;
}

} // namespace bleManager
