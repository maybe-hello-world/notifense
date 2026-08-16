#include "ble_manager.h"

#include <Arduino.h>
#include <bluefruit.h>

#include "diagnostics.h"
#include "notification_processor.h"

namespace bleManager {
namespace {

constexpr char DEVICE_NAME[] = "Notifense";
constexpr unsigned long STATUS_INTERVAL_MS = 5000;

BLEAncs ancsClient;
unsigned long nextStatusAt = 0;

void connectCallback(uint16_t connectionHandle);
void disconnectCallback(uint16_t connectionHandle, uint8_t reason);
void pairingCompleteCallback(uint16_t connectionHandle, uint8_t authStatus);
void connectionSecuredCallback(uint16_t connectionHandle);
void notificationCallback(AncsNotification_t *notification);

void startAdvertising()
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // BLEAncs is a client service, so this emits the 128-bit ANCS
    // Service Solicitation rather than pretending the accessory hosts ANCS.
    if (!Bluefruit.Advertising.addService(ancsClient)) {
        diagnostics::fatalError(F("could not add ANCS Service Solicitation"));
    }

    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);

    // Apple's recommended discovery pattern: 20 ms for 30 seconds, then
    // 152.5 ms indefinitely.
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);

    if (!Bluefruit.Advertising.start(0)) {
        diagnostics::fatalError(F("advertising did not start"));
    }

    Serial.println(F("[BLE] Advertising started"));
    Serial.println(F("[BLE] Fast interval 20 ms for 30 s, then 152.5 ms"));
    Serial.println(F("[ANCS] 128-bit Service Solicitation is present"));
    Serial.println(F("[BLE] Open iOS Settings > Bluetooth and select Notifense"));
}

void connectCallback(uint16_t connectionHandle)
{
    Serial.println();
    Serial.print(F("[BLE] Connected on handle "));
    Serial.println(connectionHandle);
    diagnostics::setLed(LED_GREEN, true);

    BLEConnection *connection = Bluefruit.Connection(connectionHandle);
    if (connection == nullptr) {
        Serial.println(F("[BLE] Connection object is unavailable"));
        diagnostics::blinkLed(LED_RED, 2);
        return;
    }

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
        diagnostics::blinkLed(LED_RED, 2);
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
    diagnostics::blinkLed(LED_RED, 2);
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
        diagnostics::blinkLed(LED_RED, 3);
    }
}

void connectionSecuredCallback(uint16_t connectionHandle)
{
    BLEConnection *connection = Bluefruit.Connection(connectionHandle);
    if (connection == nullptr) {
        Serial.println(F("[SEC] Secured callback has no connection object"));
        diagnostics::blinkLed(LED_RED, 2);
        return;
    }

    if (!connection->secured()) {
        Serial.println(F("[SEC] Stored key was rejected; requesting a fresh pairing"));
        connection->requestPairing();
        return;
    }

    Serial.println(F("[SEC] Link encrypted"));

    if (!ancsClient.discovered()) {
        Serial.print(F("[ANCS] Retrying service discovery... "));
        if (ancsClient.discover(connectionHandle)) {
            Serial.println(F("found"));
        } else {
            Serial.println(F("not found"));
            Serial.println(F("[ANCS] iOS did not publish ANCS on this connection"));
            diagnostics::blinkLed(LED_RED, 2);
            return;
        }
    }

    Serial.print(F("[ANCS] Enabling Notification Source and Data Source... "));
    if (ancsClient.enableNotification()) {
        Serial.println(F("enabled"));
        Serial.println(F("[ANCS] Approve Share System Notifications on the iPhone if prompted"));
        Serial.println(F("[ANCS] Ready for iOS notifications"));
        diagnostics::blinkLed(LED_BLUE, 3);
    } else {
        Serial.println(F("failed"));
        diagnostics::blinkLed(LED_RED, 3);
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
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    if (!Bluefruit.begin()) {
        diagnostics::fatalError(F("Bluefruit stack initialization failed"));
    }

    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(4);
    Bluefruit.setName(DEVICE_NAME);

    // Explicitly select Just Works capabilities. Bonding is enabled by default
    // and the Bluefruit stack stores the generated keys in InternalFS.
    Bluefruit.Security.setIOCaps(false, false, false);
    Bluefruit.Security.setPairCompleteCallback(pairingCompleteCallback);
    Bluefruit.Security.setSecuredCallback(connectionSecuredCallback);

    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    if (!ancsClient.begin()) {
        diagnostics::fatalError(F("ANCS client initialization failed"));
    }
    ancsClient.setNotificationCallback(notificationCallback);

    Serial.print(F("[BLE] Device name: "));
    Serial.println(DEVICE_NAME);
    Serial.println(F("[SEC] Bond keys persist across reset"));
    Serial.println(F("[SEC] Send C while disconnected to clear the device bond store"));

    startAdvertising();
    nextStatusAt = millis() + STATUS_INTERVAL_MS;
}

void update()
{
    const unsigned long now = millis();
    if (static_cast<long>(now - nextStatusAt) >= 0) {
        nextStatusAt = now + STATUS_INTERVAL_MS;

        if (!Bluefruit.connected()) {
            Serial.println(F("[BLE] Advertising; waiting for iPhone connection"));
            diagnostics::blinkLed(LED_RED, 1);
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
