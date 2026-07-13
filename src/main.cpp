#include <HapticDrivers.hpp>
#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Haptic driver instance for DRV2605
HapticDriver_DRV2605 haptic;

const int blueLED = 13u;
const int greenLED = 12u;
const int redLED = 11u;
const int SENSOR_SDA = 4; // SDA pin for I2C
const int SENSOR_SCL = 5; // SCL pin for I2C

static const char *effectList[] = {
    "1 - Strong Click 100%", "2 - Strong Click 60%", "3 - Strong Click 30%",
    "4 - Sharp Click 100%", "5 - Sharp Click 60%", "6 - Sharp Click 30%",
    "7 - Soft Bump 100%", "8 - Soft Bump 60%", "9 - Soft Bump 30%",
    "10 - Double Click 100%", "11 - Double Click 60%", "12 - Triple Click 100%",
    "13 - Soft Fuzz 60%", "14 - Strong Buzz 100%", "15 - Alert 750ms 100%",
    "16 - Alert 1000ms 100%"
};

void printHelp()
{
    Serial.println("\n========== Commands ==========");
    Serial.println("1-16  : Play effect (ERM Library 1)");
    Serial.println("e     : List all effects (1-117)");
    Serial.println("l 1-6 : Select ROM library");
    Serial.println("a     : Toggle ERM/LRA mode");
    Serial.println("s     : Stop playback");
    Serial.println("r     : RTP test (0-255)");
    Serial.println("c     : Calibrate");
    Serial.println("h     : Help");
    Serial.println("==============================");
}

void setLED(int LED, bool state)
{
    digitalWrite(LED, state ? LOW : HIGH);
}

void blink(int LED, int times, int delayMs)
{
    for (int i = 0; i < times; ++i) {
        setLED(LED, true);
        delay(delayMs);
        setLED(LED, false);
        delay(delayMs);
    }
}

void initBLE() {
    // begin initialization
    while (!BLE.begin()) {
        blink(blueLED, 1, 300);
        Serial.println("starting Bluetooth® Low Energy module failed!");
        delay(1000);
    }

    // set advertised local name and service UUID:
    BLE.setLocalName("LED");
    BLE.setAdvertisedService(ledService);

    // add the characteristic to the service
    ledService.addCharacteristic(switchCharacteristic);

    // add service
    BLE.addService(ledService);

    // set the initial value for the characeristic:
    switchCharacteristic.writeValue(0);

    // start advertising
    BLE.advertise();

    blink(blueLED, 1, 1000);
    Serial.println("BLE initialized");
}

void initHaptic() {
    while (!haptic.begin(Wire, DRV2605_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        // blink if we couldn't initialize the driver
        blink(redLED, 3, 300);
        Serial.println("[FATAL] DRV2605 init failed!");
        delay(1000);
    }

    blink(redLED, 1, 1000);
    Wire.begin();
    Serial.println("Haptic initialized");
}

void setup()
{
    pinMode(blueLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    blink(greenLED, 1, 300);

    Serial.begin(9600);
    while (!Serial);
    blink(greenLED, 1, 300);
    delay(500);

    initBLE();
    initHaptic();

    Serial.println("Everything initialized");
    blink(greenLED, 1, 1000);
    delay(10);
}

void loop()
{
    // listen for BLE peripherals to connect:
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central) {
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());
        setLED(greenLED, true);

        // while the central is still connected to peripheral:
        while (central.connected()) {
            if (switchCharacteristic.written()) {
                byte value = switchCharacteristic.value();
                Serial.print("Switch characteristic written: ");
                Serial.println(value);
                haptic.playEffect(value + 1);
            }
        }

        // when the central disconnects, print it out:
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
        setLED(greenLED, false);
    }

    delay(10);
}