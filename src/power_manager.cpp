#include "power_manager.h"

#include <Arduino.h>
#include <Adafruit_SPIFlash.h>

#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport(
    PIN_QSPI_SCK,
    PIN_QSPI_CS,
    PIN_QSPI_IO0,
    PIN_QSPI_IO1,
    PIN_QSPI_IO2,
    PIN_QSPI_IO3
);

Adafruit_SPIFlash externalFlash(&flashTransport);
#endif

namespace powerManager {
namespace {

#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash externalFlash(&flashTransport);
#endif

void powerDownImu()
{
#ifdef PIN_LSM6DS3TR_C_POWER
    // Preload LOW before switching to output so the IMU never gets
    // accidentally powered during the transition.
    digitalWrite(PIN_LSM6DS3TR_C_POWER, LOW);
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);

#ifdef PIN_LSM6DS3TR_C_INT1
    // The IMU is unpowered, so keep its interrupt line from floating.
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN);
#endif

    Serial.println(F("[POWER] Sense IMU powered off"));
#endif
}

void powerDownMicrophone()
{
#ifdef PIN_PDM_PWR
    // The Seeed PDM library uses HIGH = microphone powered,
    // LOW = microphone off.
    digitalWrite(PIN_PDM_PWR, LOW);
    pinMode(PIN_PDM_PWR, OUTPUT);
#endif

#ifdef PIN_PDM_CLK
    // Never drive a clock into an unpowered microphone.
    digitalWrite(PIN_PDM_CLK, LOW);
    pinMode(PIN_PDM_CLK, OUTPUT);
#endif

#ifdef PIN_PDM_DIN
    // Avoid a floating digital input.
    pinMode(PIN_PDM_DIN, INPUT_PULLDOWN);
#endif

    Serial.println(F("[POWER] Sense microphone powered off"));
}

bool powerDownExternalFlash()
{
#if defined(EXTERNAL_FLASH_USE_QSPI)

    if (!externalFlash.begin()) {
        Serial.println(F("[POWER] Could not initialize external QSPI flash"));
        return false;
    }

    const uint32_t jedec = externalFlash.getJEDECID();

    Serial.print(F("[POWER] QSPI JEDEC: 0x"));
    Serial.println(jedec, HEX);

    // GD25Q16C deep-power-down command
    flashTransport.runCommand(0xB9);

    delay(1);

    externalFlash.end();

    Serial.println(F("[POWER] External QSPI flash sent to deep power-down"));
    return true;

#else
    Serial.println(F("[POWER] No external QSPI flash configured"));
    return true;
#endif
}

} // namespace

void begin()
{
    powerDownImu();
    powerDownMicrophone();
    powerDownExternalFlash();
}

} // namespace powerManager