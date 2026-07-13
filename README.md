# Notifense

Just my personal notification haptic wearable project. Totally not a vibrator.

## Hardware

- Seeed Studio XIAO nRF52840 Sense
- DRV2605L haptic driver
- LRA motor
- 100mah battery

## How It Works

On startup, the firmware initializes the LRA driver, puts it in standby, and advertises as `Notifense`. After a phone connects, it can write one-byte haptic commands with or without a BLE response.

- Values `1..123` play the matching DRV2605L effect.
- Value `0` is ignored.
- Effects are queued, played in order, and the driver returns to standby afterward.
- If the phone disconnects or moves out of range, advertising resumes automatically.

## Status LEDs

- Green once: phone connected.
- Red twice: DRV or BLE initialization failed; initialization is retried.
- In debug mode, blue indicates advertising or disconnection, and red indicates runtime errors.
- LEDs remain off between events. Serial logging is also disabled when `NOTIFENSE_DEBUG` is `0`.
