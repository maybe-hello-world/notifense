# Repository Guidelines

## Project Structure & Module Organization

`src/main.cpp` is the deployable Arduino firmware. It initializes BLE and the DRV2605 haptic driver, then maps BLE characteristic values to haptic effects. `examples/` contains standalone hardware diagnostics: `ble_main.cpp`, `check_0x5A.cpp`, and `drv2605_test.cpp`. These examples are reference sketches, not part of the normal build. `platformio.ini` defines the single `xiaoblesense_arduinocore_mbed` environment and pins library dependencies. Treat `.pio/` and generated `.vscode/` metadata as local artifacts; do not commit them.

## Build, Upload, and Development Commands

Run commands from the repository root with PlatformIO Core installed:

- `pio run -e xiaoblesense_arduinocore_mbed` compiles the firmware.
- `pio run -e xiaoblesense_arduinocore_mbed -t upload` builds and flashes a connected XIAO nRF52840 Sense.
- `pio device monitor -b 9600` opens the firmware's serial console.
- `pio run -t clean` removes generated build output.

The recommended VS Code extension is PlatformIO IDE. Keep board, framework, and dependency changes in `platformio.ini`, not generated editor files.

## Coding Style & Naming Conventions

Use four-space indentation for C++ and match brace placement in the code you touch. Prefer `lowerCamelCase` for functions and variables (`initBLE`, `switchCharacteristic`) and `UPPER_SNAKE_CASE` for preprocessor constants and fixed hardware identifiers (`SENSOR_SDA`). Keep `setup()` and `loop()` readable by extracting focused helpers. Comments should explain hardware constraints or protocol behavior, not restate the code. No formatter or linter is configured, so avoid unrelated reformatting.

## Testing Guidelines

There is no automated test suite or coverage threshold. Every change must compile with `pio run` and be checked on the target board. Exercise affected BLE connection flows, characteristic writes, I2C initialization, haptic playback, and serial diagnostics as applicable. Use the sketches in `examples/` for focused manual checks, and document the sketch and hardware result in the pull request.

## Commit & Pull Request Guidelines

History currently uses brief lowercase subjects (`initial`, `readme`). Keep commits focused and use concise subjects such as `add ble reconnect handling`. Pull requests should explain the motivation, summarize behavior changes, link relevant issues, and list build plus on-device validation. Include serial logs or hardware photos when they clarify physical-device behavior.
