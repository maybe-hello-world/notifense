#pragma once

#include <stdint.h>

namespace batteryManager {

void begin();
float readVoltage();
uint8_t readPercentage();
bool isCharging();
void reportStatus();

} // namespace batteryManager
