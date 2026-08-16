#pragma once

namespace batteryManager {

void begin();
float readVoltage();
bool isCharging();
void reportStatus();

} // namespace batteryManager
