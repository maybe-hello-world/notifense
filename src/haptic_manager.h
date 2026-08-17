#pragma once

#include <stdint.h>

namespace hapticManager {

void begin();
void playEffect(uint8_t effectId);
void update();

} // namespace hapticManager
