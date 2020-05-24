#pragma once

#include <stdint.h>
namespace AP_HAL {
void init();

void panic(const char *errormsg);

uint32_t micros();
uint32_t millis();
uint64_t micros64();
uint64_t millis64();

} // namespace AP_HAL
