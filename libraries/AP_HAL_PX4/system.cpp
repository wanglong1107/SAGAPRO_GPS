#include <stdarg.h>
#include <stdio.h>

#include <hal_drivers/drv_hrt.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

extern const AP_HAL::HAL& hal;

//extern bool _px4_thread_should_exit;

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg)
{
	
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    return hrt_absolute_time();
}

uint64_t millis64()
{
    return micros64() / 1000;
}

} // namespace AP_HAL
