#pragma once

#include "string.h"

namespace AP_HAL {

/* Toplevel pure virtual class Hal.*/
class HAL;
class Device;

/* Toplevel class names for drivers: */
class UARTDriver;
class I2CDevice;
class SPIDevice;
class Scheduler;
class Semaphore;
class Util;

// Must be implemented by the concrete HALs.
const HAL& get_HAL();
}
