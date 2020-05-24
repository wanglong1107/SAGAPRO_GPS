#pragma once

//#include <inttypes.h>
//#include <vector>

#include "AP_HAL_Namespace.h"
#include "Device.h"

namespace AP_HAL{
class I2CDevice : public Device{
public:
    I2CDevice() : Device(BUS_TYPE_I2C) { }

    virtual ~I2CDevice() {}
	
	virtual bool initialize() override  = 0;
    virtual bool set_speed(Device::Speed speed) override = 0;
	
	/* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

	virtual bool writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)  override = 0;
	virtual bool readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)  override = 0;
};
}
