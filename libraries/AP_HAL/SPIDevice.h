#pragma once

#include "AP_HAL_Namespace.h"
#include "Device.h"

namespace AP_HAL {
class SPIDevice : public Device {
public:
    SPIDevice() : Device(BUS_TYPE_SPI) { }

    virtual ~SPIDevice() { }
    /* Device implementation */

	virtual bool initialize() override  = 0;
    virtual bool set_speed(Device::Speed speed) override = 0;
	
	/* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;
	
	virtual bool writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)  override = 0;
	virtual bool readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)  override = 0;

};
}
