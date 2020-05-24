#pragma once

#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <hal_drivers/hal_qspi.h>
    
class PX4::PX4SPIDevice : public AP_HAL::SPIDevice {
public:
	PX4SPIDevice();
    PX4SPIDevice(HAL_QSPI *_dev);

    virtual ~PX4SPIDevice();

	virtual bool initialize() override;
    bool set_speed(AP_HAL::Device::Speed speed) override;
	
	AP_HAL::Semaphore *get_semaphore() override
	{
		return &semaphore;
	}

	bool writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) override;
	bool readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) override;

private:
	HAL_QSPI *dev;
	PX4Semaphore semaphore;
	
    uint32_t frequency;
    char *pname;
    bool cs_forced;
};
