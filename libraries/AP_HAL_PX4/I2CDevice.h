#pragma once

#include <inttypes.h>


#include "AP_HAL_PX4.h"
#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <hal_drivers/hal_i2c.h>

class PX4::PX4I2CDevice : public AP_HAL::I2CDevice {
public:
    PX4I2CDevice();
	PX4I2CDevice(HAL_I2C *_dev);
    ~PX4I2CDevice();

	virtual bool initialize() override;
    void set_address(uint8_t address) override { _address = address; }

    bool set_speed(enum Device::Speed speed) override;
	
	AP_HAL::Semaphore* get_semaphore() override
	{
		return &semaphore;
	}

	bool writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) override;
	bool readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) override;

private:
	HAL_I2C *dev;
	PX4Semaphore semaphore;
	
    static const uint8_t num_buses = 3;
    uint8_t _busnum;
    uint8_t _address;
    char *pname;
    bool _split_transfers;
};
