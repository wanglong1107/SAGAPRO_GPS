#pragma once

#include "AP_HAL_Namespace.h"

#include "SPIDevice.h"
#include "I2CDevice.h"
#include "UARTDriver.h"
#include "system.h"


class AP_HAL::HAL {
public:
	HAL(
//		AP_HAL::UARTDriver* _uartA, // console
//		AP_HAL::UARTDriver* _uartB, // 1st GPS
//		AP_HAL::UARTDriver* _uartC, // telem1
//		AP_HAL::UARTDriver* _uartD, // telem2
//		AP_HAL::UARTDriver* _uartE, // 2nd GPS
//		AP_HAL::UARTDriver* _uartF, // extra1
		AP_HAL::SPIDevice* _spi_dev1,
        AP_HAL::I2CDevice* _i2c_dev1,
		AP_HAL::I2CDevice* _i2c_dev2,

        AP_HAL::Scheduler*  _scheduler,
        AP_HAL::Util*       _util
	)
:
//	uartA(_uartA),
//	uartB(_uartB),
//	uartC(_uartC),
//	uartD(_uartD),
//	uartE(_uartE),
//	uartF(_uartF),

	spi_dev1(_spi_dev1),
	i2c_dev1(_i2c_dev1),
	i2c_dev2(_i2c_dev2),

	scheduler(_scheduler),
	util(_util)
	{
//		AP_HAL::init();
	}
	
	virtual void initialize() const = 0;

//	AP_HAL::UARTDriver* uartA;
//	AP_HAL::UARTDriver* uartB;
//	AP_HAL::UARTDriver* uartC;
//	AP_HAL::UARTDriver* uartD;
//	AP_HAL::UARTDriver* uartE;
//	AP_HAL::UARTDriver* uartF;
	AP_HAL::SPIDevice* spi_dev1;
	AP_HAL::I2CDevice* i2c_dev1;
	AP_HAL::I2CDevice* i2c_dev2;
	
	AP_HAL::Scheduler*  scheduler;
	AP_HAL::Util        *util;
};

size_t strnlen(const char *s, size_t count);
