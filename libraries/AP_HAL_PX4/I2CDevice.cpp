#include "I2CDevice.h"
#include <AP_HAL/AP_HAL.h>
#include "Scheduler.h"


using namespace PX4;

PX4I2CDevice::PX4I2CDevice()
{

}

PX4I2CDevice::PX4I2CDevice(HAL_I2C *_dev)
{
	dev = _dev;
}
    
PX4I2CDevice::~PX4I2CDevice()
{

}

bool PX4I2CDevice::initialize()
{
	dev->initialize();
	return true;
}

bool PX4I2CDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
		case AP_HAL::Device::SPEED_HIGH:
			dev->setBaudRate(400000);
			break;
		case AP_HAL::Device::SPEED_LOW:
			dev->setBaudRate(100000);
			break;
		default:
			break;
    }
    return true;
}

bool PX4I2CDevice::writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	dev->writeData( _reg, _pData, len);
	return true;
}

bool PX4I2CDevice::readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	dev->readData( _reg, _pData, len);
	return true;
}
