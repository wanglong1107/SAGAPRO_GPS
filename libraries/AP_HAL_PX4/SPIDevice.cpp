#include "SPIDevice.h"

using namespace PX4;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

PX4SPIDevice::PX4SPIDevice()
{
	dev = NULL;
}

PX4SPIDevice::PX4SPIDevice(HAL_QSPI *_dev)
{
	dev = _dev;
}

PX4SPIDevice::~PX4SPIDevice()
{

}

bool PX4SPIDevice::initialize()
{
	dev->initialize();
	return true;
}

bool PX4SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
		dev->setBaudRate(10000000);
        break;
    case AP_HAL::Device::SPEED_LOW:
		dev->setBaudRate(1000000);
        break;
	default:
		break;
    }
    return true;
}


bool PX4SPIDevice::writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	dev->writeData( _reg, _pData, len);
	return true;
}

bool PX4SPIDevice::readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len)
{
	dev->readData( _reg, _pData, len);
	return true;
}
