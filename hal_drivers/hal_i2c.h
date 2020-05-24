#ifndef _I2C_DEVICE_H
#define _I2C_DEVICE_H

#include <atmel_start.h>

class HAL_I2C
{
private:
	struct i2c_m_sync_desc *i2c;
	uint8_t deviceAddr;

public:
	bool initialFlag;

	HAL_I2C( struct i2c_m_sync_desc *_i2c, uint8_t _deviceAddr)
	{
		i2c = _i2c;
		deviceAddr = _deviceAddr;
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	int setBaudRate( uint32_t cnt_Hz);
	
	int writeData(uint8_t reg, uint8_t* pData, uint8_t len);
	int readData(uint8_t reg, uint8_t* pData, uint8_t len);
};

#endif //_I2C_DEVICE_H
