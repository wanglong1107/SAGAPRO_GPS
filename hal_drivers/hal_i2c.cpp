#include "hal_i2c.h"
#include <string.h>
#include <task.h>
#include <AP_HAL/AP_HAL.h>
#include <hal_drivers/drv_hrt.h>
#define C21_qmc5883 
struct io_descriptor *I2C_0_io;
int HAL_I2C::initialize(void)
{
	if(!initialFlag)
	{
		i2c_m_sync_set_slaveaddr( i2c, deviceAddr, I2C_M_SEVEN);
		i2c_m_sync_enable( i2c );
		initialFlag = true;
	}
	return 0;
}


int HAL_I2C::disInitialize(void)
{
	if(initialFlag)
	{
		i2c_m_sync_disable( i2c );
		initialFlag = false;
	}
	return 0;
}

int HAL_I2C::setBaudRate( uint32_t cnt_Hz)
{
	
	return 0;
}

int HAL_I2C::writeData(uint8_t reg, uint8_t* pData, uint8_t len)
{
	int32_t ret = 0;
	uint8_t pData_temp[2] = {0};
	
	pData_temp[0] = reg;
	pData_temp[1] = *pData;
	//taskENTER_CRITICAL();
    //i2c_m_sync_set_slaveaddr( i2c, deviceAddr, I2C_M_SEVEN);
#ifdef C21_qmc5883	
	//struct i2c_m_sync_desc *i2c = CONTAINER_OF(io, struct i2c_m_sync_desc, io);
	struct _i2c_m_msg       msg;

	msg.addr   = i2c->slave_addr;
	msg.len    = len+1;
	msg.flags  = I2C_M_STOP;
	msg.buffer = (uint8_t *)pData_temp;

	ret = _i2c_m_sync_transfer(&i2c->device, &msg);
#else	
	ret = i2c_m_sync_cmd_write( i2c, reg, pData, len);
#endif
	//taskEXIT_CRITICAL();
	if(ret != 0 )
	{
		return 0;
	}
	return -1;
}


int HAL_I2C::readData(uint8_t reg, uint8_t* pData, uint8_t len)
{
	int ret = 0;

	//taskENTER_CRITICAL();
	//i2c_m_sync_set_slaveaddr( i2c, deviceAddr, I2C_M_SEVEN);
	ret = i2c_m_sync_cmd_read( i2c, reg, pData, len);
	//taskEXIT_CRITICAL();
	if(ret != 0 )
	{
		return -1;
	}
	return 0;
}
