#include "hal_can.h"
#include "hpl_can_config.h"
#include "driver_init.h"
#include <string.h>

int HAL_CAN::initialize(void)
{
	if(!initialFlag)
	{
		can_async_enable(hw);
		initialFlag = true;
	}
	return 0;
}


int HAL_CAN::disInitialize(void)
{
	if(initialFlag)
	{
		can_async_disable(hw);
		initialFlag = false;
	}
	return 0;
}

int HAL_CAN::enable(void)
{
	int ret = 0;
	ret = can_async_enable(hw);
	return ret;
}

int HAL_CAN::disable(void)
{
	int ret = 0;
	ret = can_async_disable(hw);
	return ret;
}

int HAL_CAN::setBaudRate(uint32_t _baudRate)
{
	int ret = 0;
	
	if(ret != 0)
	{
		return -1;
	}
	baudRate = _baudRate;
	return 0;
}

int HAL_CAN::setFilter(uint8_t index, enum can_format fmt,struct can_filter *filter)
{
	int ret = 0;
	ret = can_async_set_filter( hw, index, fmt, filter);
	if(ret<0)
	{
		return -1;
	}
	return 0;
}

int HAL_CAN::writeData(uint8_t* data)
{
	int ret = 0;
	uint16_t len;
	uint16_t head;
	struct can_message msg;
	msg.type = CAN_TYPE_DATA;
	msg.len  = 8;
	msg.fmt  = CAN_FMT_STDID;
	/********************************/
	len = (uint16_t)(data[4])+(uint16_t)(data[5]<<8);
	if(data[2] == 0x01)
	{
		msg.id = 0x006;
		msg.fmt  = CAN_FMT_STDID;
		head = 0;
		bool flg = true;
		while(flg)//len>0
		{
			msg.data = data+head;
			can_async_write( hw, &msg);
			delay_ms(1);
			head +=8;
			if(len>7)
			{
				len -=8;
			}
			else
			{
				//len =0;
				flg = false;
			}
		}
		msg.len  = len;
		msg.data = data+head;
		can_async_write( hw, &msg);
	}
	else
	{
		if(data[3] == 0x01)
		{
			msg.id = 0x005;
			uint8_t sum = 0;
			for(uint8_t i=0;i<6;i++)
			{
				sum +=data[6+i];
			}
			data[6+6] = sum;
			data[6+7] = 0x55;
			msg.data = data+6;
			can_async_write( hw, &msg);
//			ret = can_async_write( hw, &msg);
		}
	}

	/********************************/
	return ret;
}

int HAL_CAN::readData(struct can_message *msg)
{
	int ret = 0;
	ret = can_async_write( hw, msg);
	return ret;	
}

int HAL_CAN::register_isr( enum can_async_callback_type type, FUNC_PTR cb)
{
	int ret = 0;
	
	ret = can_async_register_callback( hw, type, cb);
	if(ret < 0)
	{
		return -1;
	}
	return 0;
}

