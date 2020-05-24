#include "drv_can.h"

#include <string.h>

int DRV_CAN::initialize(void)
{
	if(!initialFlag)
	{
		can_async_enable(hw);
		initialFlag = true;
	}
	return 0;
}


int DRV_CAN::disInitialize(void)
{
	if(initialFlag)
	{
		can_async_disable(hw);
		initialFlag = false;
	}
	return 0;
}

int DRV_CAN::enable(void)
{
	int ret = 0;
	ret = can_async_enable(hw);
	return ret;
}
int DRV_CAN::disable(void)
{
	int ret = 0;
	ret = can_async_disable(hw);
	
	return ret;
}


int DRV_CAN::setBaudRate(uint32_t _baudRate)
{
	int ret = 0;
	
	if(ret != 0)
	{
		return -1;
	}
	baudRate = _baudRate;
	return 0;
}

int DRV_CAN::setFilter(uint8_t index, enum can_format fmt,struct can_filter *filter)
{
	int ret = 0;
	ret = can_async_set_filter( hw, index, fmt, filter);
	if(ret<0)
	{
		return -1;
	}
	return 0;
}

int DRV_CAN::writeData( struct can_message *msg )
{
	int ret = 0;
	ret = can_async_write( hw, msg);
	return ret;
}



int DRV_CAN::readData( struct can_message *msg )
{
	int ret = 0;
	ret = can_async_read( hw, msg);
	return ret;	
}

int DRV_CAN::register_isr( enum can_async_callback_type type, FUNC_PTR cb)
{
	int ret = 0;
	
	ret = can_async_register_callback( hw, type, cb);
	if(ret < 0)
	{
		return -1;
	}
	return 0;
}



