#ifndef _HAL_CAN_H
#define _HAL_CAN_H

#include <stdint.h>
#include "hal_can_async.h"

class HAL_CAN
{
private:
	struct can_async_descriptor *hw;

public:
	bool initialFlag;

	uint32_t baudRate;

	HAL_CAN( void *const _hw)
	{
		hw = (struct can_async_descriptor *)_hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	int enable(void);
	int disable(void);
	
	int setBaudRate(uint32_t _baudRate);
	int setFilter(uint8_t index, enum can_format fmt,struct can_filter *filter);
	
	int writeData( uint8_t* data );
	int readData( struct can_message *msg );
	
	int register_isr( enum can_async_callback_type type, FUNC_PTR cb);
};

#endif //_HAL_CAN_H
