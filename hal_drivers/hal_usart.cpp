#include "hal_usart.h"

#include <string.h>

int HAL_USART::initialize(void)
{
	if(!initialFlag)
	{
		usart_init(hw);
		usart_enable(hw);
		initialFlag = true;
	}
	
	usart_set_irq_state(hw, USART_RX_DONE, 1);
	return 0;
}


int HAL_USART::disInitialize(void)
{
	if(initialFlag)
	{
		usart_deinit(hw);
		initialFlag = false;
	}
	return 0;
}

int HAL_USART::setBaudRate(uint32_t _baudRate)
{
	usart_set_baud_rate( hw, _baudRate);
	baudRate = _baudRate;
	return 0;
}

int HAL_USART::writeData(uint8_t* pData, int len)
{
	int32_t ret = 0;
	
	for(int i=0; i<len; i++)
	{
		while(!usart_is_transmit_done(hw));
		usart_write_byte( hw, pData[i]);
	}
	return 0;
}



int HAL_USART::readData(uint8_t* pData, int len)
{
	int32_t ret = 0;
	for(int i=0; i<len; i++)
	{
		while(usart_is_byte_received(hw))
			pData[i] = usart_read_byte( hw );
	}
	return ret;	
}



