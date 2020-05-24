#ifndef _HAL_UART_H
#define _HAL_UART_H

#include <stdint.h>
#include "hal_usart_async.h"

class HAL_UART
{
private:
	

public:
	struct usart_async_descriptor *hw;

	struct io_descriptor *io;

	bool initialFlag;

	uint32_t baudRate;

	HAL_UART( void *const _hw)
	{
		hw = (struct usart_async_descriptor *)_hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	int setBaudRate(uint32_t _baudRate);

	int writeData(uint8_t* pData, int len);
	int readData(uint8_t* pData, int len);
	
	int register_isr( const enum usart_async_callback_type type, usart_cb_t cb );
};

#endif //_HAL_UART_H

