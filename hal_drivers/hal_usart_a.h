#ifndef _HAL_USART_A_H
#define _HAL_USART_A_H

#include <stdint.h>
#include <hal_usart_sync.h>

class HAL_UART_A
{
private:
	struct usart_sync_descriptor *hw;
	struct io_descriptor *io;

public:
	bool initialFlag;

	uint32_t baudRate;

	HAL_UART_A( void *const _hw)
	{
		hw = (struct usart_sync_descriptor *)_hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	int setBaudRate(uint32_t _baudRate);

	int writeData(uint8_t* pData, int len);
	int readData(uint8_t* pData, int len);
	
	//int register_isr( const enum usart_async_callback_type type, usart_cb_t cb );
};

#endif //_HAL_UART_H

