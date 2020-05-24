#ifndef _HAL_USART_H
#define _HAL_USART_H

#include <stdint.h>
#include <hpl_usart.h>

class HAL_USART
{
private:
	void* hw;

public:
	bool initialFlag;

	uint32_t baudRate;

	HAL_USART( void *const _hw)
	{
		hw = _hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	int setBaudRate(uint32_t _baudRate);

	int writeData(uint8_t* pData, int len);
	int readData(uint8_t* pData, int len);
	
};

#endif //_HAL_USART_H
