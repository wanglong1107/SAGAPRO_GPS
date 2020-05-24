#ifndef _CRIUS_H
#define _CRIUS_H

#include "USART_Driver.h"
#include "UART_Driver.h"
#include "CRIUS.h"

class CRIUS : public UART_Device
{
public:
	Uart* UART;
	Pin pin;
	uint32_t id;
	IRQn_Type irq;

	CRIUS(Uart* _uart, Pin _pin, uint32_t _id, IRQn_Type _irq):UART_Device()
	{
		UART = _uart;
		pin = _pin;
		
		id = _id;
		irq = _irq;
	}

	
	void CRIUS_SendData(uint8_t* dataBuffer, int len);
	void CRIUS_Config();
	void CRIUS_ParseXbee();
	
};
#endif  //_CRIUS_H
