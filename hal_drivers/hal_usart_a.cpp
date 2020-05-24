#include "hal_usart_a.h"
#include "hpl_sercom_config.h"

#include <string.h>
#include <hal_usart_sync.h>
int HAL_UART_A::initialize(void)
{
	if(!initialFlag)
	{
		usart_sync_get_io_descriptor(hw, &io);
		usart_sync_enable(hw);
		initialFlag = true;
	}
	return 0;
}


int HAL_UART_A::disInitialize(void)
{
	if(initialFlag)
	{
		usart_sync_disable(hw);
		initialFlag = false;
	}
	return 0;
}

int HAL_UART_A::setBaudRate(uint32_t _baudRate)
{
	int ret = 0;
	uint32_t tmpBaudRate = 0;
	
	tmpBaudRate = 65536 - ((65536 * 16.0f * _baudRate) / CONF_GCLK_SERCOM0_CORE_FREQUENCY);
	ret = usart_sync_set_baud_rate( hw, tmpBaudRate);
	if(ret != 0)
	{
		return -1;
	}
	baudRate = _baudRate;
	return 0;
}

int HAL_UART_A::writeData(uint8_t* pData, int len)
{
	int32_t ret = 0;
	
	ret = io_write( io, pData, len);
	if(ret < 0)
	{
		return -1;
	}
	return ret;
}



int HAL_UART_A::readData(uint8_t* pData, int len)
{
	int32_t ret = 0;
	
	ret = io_read( io, pData, len);
	if(ret < 0)
	{
		return -1;
	}
	return ret;	
}

//int HAL_UART::register_isr( const enum usart_async_callback_type type, usart_cb_t cb )
//{
//	int32_t ret = 0;
//	
//	ret = usart_async_register_callback( hw, type, cb);
//	if(ret < 0)
//	{
//		return -1;
//	}
//	return 0;
//}



