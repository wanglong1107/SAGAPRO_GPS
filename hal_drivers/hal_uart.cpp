#include "hal_uart.h"
#include "hpl_sercom_config.h"
//#include <utils_assert.h>
#include "driver_init.h"
#include <string.h>


int HAL_UART::initialize(void)
{
	if(!initialFlag)
	{
		usart_async_get_io_descriptor(hw, &io);
		usart_async_enable(hw);
		initialFlag = true;
	}
	return 0;
}


int HAL_UART::disInitialize(void)
{
	if(initialFlag)
	{
		usart_async_disable(hw);
		initialFlag = false;
	}
	return 0;
}

int HAL_UART::setBaudRate(uint32_t _baudRate)
{
	int ret = 0;
	uint32_t tmpBaudRate = 0;
	
	tmpBaudRate = 65536 - ((65536 * 16.0f * _baudRate) / CONF_GCLK_SERCOM0_CORE_FREQUENCY);
	ret = usart_async_set_baud_rate( hw, tmpBaudRate);
	if(ret != 0)
	{
		return -1;
	}
	baudRate = _baudRate;
	return 0;
}

int HAL_UART::writeData(uint8_t* pData, int len)
{
	int32_t ret = 0;

	for(int i=0; i<len; i++)
	{
		while( !usart_async_is_tx_empty( hw ) ); //Check if the usart receiver is not empty
		//while( !_usart_async_is_byte_sent( &(hw->device) ) );//Check if the usart receiver is not empty
		//hri_sercomusart_write_DATA_reg( hw->device.hw, pData[i] );
		_usart_async_write_byte( &(hw->device), pData[i] );
	}
//	ret = io_write( io, pData, len);
//	if(ret < 0)
//	{
//		return -1;
//	}
	return ret;
}



int HAL_UART::readData(uint8_t* pData, int len)
{
	int32_t ret = 0;
	
	ret = io_read( io, pData, len);
	if(ret < 0)
	{
		return -1;
	}
	return ret;	
}

int HAL_UART::register_isr( const enum usart_async_callback_type type, usart_cb_t cb )
{
	int32_t ret = 0;
	
	ret = usart_async_register_callback( hw, type, cb);
	if(ret < 0)
	{
		return -1;
	}
	return 0;
}



