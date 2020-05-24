#include "eeprom.h"
#include <string.h>
#include  <os.h>

EEPROM* EEPROM:: _s_instance  = NULL;//初始化在主线程之前

EEPROM::EEPROM()
{
	if (_s_instance)
	{
    }
    _s_instance = this;
}

int EEPROM::initialize( HAL_I2C *_dev )
{
	dev = _dev;
	dev->initialize();
	
	return 0;
}

int EEPROM::read_block( uint8_t addres, uint8_t *pData, uint8_t len)
{

    OS_ERR os_err;
	uint8_t n = 0;
	uint8_t m = 0;
	uint8_t temp[8];
	
	memset(temp, 0, 8);

	
	if(addres%8 != 0 || pData == NULL)
	{
		return -1;
	}
	
	n = len/8;
	m = len%8;
	
//	for(int i=0; i<len; i++)
//	{
//		dev->readData( addres+i, pData+i, 1);
//		//OSTimeDlyHMSM( 0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &os_err);
//	}
	if(n>0)
	{
		for(int i=0; i<n; i++)
		{
			dev->readData( addres+8*i, pData+8*i, 8);
			OSTimeDlyHMSM( 0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		}
	}
	if(m>0)
	{
		dev->readData( addres+8*n, temp, 8);
		memcpy(pData+8*n, temp, m);
		
	}
	
	return 0;
	
}


int EEPROM::write_block( uint8_t addres, uint8_t *pData, uint8_t len)
{
	OS_ERR os_err;
	uint8_t n = 0;
	uint8_t m = 0;
	uint8_t temp[8];
	
	memset(temp, 0, 8);
	
	if(addres%8 != 0 || pData == NULL)
	{
		return -1;
	}
	
	n = len/8;
	m = len%8;
	
//	for(int i=0; i<len; i++)
//	{
//		dev->writeData( addres+i, pData+i, 1);
//	}
	if(n>0)
	{
		for(int i=0; i<n; i++)
		{
			dev->writeData( (addres+8*i), (pData+8*i), 8);
			OSTimeDlyHMSM( 0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		}
	}
	
	if(m>0)
	{
		memcpy(temp, pData+8*n, m);
		dev->writeData( addres+8*n, temp, 8);
	}
	OSTimeDlyHMSM( 0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	
	return 0;
}


/*
 * Get the AP_InertialSensor singleton
 */
EEPROM *EEPROM::get_instance()
{
    if (!_s_instance) {
        _s_instance = new EEPROM();
    }
    return _s_instance;
}

