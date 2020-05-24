#ifndef _EERPOM_H
#define _EERPOM_H

#include <drivers/hal_i2c.h>



class EEPROM
{
private:
	static EEPROM *_s_instance;
public:
	HAL_I2C *dev;

	EEPROM();

	int initialize( HAL_I2C *_dev );
	int read_block( uint8_t addres, uint8_t *pData, uint8_t len);
	int write_block( uint8_t addres, uint8_t *pData, uint8_t len);
	
	static EEPROM *get_instance();
	

};


#endif //_EERPOM_H
