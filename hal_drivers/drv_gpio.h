#ifndef _HAL_GPIO_H
#define _HAL_GPIO_H
	 
#include <atmel_start.h>

class HAL_GPIO
{
private:	
	uint8_t pin;
public:
	bool initialFlag;
	
	HAL_GPIO()
	{
		initialFlag = false;
	}
	virtual ~HAL_GPIO(){}
	
	int initialize();
	int disInitialize(void);
		
	int set();
	int reSet();
	int toggle();
		
};

#endif //_HAL_GPIO_H
