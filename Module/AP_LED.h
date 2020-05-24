#ifndef _AP_LED_H
#define _AP_LED_H

#include <hal_drivers/drv_gpio.h>



class AP_LED
{

private:
	HAL_GPIO *dev;

public:

	AP_LED();
    int initialize();
    int disInitialize(void);

	void on();
	void off();
	void toggle();
	void setBlinkFrequency( uint32_t _blinkTime, float _freq);
	

};

#endif //_AP_LED_H

