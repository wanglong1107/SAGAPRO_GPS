#include "drv_gpio.h"

int HAL_GPIO::initialize()
{
	if(!initialFlag)
	{
		initialFlag = true;
	}
	return 0;
}

int HAL_GPIO::disInitialize(void)
{
	if(initialFlag)
	{
		initialFlag = false;
	}
	return 0;
}


int HAL_GPIO::set()
{
	gpio_set_pin_level(pin, true);
	return 0;
}

int HAL_GPIO::reSet()
{
	gpio_set_pin_level(pin, false);
	return 0;
}

int HAL_GPIO::toggle()
{
	gpio_toggle_pin_level(pin);
	return 0;
}