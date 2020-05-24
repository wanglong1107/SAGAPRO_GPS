#include "AP_LED.h"


AP_LED::AP_LED()
{
	
}


int AP_LED::initialize()
{
	return 0;
}


int AP_LED::disInitialize(void)
{
	return 0;
}


void AP_LED::on()
{
	dev->set();
}
void AP_LED::off()
{
	dev->reSet();
}
void AP_LED::toggle()
{
	dev->toggle();
}
void AP_LED::setBlinkFrequency( uint32_t _blinkTime, float _freq)
{
	
}

