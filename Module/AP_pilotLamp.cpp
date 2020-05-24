#include "AP_pilotLamp.h"

AP_PilotLamp::AP_PilotLamp()
{
	
}


int AP_PilotLamp::initialize()
{
	
	return 0;
}


int AP_PilotLamp::disInitialize()
{

	return 0;
}

int AP_PilotLamp::setColorFrequency(FLIGHT_COLOR_t _color, float _freq)
{
	currentColor = _color;
	blinkFreq = _freq;
	switch(currentColor)
	{
		case RED:
		{
			redLed->on();
			greenLed->off();
			blueLed->off();
			break;
		}
		case GREEN:
		{
			redLed->off();
			greenLed->on();
			blueLed->off();
			break;
		}
		case YELLOW:
		{
			redLed->on();
			greenLed->on();
			blueLed->off();
			break;
		}
		case BLUE:
		{
			redLed->off();
			greenLed->off();
			blueLed->on();
			break;
		}
		case PURPLE:
		{
			redLed->on();
			greenLed->off();
			blueLed->on();
			
			break;
		}
		case CYAN:
		{
			redLed->off();
			greenLed->on();
			blueLed->on();
			break;
		}
		case WHITE:
		{
			redLed->on();
			greenLed->on();
			blueLed->on();
			break;
		}
		default:
		{
			break;
		}				
	}

	if(blinkFreq>0)
	{
		
	}
	else
	{
		
	}

	return 0;
	
}



