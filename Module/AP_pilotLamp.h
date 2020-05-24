#ifndef _PILOT_LAMP_H_
#define _PILOT_LAMP_H_


#include "AP_LED.h"

void blinkHandl(union sigval val);

typedef enum
{
	NOCOLOR = 0,
	RED,
	GREEN,
	YELLOW,
	BLUE,
	PURPLE,
	CYAN,
	WHITE,
} FLIGHT_COLOR_t;

class AP_PilotLamp
{
public:
	AP_PilotLamp();
    int initialize();
    int disInitialize();
	
	int setColorFrequency(FLIGHT_COLOR_t _color, float _freq);

	FLIGHT_COLOR_t currentColor;
	float blinkFreq;
	
private:
	AP_LED *redLed;
	AP_LED *greenLed;
	AP_LED *blueLed;
};

#endif //_PILOT_LAMP_H_

