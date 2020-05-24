#include <assert.h>

#include "AP_InertialSensor.h"


extern HAL_SPI_BUS imuBus;
extern uint8_t adiMemseImu[16];


AP_InertialSensor *AP_InertialSensor::singleton = nullptr;

AP_InertialSensor::AP_InertialSensor()
{
    if (singleton != nullptr )
	{
		return;
    }
    singleton = this;
}

/*
 * Get the AP_InertialSensor singleton
 */
AP_InertialSensor *AP_InertialSensor::get_singleton()
{
    if (!singleton)
	{
        singleton = new AP_InertialSensor();
    }
    return singleton;
}


int AP_InertialSensor::Initialize(void)
{
	int ret = -1;
	
	backend = AP_InertialSensor_Invensense::probe( &imuBus );
	//backend = AP_InertialSensor_extImu::probe( adiMemseImu );
	
	if(backend == nullptr)
	{
		return -1;
	}
	
	return 0;
}
int AP_InertialSensor::Update( void )
{
	backend->Update();

	return 0;
}


namespace AP {

AP_InertialSensor &ins()
{
    return *AP_InertialSensor::get_singleton();
}

};
