#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include <AP_HAL/Scheduler.h>

#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_Invensense.h"

#include "AP_InertialSensor_extImu.h"

class AP_InertialSensor
{

public:
    AP_InertialSensor();

    /* Do not allow copies */
    AP_InertialSensor(const AP_InertialSensor &other) = delete;
    AP_InertialSensor &operator=(const AP_InertialSensor&) = delete;

	// get singleton instance
    static AP_InertialSensor *get_singleton();

	int Initialize(void);
	int Update( void );
	
	
	AP_InertialSensor_Backend *backend;
	
private:
	static AP_InertialSensor *singleton;
	


	
};

namespace AP {
    AP_InertialSensor &ins();
};
