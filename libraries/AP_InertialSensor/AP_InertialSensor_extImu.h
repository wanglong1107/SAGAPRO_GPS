#ifndef _AP_INERTIALSENSOR_EXTIMU_H
#define _AP_INERTIALSENSOR_EXTIMU_H


#include <AP_HAL/HAL_BUS.h>
#include <AP_HAL_PX4/HAL_SPI_BUS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Filter/LowPassFilter2p.h>
#include <AP_HAL/Scheduler.h>

#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_extImu : public AP_InertialSensor_Backend
{

public:
	
	AP_InertialSensor_extImu( void *_dev );
    virtual ~AP_InertialSensor_extImu();
	
	static AP_InertialSensor_Backend *probe( void *_dev );
	
	

	// initialize the sensor
    int Initialize(void);
	
	// read sensor raw data
	int GetRawData(void ) override;
	
	int Update(void ) override;
	
	
	int CalibrationGyroOffset(void);
	
    int CheckWhoami();
	int Start();

    enum ADI_Memse_Type {
        ADS620 = 0,
    };

	LowPassFilter2pVector3f accel_filter{1000, 15};
	LowPassFilter2pVector3f gyro_filter{1000, 30};
	

    /* Check if there's data available by either reading DRDY pin or register */
    int data_ready();

    int16_t raw_temp;

    float temp_sensitivity = 1.0/340; // degC/LSB
    float temp_zero = 36.53; // degC


    enum Rotation rotation;

    // which sensor type this is
    enum ADI_Memse_Type mpu_type;
	
	// Last status from register user control
    uint8_t last_stat_user_ctrl;    

    // buffer for fifo read
    uint8_t *fifo_buffer;
	
	
private:
	uint8_t *dev;
};


#endif //_AP_INERTIALSENSOR_EXTIMU_H
