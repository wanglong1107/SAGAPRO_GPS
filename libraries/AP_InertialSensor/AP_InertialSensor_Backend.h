#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>



class AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Backend();

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}
    
    /*
      device driver IDs. These are used to fill in the devtype field
      of the device ID, which shows up as INS*ID* parameters to
      users. The values are chosen for compatibility with existing PX4
      drivers.
      If a change is made to a driver that would make existing
      calibration values invalid then this number must be changed.
     */
    enum DevTypes {
        DEVTYPE_BMI160       = 0x09,
        DEVTYPE_L3G4200D     = 0x10,
        DEVTYPE_ACC_LSM303D  = 0x11,
        DEVTYPE_ACC_BMA180   = 0x12,
        DEVTYPE_ACC_MPU6000  = 0x13,
        DEVTYPE_ACC_MPU9250  = 0x16,
        DEVTYPE_ACC_IIS328DQ = 0x17,
        DEVTYPE_ACC_LSM9DS1  = 0x18,
        DEVTYPE_GYR_MPU6000  = 0x21,
        DEVTYPE_GYR_L3GD20   = 0x22,
        DEVTYPE_GYR_MPU9250  = 0x24,
        DEVTYPE_GYR_I3G4250D = 0x25,
        DEVTYPE_GYR_LSM9DS1  = 0x26,
        DEVTYPE_INS_ICM20789 = 0x27,
        DEVTYPE_INS_ICM20689 = 0x28,
        DEVTYPE_INS_BMI055   = 0x29,
    };
	
	// read sensor raw data
	virtual int GetRawData(void ) = 0;
	
	// Update sensor
	virtual int Update(void ) = 0;
	
	
	uint8_t imuRawDataBuffer[14];
	Vector3f accelRawData;		
	float tempRawData;
	Vector3f gyroRawData;
	
	Vector3f accelData;
	float tempData;
	Vector3f gyroData;
	
	Vector3f accelFilterData;
	Vector3f gyroFilterData;
	
	Vector3f gyroOffset;
	
	float gyro_scale;
    float accel_scale;
	
	
	
private:
	

};
