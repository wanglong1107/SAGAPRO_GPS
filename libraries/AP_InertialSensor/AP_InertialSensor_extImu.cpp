#include "AP_InertialSensor_extImu.h"

#include <stdint.h>
#include <stdlib.h>


AP_InertialSensor_extImu::AP_InertialSensor_extImu( void *_dev )
    : AP_InertialSensor_Backend()
{
	dev = (uint8_t *)_dev;
}

AP_InertialSensor_extImu::~AP_InertialSensor_extImu()
{
    if (fifo_buffer != nullptr) 
	{
        free(fifo_buffer);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_extImu::probe( void *_dev )
{
	int ret = -1;

	if(_dev == nullptr)
	{
		return nullptr;
	}
	
	AP_InertialSensor_extImu *sensor = new AP_InertialSensor_extImu( _dev );
	if(sensor == nullptr)
	{
		return nullptr;
	}
	sensor->Initialize();
	
	sensor->CalibrationGyroOffset();
	
	
	return sensor;
}




/*
  publish any pending data
 */
int AP_InertialSensor_extImu::GetRawData(void )
{
	int ret = -1;
	uint8_t temp[16]; // accel & gyro
	
	memcpy( temp, dev, 16 );
	memcpy( imuRawDataBuffer, temp, 14 );
	
	accelRawData.y = -(float)( (int16_t)(uint16_t(temp[0])<<8 | temp[1]) );
	accelRawData.x = -(float)( (int16_t)(uint16_t(temp[2])<<8 | temp[3]) );
	accelRawData.z = -(float)( (int16_t)(uint16_t(temp[4])<<8 | temp[5]) );
	
	
	gyroRawData.x = (float)( (int16_t)(uint16_t(temp[8]) <<8 | temp[9]) ); 
	gyroRawData.y = (float)( (int16_t)(uint16_t(temp[10])<<8 | temp[11]) );
	gyroRawData.z = (float)( (int16_t)(uint16_t(temp[12])<<8 | temp[13]) );
	
    return 0;
}


int AP_InertialSensor_extImu::Update(void )
{
	int ret = -1;
	
	ret = GetRawData();
	if(ret<0)
	{
		return -1;
	}
	
	accelData = accelRawData*accel_scale;
	tempData = tempRawData/340 + 36.53f;
	gyroData = (gyroRawData-gyroOffset)*gyro_scale;
	
	accelFilterData = accel_filter.apply(accelData);
	gyroFilterData = gyro_filter.apply(gyroData);
	
	return 0;
}

int AP_InertialSensor_extImu::CalibrationGyroOffset(void)
{
	int ret = -1;
	
	bool gyroCalibateOK = false;
	uint16_t cnt = 0;
	
	Vector3f gyroCalibrateSataSum(0.0f, 0.0f, 0.0f);
	Vector3f gyroCalibrateData(0.0f, 0.0f, 0.0f);
	Vector3f gyroCalibrateDataLast(0.0f, 0.0f, 0.0f);
	Vector3f goryDataError(0.0f, 0.0f, 0.0f);

	while(!gyroCalibateOK)
	{
		for(cnt=0; cnt<500; cnt++)
		{
			ret = GetRawData();
			gyroCalibrateSataSum += gyroRawData;
			HAL_Delay_ms(2);
		}
		gyroCalibrateData = gyroCalibrateSataSum/cnt;
		gyroCalibrateSataSum.zero();
		
		goryDataError = gyroCalibrateDataLast - gyroCalibrateData;
		if( goryDataError.length()<0.7f )
		{
			gyroCalibateOK = true;
		}
		else
		{
			gyroCalibateOK = false;
		}
		
		gyroCalibrateDataLast = gyroCalibrateData;
	}
	gyroOffset = gyroCalibrateDataLast;
	
	return 0;
}


/*
  check whoami for sensor type
 */
int AP_InertialSensor_extImu::CheckWhoami(void)
{
	mpu_type = ADS620;
	
	return 0;
}


int AP_InertialSensor_extImu::Initialize(void)
{
	
	gyro_scale = 0.001f;
    accel_scale = 0.01f; 
    
    return 0;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
int AP_InertialSensor_extImu::data_ready()
{

	return 0;
}


int AP_InertialSensor_extImu::Start()
{
	return 0;
}
