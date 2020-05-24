#include "hmc5883.h"

#include <string.h>

#include "Filter/AverageFilter.h"
#include <AP_AHRS/AP_AHRS_View.h>


HMC5883::HMC5883()
{
	dev = hal.i2c_dev1;
}

int HMC5883::initialize(void)
{
	int ret = 0;
	uint8_t configData = 0x00;
	
	magCalDone = false;
	float horizontalX=0.0f;
	float horizontalY=0.0f;
	uint8_t mag_cal_flag;
	
	Vector3f magCalibration_offset_r(0.0f, 0.0f, 0.0f); //#1
//	Vector3f magCalibration_offset_r(-338.0f, -260.0f, -59.0f); //#2
	
	dev->initialize();
	
	ret = readID();
	if(ret < 0)
	{
		return -1;
	}

	// config the hmc5883
	
	configData = 0x18;
	ret = dev->write_register( HMC5883_CONFIG_A, configData);	// soft reset device
	if(ret < 0)
	{
		return -1;
	}
	configData = 0xe0;
	ret = dev->write_register( HMC5883_CONFIG_B, configData);	// OSR=512, Rang=8Gauss ODR=200Hz Continuous mode
	if(ret < 0){
		return -1;
	}
	configData = 0x00;
	ret = dev->write_register( HMC5883_MODE, configData);		// OSR=512, Rang=8Gauss ODR=200Hz Continuous mode
	if(ret < 0){
		return -1;
	}
	hal.scheduler->delay(10);
	
//	calibrationOffset(); 
	offset.x = magCalibration_offset_r[0];
	offset.y = magCalibration_offset_r[1];
	offset.z = magCalibration_offset_r[2]; 


	for( int i=0; i<256; i++)
	{
		readRawData();
		horizontalX += (magrawData.x - offset.x);
		horizontalY += (magrawData.y - offset.y);
		hal.scheduler->delay(5);
	}
	
	yawOrigialAngler = -atan2(horizontalY, horizontalX) * 57.29f;
	if(yawOrigialAngler < 0)
	{
		yawOrigialAngler = yawOrigialAngler + 360;
	}
	
	return 0;
	
}


int HMC5883::readID( void )
{
	int ret = 0;
	uint8_t  temp = 0;
	
	ret = dev->read_registers( HMC5883_IDENTIFICATION_A, &temp, 1);
	if(ret < 0)
	{
		return -1;
	}

	if(0x48 != temp)
	{
		return -1;
	}
	id = temp;
	
	return 0;
	
}

void HMC5883::readRawData()
{
	uint8_t  temp[6] = {0};
	
	dev->read_registers( HMC5883_X_OUTPUT_L, temp, 6);
	memcpy( rawDataBuffer, temp, 6 );
	
	magrawData.x = -(float)( (int16_t)(uint16_t(temp[5])<<8 | temp[4]) );
	magrawData.y = (float)( (int16_t)(uint16_t(temp[1])<<8 | temp[0]) );
	magrawData.z = (float)( (int16_t)(uint16_t(temp[3])<<8 | temp[2]) );

}

void HMC5883::update(void)
{
	readRawData();
	
	magData = (magrawData-offset)*HMC5883_8_10GAIN;
	magFilterData = mag_filter.apply(magData);
}

void HMC5883::calibrationOffset(void)
{
	
	static Vector3f magMax(0.0f, 0.0f, 0.0f);
	static Vector3f magMin(0.0f, 0.0f, 0.0f);

	for(uint32_t i=0; i<1800; i++)
	{
		readRawData();
		
		if( magMax.x < magrawData.x )
		{
			magMax.x = magrawData.x;
		}
		
		if( magMax.y < magrawData.y )
		{
			magMax.y = magrawData.y;
		}
		
		if( magMax.z < magrawData.z )
		{
			magMax.z = magrawData.z;
		}
		
		
		if( magMin.x > magrawData.x )
		{
			magMin.x = magrawData.x;
		}
		
		if( magMin.y > magrawData.y )
		{
			magMin.y = magrawData.y;
		}
		
		if( magMin.z > magrawData.z )
		{
			magMin.z = magrawData.z;
		}
		
		hal.scheduler->delay(10);

	}
	
	offset = (magMin + magMax)*0.5f;

}
