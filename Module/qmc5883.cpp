#include "qmc5883.h"

#include <string.h>

#include "Filter/AverageFilter.h"
#include <AP_AHRS/AP_AHRS_View.h>
#include "hal_drivers/hal_i2c.h"
//#include <AP_HAL/I2CDevice.h>
extern uint8_t sendData[6][128];
extern int DataSend(uint8_t i);
extern uint8_t sendDataFlg[6];
extern HAL_I2C I2cToQMC5883;
//QMC5883::QMC5883(HAL_I2C * _dev)
//{
//	dev = _dev;//hal.i2c_dev1
//	
//	magError = 0; // default error
//	updateCnt = 0;
//}

int QMC5883::Restart(void)
{
	int ret = 0;
	magError = 0;
	magInitialFailFlag = 0;
	gpio_set_pin_level(Mag_EN,false);
	delay_ms(100);
	gpio_set_pin_level(Mag_EN,true);
	
	delay_ms(10);
	
	ret = readID();
	if(ret < 0)
	{
		magInitialFailFlag = 1;
		return -1;
	}
	
	delay_ms(10);// config QMC5883
	uint8_t ModValue = 0x0D;  // 0x09H = 0x1D means work in continuous mode  data output rate = 200Hz  range = 8G and over sample ratio = 512
	uint8_t FBRValue = 0x01;  // datasheet recomend value
	dev->writeData( CONFIG_MOD, &ModValue, 1);
	delay_ms(10);
	dev->writeData( CONFIG_FBR, &FBRValue, 1);
	
	return 0;
}

int QMC5883::initialize(void)
{
	int ret = 0;
	
	magCalDone = false;
	
	float horizontalX=0.0f;
	float horizontalY=0.0f;
	uint8_t mag_cal_flag;
	
	dev->initialize();
	
	Restart();

	Vector3f magCalibration_offset_r(0.0f, 0.0f, 0.0f); //#2
	
//	uint8_t ModValue = 0x1D;  // 0x09H = 0x1D means work in continuous mode  data output rate = 200Hz  range = 8G and over sample ratio = 512
//	uint8_t FBRValue = 0x01;  // datasheet recomend value

//	delay_ms(10);

//	ret = readID();
//	if(ret < 0)
//	{
//		return -1;
//	}
//	delay_ms(10);

//	dev->writeData( CONFIG_MOD, &ModValue, 1);
//	delay_ms(10);
//	dev->writeData( CONFIG_FBR, &FBRValue, 1);
//	delay_ms(10);

	
//	calibrationOffset(); 
	offset.x = magCalibration_offset_r[0];
	offset.y = magCalibration_offset_r[1];
	offset.z = magCalibration_offset_r[2]; 


	for( int i=0; i<256; i++)
	{
		readRawData();
		horizontalX += (magrawData.x - offset.x);
		horizontalY += (magrawData.y - offset.y);
		delay_ms(5);
		//hal.scheduler->delay(5);
	}
	
	yawOrigialAngler = -atan2(horizontalY, horizontalX) * 57.29f;
	if(yawOrigialAngler < 0)
	{
		yawOrigialAngler = yawOrigialAngler + 360;
	}
	
	return 0;
	
}


int QMC5883::readID( void )
{
	int ret = 0;
	uint8_t  temp = 0;
	
	ret = dev->readData( CHIP_ID, &temp, 1);
	if(ret < 0)
	{
		return -1;
	}

	if(0xFF != temp)
	{
		return -1;
	}
	id = temp;
	
	return 0;
	
}

int QMC5883::readRawData()
{
	int ret = 0;
	int j;
	uint8_t  temp[6] = {0};
	uint8_t  magData[14] ={0xB5,0x62,0x11,0x01,0x06,0x00}; 
	uint8_t CK_A=0, CK_B=0;
	ret = dev->readData( DATA_X_LSB, temp, 6);
	if(ret < 0)
	{
		return -1;
	}
	memcpy( &magData[6], temp, 6 );
	for(uint8_t i=0; i<10; i++)
	{
		CK_A = CK_A + magData[2+i];
		CK_B = CK_B + CK_A;
	}
	magData[12] = CK_A;
	magData[13] = CK_B;
	for( j=5;j>=0;j--)
	{
		if(sendDataFlg[j]==0)
		{
			sendDataFlg[j]=0x01;
			memcpy( sendData[j], magData, 14 );
			break;
		}
	}
	
//	memcpy( rawDataBuffer, temp, 6 );
//	magrawData.x = (float)( (int16_t)(uint16_t(temp[3])<<8 | temp[2]) );
//	magrawData.y = (float)( (int16_t)(uint16_t(temp[1])<<8 | temp[0]) );
//	magrawData.z = -(float)( (int16_t)(uint16_t(temp[5])<<8 | temp[4]) );
	
	return 0;
}

int QMC5883::update(void)
{
	int ret = 0;
	ret = readRawData();
	if(ret<0)
	{
		magError ++;
		if(magError>=10)
		{
			magErrorRestTime++;
			Restart();
		}
		return -1;
		
	}
	updateCnt ++;
	
	magData = (magrawData-offset)*QMC5883_8GAIN;
	magFilterData = mag_filter.apply(magData);
	
	
	return 0;
}

void QMC5883::calibrationOffset(void)
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
		
		delay_ms(5);
		//hal.scheduler->delay(10);

	}
	
	offset = (magMin + magMax)*0.5f;

}
