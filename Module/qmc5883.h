#ifndef _QMC5883_H
#define _QMC5883_H

#include <AP_HAL/AP_HAL.h>
//#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include "hal_drivers/hal_i2c.h"

#define QMC5883_2GAIN  8.333333e-5f  //1/12000     
#define QMC5883_8GAIN  3.333333e-4   //1/3000     used



/*following para for QMC5883L*/

#define CONFIG_MOD   0x09
#define CONFIG_INT   0x0A
#define CONFIG_FBR   0x0B
#define CHIP_ID      0x0D
#define DATA_X_LSB   0x00     // Z_MSB 0x05
#define STATUS_REG   0x06
#define TEMP_LSB     0x07
#define DADR_5983    0x0D

//extern const AP_HAL::HAL& hal;

class QMC5883
{
	public:
		
	QMC5883(HAL_I2C * _dev)
	{
	dev = _dev;
	
	magError = 0; 
	updateCnt = 0;
	}

	bool selfTestPassed;
	bool magCalDone;
	uint8_t id;
	
	uint8_t rawDataBuffer[6];
	Vector3f magrawData;
	Vector3f magData;
	Vector3f magFilterData;;
	Vector3f offset;
	
	uint32_t magError;
	uint32_t updateCnt;
	uint16_t magErrorRestTime;
	LowPassFilter2pVector3f mag_filter{50, 20};
	
	
	uint16_t outputRange;
	Vector3f selfTestData;
	
	float yawOrigialAngler;
	uint8_t magInitialFailFlag;
	
	
	void calibrationOffset(void);
	int Restart(void);
	int readRawData(void );
	int initialize(void);
	int readID( void);
	uint8_t selfTest(void );
	int update(void);
	
	HAL_I2C *dev;
	
private:
	
};


#endif //_QMC5883_H
