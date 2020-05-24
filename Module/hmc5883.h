#ifndef _HMC5883_H
#define _HMC5883_H

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>


#define  HMC5883_CONFIG_A			0x00
#define  HMC5883_CONFIG_B			0x01
#define  HMC5883_MODE				0x02
#define	 HMC5883_X_OUTPUT_L			0x03

#define  HMC5883_STATUS				0x09
#define  HMC5883_IDENTIFICATION_A	0x0a
#define  HMC5883_IDENTIFICATION_B	0x0b
#define  HMC5883_IDENTIFICATION_C	0x0c

#define HMC5883_0_88GAIN  7.299270072992701e-4f  //1/1370      0x20 +-0.88Ga
#define HMC5883_1_30GAIN  9.174311926605505e-4   //1/1090      0x20 +-1.30Ga
#define HMC5883_1_90GAIN  1.219512195122e-3f     //1/820       0x40 +-1.90Ga
#define HMC5883_2_50GAIN  1.5151515151515e-3f    //1/660       0x60 +-2.50Ga
#define HMC5883_4_00GAIN  2.2727272727273e-3f    //1/440       0x80 +-4.00Ga
#define HMC5883_4_70GAIN  2.5641025641026e-3f    //1/390       0xA0 +-4.70Ga
#define HMC5883_5_60GAIN  3.030303030303e-3f     //1/330       0xC0 +-5.60Ga
#define HMC5883_8_10GAIN  4.3478260869565e-3f    //1/230       0xE0 +-8.10Ga



extern const AP_HAL::HAL& hal;

class HMC5883
{
	public:
		
	HMC5883();

	bool selfTestPassed;
	bool magCalDone;
	uint8_t id;
	
	uint8_t rawDataBuffer[6];
	Vector3f magrawData;
	Vector3f magData;
	Vector3f magFilterData;;
	Vector3f offset;
	
	LowPassFilter2pVector3f mag_filter{50, 20};
	
	
	uint16_t outputRange;
	Vector3f selfTestData;
	
	float yawOrigialAngler;
	
	
	void calibrationOffset(void);
	void readRawData(void );
	int initialize(void);
	int readID( void);
	uint8_t selfTest(void );
	void update(void);
		
private:
	AP_HAL::Device *dev;
};


#endif //_QMC5883_H
