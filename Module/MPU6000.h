#ifndef _MPU6000_H
#define _MPU6000_H

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

//#include "eeprom.h"

#define MPU6000_DESIRE_GYRO_RANGE    0x10    // +-250dps 0x00; +-500dps 0x08; +-1000dps 0x10; +-2000dps 0x18;
#define MPU6000_DESIRE_ACC_RANGE     0x18    // +-2G 0x00; +-4G 0x08; +-8G 0x10; +-16G 0x18

#define MPU6000_Device_ID           ((uint8_t)0x68)  // In MPU6000     in MPU6500  ID = 0x70
// MPU 6000 registers

#define MPU6000_WHO_AM_I 						0x75 //
#define MPU6000_SMPLRT_DIV 					0x19 //
#define MPU6000_CONFIG 							0x1A //
#define MPU6000_GYRO_CONFIG 				0x1B
#define MPU6000_ACCEL_CONFIG 				0x1C
#define MPU6000_FIFO_EN 						0x23
#define MPU6000_INT_PIN_CFG 				0x37
#define MPU6000_INT_ENABLE 					0x38
#define MPU6000_INT_STATUS 					0x3A
#define MPU6000_ACCEL_XOUT_H 				0x3B //
#define MPU6000_ACCEL_XOUT_L 				0x3C //
#define MPU6000_ACCEL_YOUT_H 				0x3D //
#define MPU6000_ACCEL_YOUT_L 				0x3E //
#define MPU6000_ACCEL_ZOUT_H 				0x3F //
#define MPU6000_ACCEL_ZOUT_L 				0x40 //
#define MPU6000_TEMP_OUT_H 					0x41//
#define MPU6000_TEMP_OUT_L 					0x42//
#define MPU6000_GYRO_XOUT_H 				0x43 //
#define MPU6000_GYRO_XOUT_L 				0x44 //
#define MPU6000_GYRO_YOUT_H 				0x45 //
#define MPU6000_GYRO_YOUT_L 				0x46 //
#define MPU6000_GYRO_ZOUT_H 				0x47 //
#define MPU6000_GYRO_ZOUT_L 				0x48 //
#define MPU6000_USER_CTRL 					0x6A //
#define MPU6000_PWR_MGMT_1 					0x6B //
#define MPU6000_PWR_MGMT_2 					0x6C //
#define MPU6000_FIFO_COUNTH 				0x72
#define MPU6000_FIFO_COUNTL 				0x73
#define MPU6000_FIFO_R_W 						0x74

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 									0x40
#define BIT_H_RESET 								0x80
#define BITS_CLKSEL 								0x07
#define MPU_CLK_SEL_PLLGYROX 				0x01
#define MPU_CLK_SEL_PLLGYROZ 				0x03
#define MPU_EXT_SYNC_GYROX 					0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      	0x10
#define BIT_RAW_RDY_EN        		0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA   		0x01
 
                                            
                                            // high 4 bits  low 4 bits
                                            // Product Name Product Revision
#define MPU6000ES_REV_C4            0x14    // 0001         0100
#define MPU6000ES_REV_C5            0x15    // 0001         0101
#define MPU6000ES_REV_D6            0x16    // 0001         0110
#define MPU6000ES_REV_D7            0x17    // 0001         0111
#define MPU6000ES_REV_D8            0x18    // 0001         1000    
#define MPU6000_REV_C4              0x54    // 0101         0100 
#define MPU6000_REV_C5              0x55    // 0101         0101
#define MPU6000_REV_D6              0x56    // 0101         0110    
#define MPU6000_REV_D7              0x57    // 0101         0111
#define MPU6000_REV_D8              0x58    // 0101         1000
#define MPU6000_REV_D9              0x59    // 0101         1001


// MPU6000 gyroscope scaling	
#define MPU6000_250rps     ( 1.33231240610254e-04f )   // pi/(180*131) rps/LSB
#define MPU6000_500rps     ( 2.66462481220508e-04f )  // pi/(180*65.5) rps/LSB
#define MPU6000_1000rps    ( 5.32112576827539e-04f )  // pi/(180*32.8) rps/LSB
#define MPU6000_2000rps    ( 1.06422515365508e-03f )  // pi/(180*16.4) rps/LSB

// MPU6000 accelerometer scaling
#define MPU6000_2g       (GRAVITY_MSS / 16384.0f)   // 0.0000610352 g/LSB m/(s*s)
#define MPU6000_4g       (GRAVITY_MSS / 8192.0f)    // 0.0001220703 g/LSB m/(s*s)
#define MPU6000_8g       (GRAVITY_MSS / 4096.0f)    // 0.0002441406 g/LSB m/(s*s)
#define MPU6000_16g      (GRAVITY_MSS / 2048.0f)    // 0.0004882813 g/LSB m/(s*s)

extern int calAccOrientationFlag;

extern const AP_HAL::HAL& hal;

class MPU6000
{
public:
	
	MPU6000();
	
	
	bool mpu6000flag;
	bool gyroCalibate_OK;
	bool healthyTestPassed; 
	bool acc_healthy;
	bool gyro_healthy;

	uint8_t id;
	uint16_t gyroRange;
	uint16_t accleRange;
	
	uint64_t imuLastUpdate;
	
	uint8_t rawDataBuffer[14];
	
	Vector3f accelRawData;		
	float tempRawData;
	Vector3f gyroRawData;
	
	Vector3f accelData;
	float tempData;
	Vector3f gyroData;
	
	Vector3f accelDataScal;
	float accelMag;
	float accelMagU;
	
	Vector3f gyroAverageData;
	Vector3f accelAverageData;
	
	Vector3f accelFilterData;
	Vector3f gyroFilterData;
	
	Vector3f gyroOffset;
	Vector3f accelOffset;
	Vector3f accelScale;
	Matrix3f accelT;
	
	LowPassFilter2pVector3f accel_filter{1000, 20};
	LowPassFilter2pVector3f gyro_filter{1000, 30};

	
	
	void initialize(void);
	
	uint8_t readGyroID( void );
	uint8_t readGyroRange( void );
	uint8_t readAccRange(void);
	uint8_t readAccelRaw( void );
	uint8_t readTempRaw( void );
	uint8_t readGyroRaw( void );
	uint8_t readRaw( void );
	uint8_t selfTest(void);

	void getAccelRawData(Vector3f* _accelRawData);
	void getTempRawData(float* _tempRawData);
	void getGryoRawData(Vector3f* _gyroRawData);
	void getRawData(Vector3f* _accelRawData, float* _tempRawData, Vector3f* _gyroRawData);
	void autoCalibrationOffset(void);
	void update(void);
	
	bool accelCalibration();
	bool accelGetMeasurement(float (&_accel_ref)[6][3]);
	int accelDetectOrientation(void);
	void accelReadAvg(float (&accel_avg)[3],int samples_num);
	bool accelCalValues(float (&_accel_ref)[6][3],float (&_accel_T)[3][3],float (&_accel_offset)[3]);
	
	
private:
	
	AP_HAL::Device *dev;

		
};
	 

#endif //_MPU6000_H
