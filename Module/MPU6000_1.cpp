#include "MPU6000.h"
#define CONSTANTS_ONE_G  9.80665
hrt_abstime imuLastUpdate = 0;
int calAccOrientationFlag = -3;  //   -2:start calAcc ;  -1:  Orientation detect failed; 0:x up ; 1;x down ; 2:y up  ; 3 y down; 4 z up;  5 z down;  6  cal done;   10: one ori detect success
MPU6000::MPU6000( HAL_QSPI *_dev)
{
	dev = _dev;
	
	acc_healthy = false;
	gyro_healthy = false;
	accelT.identity();
	
}

void MPU6000::initialize(void)
{
	OS_ERR os_err;

//	EEPROM *rom = EEPROM::get_instance();
	
	healthyTestPassed = false;
	uint8_t accel_acl_flag = 0;
	
	dev->initialize();
	dev->setBaudRate(1000000);
	
	dev->writeData( MPU6000_PWR_MGMT_1, 0x80);// Reset Device
	OSTimeDlyHMSM( 0u, 0u, 0u, 50u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	
	dev->writeData( MPU6000_PWR_MGMT_1, 0x00);
	dev->writeData( MPU6000_PWR_MGMT_2, 0x00);	// Enable Acc & Gyro
	dev->writeData( MPU6000_USER_CTRL, 0x10);	// disable I2c
	dev->writeData( MPU6000_CONFIG, 0x00);		// no filter

#if 0	
	uint8_t healthyTestRepeats = 10;
	for(uint8_t i = 0; i < healthyTestRepeats; i++)
	{
		if(MPU6000_SelfTest())
		{
			healthyTestPassed = true;
			break;
		}	
	}
#endif
	
	readGyroID();   //  for mpu6000 0x68 = (uint8_t)MPU6000_ReadGyroID();	for mpu6500 ID = 0x70
	
	do
	{
		dev->writeData( MPU6000_GYRO_CONFIG, 0x10);
		OSTimeDlyHMSM( 0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		readGyroRange();
	}while( gyroRange != 1000 );
	
	do
	{
		dev->writeData( MPU6000_ACCEL_CONFIG, 0x18);
		OSTimeDlyHMSM( 0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		readAccRange();
		
	}while( accleRange != 16 );
	
	acc_healthy = true;
	gyro_healthy = true;
	
	dev->setBaudRate(10000000);
	
	MPU6000::autoCalibrationOffset();
	
	
	
//	TWI_Clear();
	
//	accel_acl_flag = 1;
//	TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x00, 1, &accel_acl_flag, 0x01, 0);
//	TWI_Read(&IIC0, EEPROM_DEVICE_ADDRESS, 0x00, 1, &accel_acl_flag, 0x01, 0);
	accel_acl_flag = 0;
	if(!accel_acl_flag)
  {
			if(MPU6000::accelCalibration())
			{
					accel_acl_flag = 1;
					//TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x00, 1, &accel_acl_flag, 0x01, 0);

			}
		  
	}
//	float accel_offset_r[3];
//	float accel_T_r[3][3];
//	float accel_offset_w[3] = {-0.0103478432,-0.26094532,0.54412508};
//	float accel_T_w[3][3] = {{0.999822438,-0.00886511989,-0.0231227372},
//													 {0.00858588051,0.998066604,-0.0149211921},
//	                         {0.0109922495,0.00212378986,0.992062569}};
	
//	TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x40 , 1, (uint8_t *)accel_offset_w, 0x08, 0);
//	TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x48 , 1, (uint8_t *)(accel_offset_w+2), 0x04, 0);
	
//	for(int i=0;i<3;i++)
//  {
//		TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x10+16*i , 1, (uint8_t *)accel_T_w[i], 0x08, 0);
//		TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x18+16*i , 1, (uint8_t *)(accel_T_w[i]+2), 0x04, 0);
//	}
//	
//	TWI_Read(&IIC0, EEPROM_DEVICE_ADDRESS, 0x40 , 1, (uint8_t *)accel_offset_r, 0x08, 0);
//	TWI_Read(&IIC0, EEPROM_DEVICE_ADDRESS, 0x48 , 1, (uint8_t *)(accel_offset_r+2), 0x04, 0);
//	for(int i=0;i<3;i++)
//  {
//		TWI_Read(&IIC0, EEPROM_DEVICE_ADDRESS, 0x10+16*i , 1, (uint8_t *)accel_T_r[i], 0x08, 0);
//		TWI_Read(&IIC0, EEPROM_DEVICE_ADDRESS, 0x18+16*i , 1, (uint8_t *)(accel_T_r[i]+2), 0x04, 0);
//	}
	
//	accelOffset = Vector3f(accel_offset_r[0],accel_offset_r[1],accel_offset_r[2]);
	
//	accelT = Matrix3f(accel_T_r[0][0],accel_T_r[1][0],accel_T_r[2][0],
//										accel_T_r[0][1],accel_T_r[1][1],accel_T_r[2][1],
//										accel_T_r[0][2],accel_T_r[1][2],accel_T_r[2][2]);

//	accelScale = Vector3f(accel_T_r[0][0],accel_T_r[1][1],accel_T_r[2][2]);
	
}
uint8_t MPU6000::readGyroRange( void )
{
	uint8_t tmp = 0;
	uint8_t reg = MPU6000_GYRO_CONFIG | 0x80;

	dev->readData( reg, &tmp, 1);
	
	switch(tmp&0x18)
	{
		case 0x00: gyroRange = 250; break;
		case 0x08: gyroRange = 500; break;
		case 0x10: gyroRange = 1000;break;
		case 0x18: gyroRange = 2000; break;
		default: break;
	}
	
	
	return 0;
}

uint8_t MPU6000::readAccRange( void )
{
	uint8_t tmp = 0;
	uint8_t reg = MPU6000_ACCEL_CONFIG | 0x80;

	dev->readData( reg, &tmp, 1);
	
	switch(tmp&0x18)   // +-2G 0x00; +-4G 0x08; +-8G 0x10; +-16G 0x18
	{
		case 0x00: accleRange = 2; break;
		case 0x08: accleRange = 4; break;
		case 0x10: accleRange = 8; break;
		case 0x18: accleRange = 16; break;
    default: break;
	}
	
	
	return 0;
}

uint8_t MPU6000::readGyroID( void )
{
	uint8_t reg = MPU6000_WHO_AM_I | 0x80;

	dev->readData( reg, &id, 1);
	
	return(id);
}


uint8_t MPU6000::readAccelRaw( void )
{
	uint8_t temp[6];
	uint8_t reg = MPU6000_ACCEL_XOUT_H | 0x80;
	
	dev->readData( reg, temp, 6);
	
	return 0;
}

uint8_t MPU6000::readTempRaw( void )
{
	uint8_t temp[2];
	uint8_t reg = MPU6000_TEMP_OUT_H | 0x80;
	
	dev->readData( reg, temp, 2 );
	
	tempRawData = (float)( (int16_t)(uint16_t(temp[0])<<8 | temp[1]) );
	
	return 0;
}


uint8_t MPU6000::readGyroRaw( void )
{
	uint8_t temp[6];
	uint8_t reg = MPU6000_GYRO_XOUT_H | 0x80;
	
	dev->readData( reg, temp, 6 );
	
	
	return 0;
}

uint8_t MPU6000::readRaw( void )
{
	uint8_t temp[14];
	uint8_t reg = MPU6000_ACCEL_XOUT_H | 0x80;
	
	dev->readData( reg, temp, 14 );
	memcpy( rawDataBuffer, temp, 14 );
	
	// x' = -y
	// y' = -x
	// z' = -z
	
//	accelRawData.x = -(float)( (int16_t)(uint16_t(temp[0])<<8 | temp[1]) );
//	accelRawData.y =  (float)( (int16_t)(uint16_t(temp[2])<<8 | temp[3]) );
//	accelRawData.z = -(float)( (int16_t)(uint16_t(temp[4])<<8 | temp[5]) );
//	
//	tempRawData = (float)( (int16_t)((uint16_t)temp[6]<<8 | temp[7]) );

//	gyroRawData.x = -(float)( (int16_t)(uint16_t(temp[8]) <<8 | temp[9]) );
//	gyroRawData.y =  (float)( (int16_t)(uint16_t(temp[10])<<8 | temp[11]) );
//	gyroRawData.z = -(float)( (int16_t)(uint16_t(temp[12])<<8 | temp[13]) );
	
	accelRawData.x = (float)( (int16_t)(uint16_t(temp[0])<<8 | temp[1]) );
	accelRawData.y = -(float)( (int16_t)(uint16_t(temp[2])<<8 | temp[3]) );
	accelRawData.z = -(float)( (int16_t)(uint16_t(temp[4])<<8 | temp[5]) );
	
	tempRawData = (float)( (int16_t)((uint16_t)temp[6]<<8 | temp[7]) );
	
	gyroRawData.x = (float)( (int16_t)(uint16_t(temp[8]) <<8 | temp[9]) ); 
	gyroRawData.y = -(float)( (int16_t)(uint16_t(temp[10])<<8 | temp[11]) );
	gyroRawData.z = -(float)( (int16_t)(uint16_t(temp[12])<<8 | temp[13]) );
	
	
	imuLastUpdate = hrt_absolute_time();
	
	return 0;
}


void MPU6000::getAccelRawData(Vector3f* _accelRawData)
{
	*_accelRawData = accelRawData; 
}

void MPU6000::getTempRawData(float* _tempRawData)
{
	*_tempRawData = tempRawData;
}

void MPU6000::getGryoRawData(Vector3f* _gyroRawData)
{
	*_gyroRawData = gyroRawData;
	
}

void MPU6000::getRawData(Vector3f* _accelRawData, float* _tempRawData, Vector3f* _gyroRawData)
{
	*_accelRawData = accelRawData;
	*_tempRawData = tempRawData;
	*_gyroRawData = gyroRawData;
}


void MPU6000::autoCalibrationOffset( void )
{
	OS_ERR os_err;
	Vector3f mpu6000_gyro_cal_data_sum(0.0f, 0.0f, 0.0f);
	Vector3f mpu6000_gyro_cal(0.0f, 0.0f, 0.0f);
	Vector3f mpu6000_gyro_cal_last(0.0f, 0.0f, 0.0f);
	Vector3f goryError(0.0f, 0.0f, 0.0f);
	
	Vector3f mpu6000_accel_cal_data_sum(0.0f, 0.0f, 0.0f);
	Vector3f mpu6000_accel_cal(0.0f, 0.0f, 0.0f);
	Vector3f mpu6000_accel_cal_last(0.0f, 0.0f, 0.0f);
	Vector3f accelError(0.0f, 0.0f, 0.0f);
	
	
	uint16_t cnt = 0;
	
	gyroCalibate_OK = false;

	while(!gyroCalibate_OK)
	{
		for(cnt=0; cnt<500; cnt++)
		{
			readRaw();
			mpu6000_gyro_cal_data_sum += gyroRawData;
			OSTimeDlyHMSM( 0u, 0u, 0u, 2u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		}
		mpu6000_gyro_cal = mpu6000_gyro_cal_data_sum/cnt;
		mpu6000_gyro_cal_data_sum = Vector3f(0.0f, 0.0f, 0.0f);
		
		goryError = mpu6000_gyro_cal_last - mpu6000_gyro_cal;
		if( goryError.length()<0.5f )
		{
			gyroCalibate_OK = true;
		}
		else
		{
			gyroCalibate_OK = false;
		}
		
		mpu6000_gyro_cal_last = mpu6000_gyro_cal;
	}
	gyroOffset = mpu6000_gyro_cal_last;
}
	

// // selfTest passed return 1, otherwise return 0;
uint8_t MPU6000::selfTest()
{
	OS_ERR os_err;
	float  gyroBase[3];
	float  accelBase[3];
	float  gyro[3];
	float  accel[3];
	const uint16_t repeats = 100;
	uint8_t trim[4] = {0};
	uint8_t aTest[3] ;
	uint8_t gTest[3] ;
	
	float accel_ftrim[3];
	float gyro_ftrim[3];
	
	uint8_t  mpuHealthy = 0;
	
	
	memset(&gyroBase,0,sizeof(gyroBase));
	memset(&accelBase,0,sizeof(accelBase));
	memset(&gyro,0,sizeof(gyro));
	memset(&accel,0,sizeof(accel));
	// get base value without self-test enabled
	dev->writeData( MPU6000_GYRO_CONFIG, BITS_FS_250DPS);
	dev->writeData( MPU6000_ACCEL_CONFIG, 0x10);  // +-8g
	
	OSTimeDlyHMSM( 0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	
	for(uint8_t i = 0; i < repeats; i++)
	{
		readAccelRaw();
		readGyroRaw();
		gyroBase[0] += gyroRawData.x;
		gyroBase[1] += gyroRawData.y;
		gyroBase[2] += gyroRawData.z;
		
		accelBase[0] += accelRawData.x;
		accelBase[1] += accelRawData.y;
		accelBase[2] += accelRawData.z;
	}
	 // get self-test values
	dev->writeData( MPU6000_GYRO_CONFIG, BITS_FS_250DPS | 0xE0); 
	dev->writeData( MPU6000_GYRO_CONFIG, 0x10 | 0xE0); 
	
	OSTimeDlyHMSM( 0u, 0u, 0u, 20u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	
	for(uint8_t i = 0; i < repeats; i++)
	{
//		MPU6000_ReadAccelRaw();
//		MPU6000_ReadGyroRaw();
		gyro[0] += gyroRawData.x;
		gyro[1] += gyroRawData.y;
		gyro[2] += gyroRawData.x;
		
		accel[0] += accelRawData.x;
		accel[1] += accelRawData.y;
		accel[2] += accelRawData.x;
	}
	for(uint8_t i=0; i < 3; i++)
	{
		gyroBase[i] *= 0.01f;
		accelBase[i] *= 0.01f;
		gyro[i] *= 0.01f;
		accel[i] *= 0.01f;
	}
	
	dev->readData(0x0D, trim, 4);
	
	aTest[0] = (uint8_t)(((trim[0] >> 3) & 0x001C) | ((trim[3] >> 4) & 0x0003));
	aTest[1] = (uint8_t)(((trim[1] >> 3) & 0x001C) | ((trim[3] >> 2) & 0x0003));
	aTest[2] = (uint8_t)(((trim[2] >> 3) & 0x001C) | ((trim[3] >> 0) & 0x0003));
	gTest[0] = (uint8_t)(trim[0] & 0x001F);
	gTest[1] = (uint8_t)(trim[1] & 0x001F);
	gTest[2] = (uint8_t)(trim[2] & 0x001F);
	
	for(uint8_t i = 0;i < 3; i++)
	{
		accel_ftrim[i] = 4096 * 0.34f * pow(0.92f/0.34f, (aTest[i]-1)/30.0f);
		gyro_ftrim[i] = 25 * 131.0f * pow(1.046f, gTest[i]-1);
	}
	gyro_ftrim[1] *= -1;
	
	for(uint8_t i = 0; i < 3 ; i++)
	{
		float diff, err;
		diff = accel[i] - accelBase[i];
		err = 100.0f * (diff - accel_ftrim[i]) /  accel_ftrim[i];
		if(fabsf(err) > 14.0f)
		{
			mpuHealthy++;
		}
	}
	for(uint8_t i = 0; i < 3 ; i++)
	{
		float diff, err;
		diff = gyro[i] - gyroBase[i];
		err = 100.0f * (diff - gyro_ftrim[i]) /  gyro_ftrim[i];
		
		if(fabsf(err) > 14.0f)
		{
			mpuHealthy++;
		}
	}
	if(mpuHealthy > 0)
	{
		return 0;
	}
	else 
	{
		return 1;
	}
	
}


void MPU6000::update(void)
{
	readRaw();
	accelData = accelRawData*MPU6000_16g;
	tempData = tempRawData/340 + 36.53f;
	gyroData = (gyroRawData-gyroOffset)*MPU6000_1000rps;
	
	accelFilterData = accel_filter.apply(accelData);
	gyroFilterData = gyro_filter.apply(gyroData);
}








/*  
  theoryvalue = T * (measurevalue - offset)      

   [g 0 0]           [(ref[0][0]-offs[0]) (ref[2][0]-offs[0]) (ref[4][0]-offs[0])]
   [0 g 0] =T[3][3] *[(ref[0][1]-offs[1]) (ref[2][1]-offs[1]) (ref[4][1]-offs[1])]      
   [0 0 g]           [(ref[0][2]-offs[2]) (ref[2][2]-offs[2]) (ref[4][2]-offs[2])]
   
   offs[0]=1/2(ref[0][0]+ref[1][0]);
   offs[1]=1/2(ref[2][1]+ref[3][1]);
   offs[2]=1/2(ref[4][2]+ref[5][2]);
   
   [g 0 0]           
   [0 g 0] =T*mat_A;  gE = T*mat_A
   [0 0 g]           
   T = g*(mat_A)^-1;
*/
// // do accel calibration return T  and offset;
bool MPU6000::accelCalibration()
{
	bool res = true;
	float accel_offset[3];
	float accel_T[3][3];
	float accel_ref[6][3];
	OS_ERR os_err;

//start calAcc**************	
	calAccOrientationFlag = -2;
	OSTimeDlyHMSM( 0u, 0u, 0u, 1000u, OS_OPT_TIME_HMSM_STRICT, &os_err);
//	ledPD1.LED_on();
//	delayMs(1000);
//	ledPD1.LED_off(); 
	res = accelGetMeasurement(accel_ref);   //get accel_ref[6][3]
	if(res)
  {
			res = accelCalValues(accel_ref,accel_T,accel_offset);
		  if(!res) return false;
	}else
	{
			return false;
	}
	
//	TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x40 , 1, (uint8_t *)accel_offset, 0x08, 0);
//	TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x48 , 1, (uint8_t *)(accel_offset+2), 0x04, 0);
//	
//	for(int i=0;i<3;i++)
//  {
//		TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x10+16*i , 1, (uint8_t *)accel_T[i], 0x08, 0);
//		TWI_Write(&IIC0, EEPROM_DEVICE_ADDRESS, 0x18+16*i , 1, (uint8_t *)(accel_T[i]+2), 0x04, 0);
//	}
	return true;
}
//get accel_ref[6][3]
//first judge which orientation aircraft is in    then get the measure value 
//this method let you can change orientation freely, you only need make it detect six orientation
bool MPU6000::accelGetMeasurement(float (&_accel_ref)[6][3])
{
	bool orientation_station[6] = {false,false,false,false,false,false};  //"x+", "x-", "y+", "y-", "z+", "z-"
	bool done = false;
	const int samples_num = 2500;
	OS_ERR os_err;
	
	calAccOrientationFlag = -1;
	OSTimeDlyHMSM( 0u, 0u, 0u, 1000u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	
	while(true)           
  {
		done = true;
		int orientation = accelDetectOrientation();  
		if(orientation < 0)
		{
				return false;         //detect orientation false
		}
		
		accelReadAvg(_accel_ref[orientation],samples_num); //get accel data in the detected orientation
		orientation_station[orientation] = true;
//		ledPD1.LED_on();
//		delayMs(500);
//		ledPD1.LED_off();
//		delayMs(500);
//		ledPD1.LED_on();
//		delayMs(500);
//		ledPD1.LED_off();
//		delayMs(500);
	   calAccOrientationFlag = 10;  //
	   OSTimeDlyHMSM( 0u, 0u, 0u, 1000u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		
		for(int i=0; i<6; i++)
		{
			if(!orientation_station[i])
			{
					done = false;
			}
		}
		if(done)
		{
//			ledPD1.LED_on();
//		  delayMs(1000);
//			ledPD1.LED_off();
//			delayMs(1000);
			calAccOrientationFlag =6;
			OSTimeDlyHMSM( 0u, 0u, 0u, 1000u, OS_OPT_TIME_HMSM_STRICT, &os_err);
			return true;                
		}                 //six orientation is measured          	
	}
}
//// if detect orientation false ,the all calibrate is false
int  MPU6000::accelDetectOrientation()
{
	float accel_data[3] = {0.0f,0.0f,0.0f};  //accel measurement
  float accel_ema[3] = {0.0f,0.0f,0.0f};  //exponential moving average of accel
	float accel_disp[3] = {0.0f,0.0f,0.0f}; //max-hold dispersion of accel  /bias
	float ema_len = 0.5f;  //EMA time constant in seconds
	float still_thr2 = pow(0.25f,2); //set "still" threshold to 0.25 m/s^2 
	float accel_err_thr = 5.0f;   //set accel error threshold to 5m/s^2 
	uint64_t still_time_us = 5000000; //still time required in 5s
	uint64_t t_start_us = hrt_absolute_time();
	uint64_t timeout_us = 30000000;  //set timeout to 30s
	uint64_t t_timeout_us = t_start_us + timeout_us;
	uint64_t t_us = t_start_us;
	uint64_t t_prev_us = t_start_us;
	uint64_t t_still_us = 0;   
	
	unsigned poll_errcount = 0;
	OS_ERR os_err;
	
	while(true)
	{
//			delayMs(4);
//			MPU6000_ReadRaw();
		  OSTimeDlyHMSM( 0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		  readRaw();
			accel_data[0] = accelRawData.x * MPU6000_16g;
			accel_data[1] = accelRawData.y * MPU6000_16g;
			accel_data[2] = accelRawData.z * MPU6000_16g;    //measure orinal data
			
			t_us = hrt_absolute_time();  // record now time
			float dt = (t_us - t_prev_us)/1000000.0f;  
			t_prev_us = t_us;
			float w = dt/ema_len;    //weight
			
			for(int i=0;i<3;i++)
			{
					float d = accel_data[i] - accel_ema[i];
					accel_ema[i] = accel_ema[i] + w * d;       //smooth  weight average / EMA_now = w * measurement_now + (1-w) * EMA_prev 
					d = d * d;           // expode +-
					accel_disp[i] = accel_disp[i] * (1 - w);  //bias between measurement_now and EMA_prev
					if (d > still_thr2 * 8.0f) 
					{
							d = still_thr2 * 8.0f;   
					}
					if (d > accel_disp[i]) 
					{
							accel_disp[i] = d;  //hold max bias
					}
			}
			//if accel_disp[i] is small and still >2s shows aircraft is stable so you can record current accel data
			//if accel_disp[i] is big shows aircraft may be moving you need wait			
			if((accel_disp[0] < still_thr2)&&(accel_disp[1] < still_thr2)&&(accel_disp[2] < still_thr2))  //bias is small
			{
					if(t_still_us == 0)  //prevous bias is not small
					{
							t_still_us = t_us;
						  t_timeout_us = t_us + timeout_us;
					}else
					{
							if(t_us > t_still_us + still_time_us) //small bias is still > 2s so data sample success
							{
									break;
							}
					}
			}else if((accel_disp[0] > still_thr2 * 4.0f)||(accel_disp[0] > still_thr2 * 4.0f)||(accel_disp[0] > still_thr2 * 4.0f))
			{
					if(t_still_us != 0)   //prevous bias is small,but now bias is big so data sample again
					{
							t_still_us = 0;
					}
			}
			if(t_us > t_timeout_us)  //while cycle time >30s record once error
			{ 
					poll_errcount++;  
			}
			if(poll_errcount> 50) //while record error >100 false
			{
					return -1;
			}
	}
	if ((fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr) && (fabsf(accel_ema[1]) < accel_err_thr) && (fabsf(accel_ema[2]) < accel_err_thr)) {
	//	delayMs(1000);
	  calAccOrientationFlag = 0;
	  return calAccOrientationFlag;        // correspond [ g, 0, 0 ] //x up
	}

	if ((fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr) && (fabsf(accel_ema[1]) < accel_err_thr) && (fabsf(accel_ema[2]) < accel_err_thr)) 
	{
//		ledPD1.LED_on();
//		delayMs(1000);
//		ledPD1.LED_off();
		calAccOrientationFlag = 1;
	  return calAccOrientationFlag;     // [ -g, 0, 0 ]  //x down
	}

	if ((fabsf(accel_ema[0]) < accel_err_thr) && (fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr) && (fabsf(accel_ema[2]) < accel_err_thr)) 
	{
//		ledPD1.LED_on();
//		delayMs(1000);
//		ledPD1.LED_off();
		calAccOrientationFlag = 2;
	  return calAccOrientationFlag;      // [ 0, g, 0 ]  //Y up
	}

	if ((fabsf(accel_ema[0]) < accel_err_thr) && (fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr) && (fabsf(accel_ema[2]) < accel_err_thr)) 
	{
//		ledPD1.LED_on();
//		delayMs(1000);
//		ledPD1.LED_off();
		calAccOrientationFlag = 3;
	  return calAccOrientationFlag;       // [ 0, -g, 0 ]  //Y down
	}

	if ((fabsf(accel_ema[0]) < accel_err_thr) && (fabsf(accel_ema[1]) < accel_err_thr) && (fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr))
	{
//		ledPD1.LED_on();
//		delayMs(1000);
//		ledPD1.LED_off();
		calAccOrientationFlag = 4;
	  return calAccOrientationFlag;       // [ 0, 0, g ]   //Z up
	}

	if ((fabsf(accel_ema[0]) < accel_err_thr) && (fabsf(accel_ema[1]) < accel_err_thr) && (fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr)) 
	{
//		ledPD1.LED_on();
//		delayMs(1000);
//		ledPD1.LED_off();
		calAccOrientationFlag = 5;
	  return calAccOrientationFlag;        // [ 0, 0, -g ]   //Z down
	}
	
	return calAccOrientationFlag;
}

//// get measure accel average value   
void  MPU6000::accelReadAvg(float (&accel_avg)[3],int samples_num)
{
	float accel_sum[3] = {0.0f,0.0f,0.0f};
	float accel_data[3] = {0.0f,0.0f,0.0f};
	int count = 0;
	OS_ERR os_err;
	
	while(count < samples_num)
	{
			//delayMs(4);
		  OSTimeDlyHMSM( 0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &os_err);
		  //MPU6000_ReadRaw();
		  readRaw();
			accel_data[0] = accelRawData.x * MPU6000_16g;
			accel_data[1] = accelRawData.y * MPU6000_16g;
			accel_data[2] = accelRawData.z * MPU6000_16g;
			for(int i = 0; i < 3; i++)
			{
					accel_sum[i] += accel_data[i];	
			}
			count++;
	}
	for(int i = 0; i < 3; i++)
	{
			accel_avg[i] = accel_sum[i]/count;
	}
}

bool mat_invert3(float (&src)[3][3], float (&dst)[3][3]);
bool  MPU6000::accelCalValues(float (&_accel_ref)[6][3],float (&_accel_T)[3][3],float (&_accel_offset)[3])
{
		for(int i = 0; i < 3; i++)
		{
				_accel_offset[i] = (_accel_ref[i*2][i] + _accel_ref[i*2+1][i])/2;
		}
		float mat_A[3][3];
		for(int i = 0; i < 3; i++)
		{
				for(int j = 0; j < 3; j++)
				{
						mat_A[i][j] = _accel_ref[i*2][j] - _accel_offset[j];
				}
		}
		
		float mat_A_inv[3][3];
		if(mat_invert3(mat_A, mat_A_inv))
		{
				for(int i = 0; i < 3; i++)
				{
						for(int j = 0; j < 3; j++)
						{
								_accel_T[i][j] = mat_A_inv[i][j] * CONSTANTS_ONE_G;
						}
				}
				return true;
		}else return false;
}

bool mat_invert3(float (&src)[3][3], float (&dst)[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return false;        // Singular matrix
	}

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return true;
}

