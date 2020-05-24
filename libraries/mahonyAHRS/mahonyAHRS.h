#ifndef _MAHONY_AHRS_H
#define _MAHONY_AHRS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define IMUPI 3.1415926f
typedef struct
{
	float gx, gy, gz;
	float ax, ay, az;
	float mx, my, mz;
	
	float accelIntegralX;
	float accelIntegralY;
	float accelIntegralZ;
	
	float magIntegralX;
	float magIntegralY;
	float magIntegralZ;
	
	float q0, q1, q2, q3;
	float roll, pitch, yaw;
	
	float accelKp;
	float accelKi;
	uint8_t accelKiEnable;
	
	uint8_t useMag;
	float magKp;
	float magKi;
	uint8_t magKiEnable;
	
	uint8_t initialRPYFlag;

} imu_ahrs_t;


void ahrsInit( imu_ahrs_t* _gdu_ahrs);
void ahrsInitRPY(imu_ahrs_t* _gdu_ahrs);
void ahrsUpdateData( imu_ahrs_t* _gdu_ahrs, float* sensorData );
void MahonyAHRSupdate( imu_ahrs_t* _gdu_ahrs, float deltaT);

#ifdef __cplusplus
}
#endif
#endif //_MAHONY_AHRS_H