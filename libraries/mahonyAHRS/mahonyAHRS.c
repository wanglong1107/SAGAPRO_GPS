#include <math.h>
#include <string.h>
#include "mahonyAHRS.h"


static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}


void ahrsInitRPY( imu_ahrs_t* _gdu_ahrs)
{
	float recipNorm;
	float q0, q1, q2, q3;
	static uint8_t InitDataCnt = 0;
	static float ax = 0.0f, ay = 0.0f, az = 0.0f , mx = 0.0f , my = 0.0f, mz = 0.0f;
	float init_roll = 0.0f, init_pitch = 0.0f ,init_yaw = 0.0f;

	if(InitDataCnt < 10)
	{
		ax += _gdu_ahrs->ax;
		ay += _gdu_ahrs->ay;
		az += _gdu_ahrs->az;
		mx += _gdu_ahrs->mx;
		my += _gdu_ahrs->my;
		mz += _gdu_ahrs->mz;
		InitDataCnt++;
		//trace(0,"%d ",InitDataCnt);
	}
	else
	{	
		ax = ax / 10.0f;
		ay = ay / 10.0f;
		az = az / 10.0f;
		mx = mx / 10.0f;
		my = my / 10.0f;
		mz = mz / 10.0f;
		
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		
		init_roll = atan2f(ay, az);
		init_pitch = asinf(-ax);
		//init_roll = 0;
		//init_pitch = 0;
		
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
		
		float sin_roll, cos_roll, sin_pitch, cos_pitch;
		float magX, magY;
		sin_roll = sin(init_roll);
		cos_roll = cos(init_roll);
		cos_pitch = cos(init_pitch);
		sin_pitch = sin(init_pitch);
		magX = mx * cos_pitch + my*sin_pitch*sin_roll + mz*sin_pitch*cos_roll;
		magY = my * cos_roll - mz*sin_roll;
		init_yaw = atan2f(-magY, magX);
		//trace(0,".3f	%.3f	%.3f",mx,my,mz);
		//trace(0,"%.3f	%.3f	%.3f",init_roll,init_pitch,init_yaw);
		/*
		if(magX!=0)
			init_yaw = atan(fabsf(magY/magX));
		else if(magY>0)
			init_yaw = -IMUPI/2;
		else
			init_yaw = IMUPI/2;
			*/
		
		//trace(0,"test 2");
		float cr2 = cosf(init_roll*0.5f);
		float cp2 = cosf(init_pitch*0.5f);
		float cy2 = cosf(init_yaw*0.5f);
		float sr2 = sinf(init_roll*0.5f);
		float sp2 = sinf(init_pitch*0.5f);
		float sy2 = sinf(init_yaw*0.5f);

		q0 = cr2*cp2*cy2 + sr2*sp2*sy2;
		q1 = sr2*cp2*cy2 - cr2*sp2*sy2;
		q2 = cr2*sp2*cy2 + sr2*cp2*sy2;
		q3 = cr2*cp2*sy2 - sr2*sp2*cy2;
		
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		
		_gdu_ahrs->q0 = q0;
		_gdu_ahrs->q1 = q1;
		_gdu_ahrs->q2 = q2;
		_gdu_ahrs->q3 = q3;
		_gdu_ahrs->initialRPYFlag = 1; //初始化完成标志
	}
}

void ahrsInit( imu_ahrs_t* _gdu_ahrs)
{//只会执行一次
	
	_gdu_ahrs->q0 = 1.0f;//1.0f;
	_gdu_ahrs->q1 = 0.0f;
	_gdu_ahrs->q2 = 0.0f;
	_gdu_ahrs->q3 = 0.0f;
	
	_gdu_ahrs->accelIntegralX = 0.0f;
	_gdu_ahrs->accelIntegralY = 0.0f;
	_gdu_ahrs->accelIntegralZ = 0.0f;
	
	_gdu_ahrs->magIntegralX = 0.0f;
	_gdu_ahrs->magIntegralY = 0.0f;
	_gdu_ahrs->magIntegralZ = 0.0f;

	
	_gdu_ahrs->yaw = 0.0f;
	_gdu_ahrs->pitch = 0.0f;
	_gdu_ahrs->roll = 0.0f;

	_gdu_ahrs->accelKp = 0.35f;
	_gdu_ahrs->accelKi = 0.0001f;
	_gdu_ahrs->accelKiEnable = 1;
	
	
	_gdu_ahrs->useMag = 1;
	_gdu_ahrs->magKp = 0.21f;
	_gdu_ahrs->magKi = 0.000135f;
	_gdu_ahrs->magKiEnable = 0;
	
	_gdu_ahrs->initialRPYFlag = 0;
}

void ahrsUpdateData( imu_ahrs_t* _gdu_ahrs, float* sensorData )
{//更新数据
	_gdu_ahrs->gx = sensorData[0];
	_gdu_ahrs->gy = sensorData[1];
	_gdu_ahrs->gz = sensorData[2];
	
	_gdu_ahrs->ax = -sensorData[3];
	_gdu_ahrs->ay = -sensorData[4];
	_gdu_ahrs->az = -sensorData[5];
	
	_gdu_ahrs->mx = sensorData[6];
	_gdu_ahrs->my = sensorData[7];
	_gdu_ahrs->mz = sensorData[8];
}

void MahonyAHRSupdate( imu_ahrs_t* _gdu_ahrs, float deltaT)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float qa, qb, qc, qd;

	float estimatedAccelX;
	float estimatedAccelY;
	float estimatedAccelZ;
	
	float accelErrorX;
	float accelErrorY;
	float accelErrorZ;
	
	float earthReferenceDirectionMagX;
	float earthReferenceDirectionMagY;
	float earthReferenceDirectionMagZ;
	float bodyReferenceDirectionMagX;
	float bodyReferenceDirectionMagY;
	float bodyReferenceDirectionMagZ;
	
	float estimatedMagX;
	float estimatedMagY;
	float estimatedMagZ;
	
	float magErrorX;
	float magErrorY;
	float magErrorZ;	
	
	float correctedGyroX = 0.0f;
	float correctedGyroY = 0.0f;
	float correctedGyroZ = 0.0f;

	
	float gx = _gdu_ahrs->gx;
	float gy = _gdu_ahrs->gy;
	float gz = _gdu_ahrs->gz;
	
	float ax = _gdu_ahrs->ax;
	float ay = _gdu_ahrs->ay;
	float az = _gdu_ahrs->az;
	
	float mx = _gdu_ahrs->mx;
	float my = _gdu_ahrs->my;
	float mz = _gdu_ahrs->mz;	
	
	if( 0 == _gdu_ahrs->initialRPYFlag)
	{//进行初始化
		ahrsInitRPY(_gdu_ahrs);
	}
	else
	{//初始化完成	
		
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		
		float q0 = _gdu_ahrs->q0;
		float q1 = _gdu_ahrs->q1;
		float q2 = _gdu_ahrs->q2;
		float q3 = _gdu_ahrs->q3;	
		
		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;	

		// Estimated direction of gravity
		estimatedAccelX = q1q3 - q0q2;
		estimatedAccelY = q0q1 + q2q3;
		estimatedAccelZ = q0q0 - 0.5f + q3q3;
		
		accelErrorX = ay * estimatedAccelZ - az * estimatedAccelY;
		accelErrorY = az * estimatedAccelX - ax * estimatedAccelZ;
		accelErrorZ = ax * estimatedAccelY - ay * estimatedAccelX;
		
		if(_gdu_ahrs->accelKiEnable)
		{
			_gdu_ahrs->accelIntegralX += accelErrorX * _gdu_ahrs->accelKi * deltaT;
			_gdu_ahrs->accelIntegralY += accelErrorY * _gdu_ahrs->accelKi * deltaT;
			_gdu_ahrs->accelIntegralZ += accelErrorZ * _gdu_ahrs->accelKi * deltaT;
		}
		else
		{
			_gdu_ahrs->accelIntegralX = 0.0f;
			_gdu_ahrs->accelIntegralY = 0.0f;
			_gdu_ahrs->accelIntegralZ = 0.0f;
			
		}
		
		correctedGyroX =  gx + accelErrorX * _gdu_ahrs->accelKp + _gdu_ahrs->accelIntegralX;
		correctedGyroY =  gy + accelErrorY * _gdu_ahrs->accelKp + _gdu_ahrs->accelIntegralY;
		correctedGyroZ =  gz + accelErrorZ * _gdu_ahrs->accelKp + _gdu_ahrs->accelIntegralZ;
			
		if(_gdu_ahrs->useMag)
		{
			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
			
			// Reference direction of Earth鈥檚 magnetic field
			earthReferenceDirectionMagX = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			earthReferenceDirectionMagY = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			earthReferenceDirectionMagZ = 0.0f;
			bodyReferenceDirectionMagX = sqrt(earthReferenceDirectionMagX * earthReferenceDirectionMagX + earthReferenceDirectionMagY * earthReferenceDirectionMagY);
			bodyReferenceDirectionMagY = 0.0f;
			bodyReferenceDirectionMagZ = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
			
			// Estimated direction of magnetic field
			estimatedMagX = bodyReferenceDirectionMagX * (0.5f - q2q2 - q3q3) + bodyReferenceDirectionMagZ * (q1q3 - q0q2);
			estimatedMagY = bodyReferenceDirectionMagX * (q1q2 - q0q3) + bodyReferenceDirectionMagZ * (q0q1 + q2q3);
			estimatedMagZ = bodyReferenceDirectionMagX * (q0q2 + q1q3) + bodyReferenceDirectionMagZ * (0.5f - q1q1 - q2q2);
			
//			// Estimated direction of magnetic field
//			estimatedMagX = 0.5f - q2q2 - q3q3;
//			estimatedMagY = q1q2 - q0q3;
//			estimatedMagZ = q0q2 + q1q3;


			// Error is sum of cross product between estimated direction and measured direction of field vectors
			magErrorX = my * estimatedMagZ - mz * estimatedMagY;
			magErrorY = mz * estimatedMagX - mx * estimatedMagZ;
			magErrorZ = mx * estimatedMagY - my * estimatedMagX;
		
			if(_gdu_ahrs->magKiEnable)
			{
				_gdu_ahrs->magIntegralX += magErrorX * _gdu_ahrs->magKi * deltaT;
				_gdu_ahrs->magIntegralY += magErrorY * _gdu_ahrs->magKi * deltaT;
				_gdu_ahrs->magIntegralZ += magErrorZ * _gdu_ahrs->magKi * deltaT;
			}
			else
			{
				_gdu_ahrs->magIntegralX = 0.0f;
				_gdu_ahrs->magIntegralY = 0.0f;
				_gdu_ahrs->magIntegralZ = 0.0f;
			}
			
			correctedGyroX +=  magErrorX * _gdu_ahrs->magKp + _gdu_ahrs->magIntegralX;
			correctedGyroY +=  magErrorY * _gdu_ahrs->magKp + _gdu_ahrs->magIntegralY;
			correctedGyroZ +=  magErrorZ * _gdu_ahrs->magKp + _gdu_ahrs->magIntegralZ;
		}

		// Integrate rate of change of quaternion
		correctedGyroX *= (0.5f * deltaT); // pre-multiply common factors
		correctedGyroY *= (0.5f * deltaT);
		correctedGyroZ *= (0.5f * deltaT);
		qa = q0;
		qb = q1;
		qc = q2;
		qd = q3;
		q0 += (-qb * correctedGyroX - qc * correctedGyroY - qd * correctedGyroZ);
		q1 += ( qa * correctedGyroX + qc * correctedGyroZ - qd * correctedGyroY);
		q2 += ( qa * correctedGyroY - qb * correctedGyroZ + qd * correctedGyroX);
		q3 += ( qa * correctedGyroZ + qb * correctedGyroY - qc * correctedGyroX);

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		_gdu_ahrs->yaw = (atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)) * 57.2957795131f;// yaw
		_gdu_ahrs->pitch = (asin(-2 * q1 * q3 + 2 * q0 * q2)) * 57.2957795131f; // pitch
		_gdu_ahrs->roll = (atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)) * 57.2957795131f; // roll
		
		_gdu_ahrs->q0 = q0;
		_gdu_ahrs->q1 = q1;
		_gdu_ahrs->q2 = q2;
		_gdu_ahrs->q3 = q3;
	}
}
