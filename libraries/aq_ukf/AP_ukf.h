/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ?2011-2014  Bill Nesbitt
*/

#ifndef AP_ukf_h
#define AP_ukf_h

//#include "aq.h"
#include <aq_math/srcdkf.h>
#include "sensors_ukf.h"
#include "parameters_ukf.h"

#define UKF_LOG_SIZE		(17*sizeof(float))
#define UKF_LOG_BUF_SIZE	(UKF_LOG_SIZE*40)
//#define UKF_LOG_FNAME		"UKF"		// comment out to disable logging

#define SIM_S                   17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   12//16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		40

#define UKF_STATE_VELN		0
#define UKF_STATE_VELE		1
#define UKF_STATE_VELD		2
#define UKF_STATE_POSN		3
#define UKF_STATE_POSE		4
#define UKF_STATE_POSD		5
#define UKF_STATE_ACC_BIAS_X	6
#define UKF_STATE_ACC_BIAS_Y	7
#define UKF_STATE_ACC_BIAS_Z	8
#define UKF_STATE_GYO_BIAS_X	9
#define UKF_STATE_GYO_BIAS_Y	10
#define UKF_STATE_GYO_BIAS_Z	11
#define UKF_STATE_Q1		12
#define UKF_STATE_Q2		13
#define UKF_STATE_Q3		14
#define UKF_STATE_Q4		15
#define UKF_STATE_PRES_ALT	16

#define UKF_V_NOISE_ACC_BIAS_X	0
#define UKF_V_NOISE_ACC_BIAS_Y	1
#define UKF_V_NOISE_ACC_BIAS_Z	2
#define UKF_V_NOISE_GYO_BIAS_X	3
#define UKF_V_NOISE_GYO_BIAS_Y	4
#define UKF_V_NOISE_GYO_BIAS_Z	5
#define UKF_V_NOISE_RATE_X	6
#define UKF_V_NOISE_RATE_Y	7
#define UKF_V_NOISE_RATE_Z	8
#define UKF_V_NOISE_VELN	9
#define UKF_V_NOISE_VELE	10
#define UKF_V_NOISE_VELD	11

#define UKF_VELN		navUkfData.x[UKF_STATE_VELN]
#define UKF_VELE		navUkfData.x[UKF_STATE_VELE]
#define UKF_VELD		navUkfData.x[UKF_STATE_VELD]
#define UKF_POSN		navUkfData.x[UKF_STATE_POSN]
#define UKF_POSE		navUkfData.x[UKF_STATE_POSE]
#define UKF_POSD		navUkfData.x[UKF_STATE_POSD]
#define UKF_ACC_BIAS_X		navUkfData.x[UKF_STATE_ACC_BIAS_X]
#define UKF_ACC_BIAS_Y		navUkfData.x[UKF_STATE_ACC_BIAS_Y]
#define UKF_ACC_BIAS_Z		navUkfData.x[UKF_STATE_ACC_BIAS_Z]
#define UKF_GYO_BIAS_X		navUkfData.x[UKF_STATE_GYO_BIAS_X]
#define UKF_GYO_BIAS_Y		navUkfData.x[UKF_STATE_GYO_BIAS_Y]
#define UKF_GYO_BIAS_Z		navUkfData.x[UKF_STATE_GYO_BIAS_Z]
#define UKF_Q1			navUkfData.x[UKF_STATE_Q1]
#define UKF_Q2			navUkfData.x[UKF_STATE_Q2]
#define UKF_Q3			navUkfData.x[UKF_STATE_Q3]
#define UKF_Q4			navUkfData.x[UKF_STATE_Q4]
#define UKF_PRES_ALT		navUkfData.x[UKF_STATE_PRES_ALT]

#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif

#define UKF_HIST		40
#define UKF_P0			101325.0f			    // standard static pressure at sea level

#define UKF_FLOW_ROT		-90.0f	//-90.0f			    // optical flow mounting rotation in degrees
#define UKF_FOCAL_LENGTH	16.0f				    // 16mm
#define UKF_FOCAL_PX		(UKF_FOCAL_LENGTH / (4.0f * 6.0f) * 1000.0f)   // pixel size: 6um, binning 4 enabled


extern float accNED_N;
extern float accNED_E;
extern float accNED_D;
extern float gps_x;
extern float gps_y;

extern float VELL[3];
extern float ACCP[3];

enum sensorNoiseConstants {
    UKF_NOISE_GPS_POS_N,
    UKF_NOISE_GPS_POS_M_N,
    UKF_NOISE_GPS_ALT_N,
    UKF_NOISE_GPS_ALT_M_N,
    UKF_NOISE_GPS_VEL_N,
    UKF_NOISE_GPS_VEL_M_N,
    UKF_NOISE_GPS_VD_N,
    UKF_NOISE_GPS_VD_M_N,
    UKF_NOISE_ALT_N,
    UKF_NOISE_ACC_N,
    UKF_NOISE_DIST_N,
    UKF_NOISE_MAG_N,
    UKF_NOISE_POS_DELAY,
    UKF_NOISE_VEL_DELAY,
    UKF_NOISE_CONSTANTS_ENUM_END
};

typedef struct {
    double holdLat, holdLon;
    double r1, r2;

    srcdkf_t *kf;
    float *x;			// states
	
	float presAltOffset;

    float v0a[3];
    float v0m[3];
    float posN[UKF_HIST];
    float posE[UKF_HIST];
    float posD[UKF_HIST];
    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];
    float sensorNoise[UKF_NOISE_CONSTANTS_ENUM_END];
    float yaw, pitch, roll;
    float yawCos, yawSin;

    float flowSumX, flowSumY, flowSumAlt;
    float flowVelX, flowVelY;
    float flowPosN, flowPosE;
    float flowQuality;
    float flowAlt;
    float flowRotCos, flowRotSin;
    uint32_t flowCount, flowAltCount;
    int32_t flowSumQuality;

    int16_t logPointer;
    int8_t logHandle;

    int8_t navHistIndex;
	
	uint64_t ap_outerTimestep;

    uint8_t flowInit;
    volatile uint8_t flowLock;
} navUkfStruct_t __attribute__((aligned));

extern float delta_posx_test;

class AP_UKF
{
public:
	void navPressureAdjust(float altitude);
	float navUkfPresToAlt(float pressure);
	void navUkfSetGlobalPositionTarget(double lat, double lon);
	void navUkfSetHereAsPositionTarget(void);
	void navUkfRotateVectorByQuat(float *vr, float *v, float *q);
	void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
	void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);

	void navUkfFinish(void);
	void navUkfInertialUpdate(void);
	void simDoPresUpdate(float pres);
	void simDoAccUpdate(float accX, float accY, float accZ);
	void simDoMagUpdate(float magX, float magY, float magZ);
	void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc);
	void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc);
	void navUkfFlowUpdate(void);
	void navUkfOpticalFlow(int16_t x, int16_t y, uint8_t quality, float ground);
	void navUkfZeroRate(float zRate, int axis);
	void navUkfZeroPos(void);
	void navUkfResetPosition(float deltaN, float deltaE, float deltaD);
	void navUkfZeroVel(void);
	void navUkfResetVels(void);
	void navUkfResetBias(void);
	void navUkfResetQuat(void);
	void navUkfInitState(void);
	void navUkfInitVariance(void);
	void navUkfInit(void);
	
	navUkfStruct_t navUkfData;
};

#endif //AP_ukf_h
