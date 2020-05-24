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

#include "ukf_run.h"

#include "nav_ukf.h"
#include "alt_ukf.h"
#include "sensors_ukf.h"
#include "alt_ukf.h"
#include <hal_drivers/drv_hrt.h>
#include <string.h>
#include <os.h>

volatile float  bg,sm,md;
runStruct_t runData; //__attribute__((section(".ccm")));
uint8_t ukfinitializeOK = 0;

uint8_t headInitFlag = 0;
float initialHead = 0.0f;

uint64_t ti = 0;
uint64_t to = 0;

void ukfRunTask(void  *p_arg)
{
	OS_ERR  os_err;

	int ret = -1;
    (void)p_arg;
	
    uint32_t axis = 0;
    uint32_t loops = 0;

#ifdef _ALT_MULTI
    float volatile alt_delta = 0;

    runData.alt_a=0;
    runData.alt_b=0;
#endif
	navUkfInit();
	altUkfInit();
	runInit();

    while (1) 
	{
        // soft start GPS accuracy
        runData.accMask *= 0.99f;//  0.999f

		runData.flyingTime = hrt_absolute_time();
		if(runData.flyingTime>5000000) //5s
		{
			runData.flyingFlag = 1;
		}
		
		if (runData.ukfInitFlag) 
		{
			runData.ukfInitFlag = 0;
			if (!(runData.flyingFlag))
			{
				runInitHistory();
				navUkfInitState();
			}
			ukfinitializeOK = 1;
		}
		
		ti = hrt_absolute_time();
		navUkfInertialUpdate();
		to = hrt_absolute_time() - ti;
        // record history for acc & mag & pressure readings for smoothing purposes
        // acc
        runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];
        runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
        runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];

        runData.accHist[0][runData.sensorHistIndex] = ap_accel.x;
        runData.accHist[1][runData.sensorHistIndex] = ap_accel.y;
        runData.accHist[2][runData.sensorHistIndex] = ap_accel.z;

        runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];
        runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];
        runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];

        // mag
        runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
        runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
        runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];

        runData.magHist[0][runData.sensorHistIndex] = ap_mag.x;
        runData.magHist[1][runData.sensorHistIndex] = ap_mag.y;
        runData.magHist[2][runData.sensorHistIndex] = ap_mag.z;

        runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];
        runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];
        runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];

        // pressure
        runData.sumPres -= runData.presHist[runData.sensorHistIndex];
        runData.presHist[runData.sensorHistIndex] = ap_baro.pressure;
        runData.sumPres += runData.presHist[runData.sensorHistIndex];

        runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;

		if (!((loops+1) % 5))
		{
//			simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST));
            //acc has do 25hz lpf in driver
			simDoAccUpdate(ap_accel.x, ap_accel.y, ap_accel.z);
		}
		if(ap_baro.updateFlag)
		{
			ap_baro.updateFlag = 0;
			simDoPresUpdate(ap_baro.pressure);
		}
#if 1
        
		if(ap_mag.updateFlag)
		{
			
			ap_mag.updateFlag = 0;
//			simDoMagUpdate(ap_mag.x, ap_mag.y, ap_mag.z);
			
			simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[2]*(1.0f / (float)RUN_SENSOR_HIST));
			runData.magYaw = atan2(ap_mag.y, ap_mag.x)*RAD_TO_DEG;
//			simDoQuatUpdate(navUkfData.roll,navUkfData.pitch,runData.magYaw);
		}
#endif 
        // optical flow update
        if (navUkfData.flowCount >= 10 && !navUkfData.flowLock)
		{
            navUkfFlowUpdate();
        }
        // only accept GPS updates if there is no optical flow
        if ( (gnssData.pos_updateFlag==1) && gnssData.hAcc < NAV_MIN_GPS_ACC && gnssData.vAcc < NAV_MIN_GPS_VACC && gnssData.tDOP != 0.0f)
        {  
			gnssData.pos_updateFlag = 0;
			navUkfGpsPosUpdate(gnssData.lastPosUpdate, gnssData.lat, gnssData.lon, gnssData.height, gnssData.hAcc + runData.accMask, gnssData.vAcc + runData.accMask);
           

            // refine static sea level pressure based on better GPS altitude fixes
            if ((gnssData.hAcc < runData.bestHacc) && (gnssData.hAcc < NAV_MIN_GPS_ACC) &&  (gnssData.vAcc < NAV_MIN_GPS_VACC)  && (gnssData.fix_type >= 3))
            {   
				navPressureAdjust(gnssData.height);
                runData.bestHacc = gnssData.hAcc;
            }
        }
        if ( (gnssData.vel_updateFlag==1) && navUkfData.flowQuality == 0.0f && gnssData.sAcc < (NAV_MIN_GPS_ACC*2.0f)/*NAV_MIN_GPS_ACC/2*/ && gnssData.tDOP != 0.0f)
		{
			gnssData.vel_updateFlag = 0;
            navUkfGpsVelUpdate(gnssData.lastVelUpdate, gnssData.velN, gnssData.velE, gnssData.velD, gnssData.sAcc + runData.accMask);
           
        }
        // observe zero position
        if (!((loops+4) % 20) && (gnssData.hAcc >= NAV_MIN_GPS_ACC || gnssData.tDOP == 0.0f))
		{
//            navUkfZeroPos();
        }
        // observe zero velocity
        if (!((loops+10) % 20) && (gnssData.sAcc >= (NAV_MIN_GPS_ACC*0.75f)/*NAV_MIN_GPS_ACC/2*/ || gnssData.tDOP == 0.0f))
		{
//            navUkfZeroVel();
        }
		
        // observe that the rates are exactly 0 if not flying or moving
        if (!(runData.flyingFlag))
		{
			float stdX, stdY, stdZ;

            arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
            arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
            arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

            if ((stdX + stdY + stdZ) < (0.05f*2))
			{
                if (!((axis + 0) % 3))
                    navUkfZeroRate(ap_gyro.x, 0);
                else if (!((axis + 1) % 3))
                    navUkfZeroRate(ap_gyro.y, 1);
                else
                    navUkfZeroRate(ap_gyro.z, 2);
                axis++;
            }
        }

        navUkfFinish();
		if(headInitFlag==0)
		{
			headInitFlag = 1;
			initialHead = navUkfData.yaw;
		}
        altUkfProcess(ap_baro.pressure);

        // determine which altitude estimate to use
        if (gnssData.hAcc > 0.0 )
		{

#ifdef _ALT_MULTI
            alt_delta = alt_delta*0.999f + (ALT_POS-UKF_ALTITUDE)*0.001f;//alt_delta = 0.0f;

            //md = md*0.99f + (ALT_POS-UKF_ALTITUDE)*0.01f;//alt_delta = 0.0f;
            //bg = bg*0.999f + (ALT_POS-UKF_ALTITUDE)*0.001f;//alt_delta = 0.0f;
            //sm = alt_delta;
            runData.alt_a = UKF_ALTITUDE + alt_delta;
            runData.alt_b = ALT_POS - alt_delta;

            runData.altPos = &ALT_POS;
#else
            runData.altPos = &ALT_POS;
#endif
            runData.altVel = &ALT_VEL;


        }
        else 
		{
#ifdef _ALT_MULTI

            alt_delta = alt_delta*0.999f + (ALT_POS-UKF_ALTITUDE)*0.001f;
            //md = md*0.99f + (ALT_POS-UKF_ALTITUDE)*0.01f;//alt_delta = 0.0f;
            //bg = bg*0.999f + (ALT_POS-UKF_ALTITUDE)*0.001f;//alt_delta = 0.0f;
            //sm = alt_delta;
            runData.alt_a = UKF_ALTITUDE + alt_delta;
            runData.alt_b = ALT_POS - alt_delta;

            runData.altPos = &runData.alt_a;
            //runData.altPos = &UKF_ALTITUDE;
#else
            runData.altPos = &UKF_ALTITUDE;
#endif

            runData.altVel = &UKF_VELD;
        }
		
        loops++;
		OSTimeDlyHMSM( 0u, 0u, 0u, 5u, OS_OPT_TIME_HMSM_STRICT, &os_err);
    }
}


void runInitUkf(void)
{
    runData.ukfInitFlag = 1;
	runData.flyingFlag = 0;
}

void runInitHistory(void) {
    float acc[3], mag[3];
    float pres;
    int i;

    acc[0] = ap_accel.x;
    acc[1] = ap_accel.y;
    acc[2] = ap_accel.z;

    mag[0] = ap_mag.x;
    mag[1] = ap_mag.y;
    mag[2] = ap_mag.z;

    pres = ap_baro.pressure;

    // initialize sensor history
    for (i = 0; i < 3; ++i)
	{
        runData.sumAcc[i] = 0.0f;
        runData.sumMag[i] = 0.0f;
    }
    runData.sumPres = 0.0f;
    for (i = 0; i < RUN_SENSOR_HIST; i++)
	{
        runData.accHist[0][i] = acc[0];
        runData.accHist[1][i] = acc[1];
        runData.accHist[2][i] = acc[2];
        runData.magHist[0][i] = mag[0];
        runData.magHist[1][i] = mag[1];
        runData.magHist[2][i] = mag[2];
        runData.presHist[i] = pres;

        runData.sumAcc[0] += acc[0];
        runData.sumAcc[1] += acc[1];
        runData.sumAcc[2] += acc[2];
        runData.sumMag[0] += mag[0];
        runData.sumMag[1] += mag[1];
        runData.sumMag[2] += mag[2];
        runData.sumPres += pres;
    }

    runData.sensorHistIndex = 0;

    runData.bestHacc = 1000.0f;
    runData.accMask = 1000.0f;

    // use altUkf altitude & vertical velocity estimates to start with
    runData.altPos = &ALT_POS;
    runData.altVel = &ALT_VEL;

}

void runInit(void)
{
    memset((void *)&runData, 0, sizeof(runData));

	runInitUkf();
	runInitHistory();

}
