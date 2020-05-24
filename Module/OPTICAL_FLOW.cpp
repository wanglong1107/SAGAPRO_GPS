#include "OPTICAL_FLOW.h"
#include "globalVariables.h"
#include <math.h>
#include "drv_hrt.h"

uint8_t FlowParseBuffer[FLOW_SIZE];



int OPTICAL_FLOW::OpticalFlowParse( )
{	
	while( 0xAA != OpticalFlow_Queue.pBase[OpticalFlow_Queue.front])
	{
		if(!OpticalFlow_Queue.DeQueue(1))
		{
			return 0;
		}
	}
	
	if( !OpticalFlow_Queue.ReadQueue(FlowParseBuffer, FLOW_SIZE) )
	{
		return 0;
	}
	
 if(this->OpticalFlowDataParse())
 {
	 return 1;
 }
 else
 {
	 return 0;
 }
	
}


int OPTICAL_FLOW::OpticalFlowDataParse()
{
	uint8_t CheckSum=0;
	if( (0xAA==FlowParseBuffer[0]) & (0xAA==FlowParseBuffer[1]) )
	{
		/*calculate checksum*/
		for(int i=0;i < (FLOW_SIZE-1);i++)
		{
			CheckSum += FlowParseBuffer[i];
		}
		if(CheckSum == FlowParseBuffer[FLOW_SIZE - 1])
		{
			GyroRawFromFlow.x      = ((uint16_t) FlowParseBuffer[4] << 8 ) | (uint16_t) FlowParseBuffer[5];
			GyroRawFromFlow.y      = ((uint16_t) FlowParseBuffer[6] << 8 ) | (uint16_t) FlowParseBuffer[7];
			GyroRawFromFlow.z      = ((uint16_t) FlowParseBuffer[8] << 8 ) | (uint16_t) FlowParseBuffer[9];
			fGyroRawFromFlow_rad.x = ((float) GyroRawFromFlow.x) * 0.001f;
			fGyroRawFromFlow_rad.y = ((float) GyroRawFromFlow.y) * 0.001f;
			fGyroRawFromFlow_rad.z = ((float) GyroRawFromFlow.z) * 0.001f;
			
			DistancRaweFromFlow    = ((uint16_t) FlowParseBuffer[10] << 8 ) | (uint16_t) FlowParseBuffer[11];
			fDistancRaweFromFlow_m = ((float) DistancRaweFromFlow)*0.001f;
			
			FlowXRaw   =  ((uint16_t) FlowParseBuffer[12] << 8 ) | (uint16_t) FlowParseBuffer[13];
			FlowYRaw   =  ((uint16_t) FlowParseBuffer[14] << 8 ) | (uint16_t) FlowParseBuffer[15];
			fFlowXRaw  = (float) FlowXRaw;
			fFlowYRaw  = (float) FlowYRaw;
			
			FuseFlowX  = ((uint16_t) FlowParseBuffer[16] << 8 ) | (uint16_t) FlowParseBuffer[17];
			FuseFlowY  = ((uint16_t) FlowParseBuffer[18] << 8 ) | (uint16_t) FlowParseBuffer[19];
			fFuseFlowX = ((float) FuseFlowX) * 0.001f;
			fFuseFlowY = ((float) FuseFlowY) * 0.001f;
			
			TempFromFlow  = ((uint16_t) FlowParseBuffer[20] << 8 ) | (uint16_t) FlowParseBuffer[21];
			fTempFromFlow = (float) TempFromFlow;
			
			IntegralGyroFormFlow.x     = ((uint16_t) FlowParseBuffer[22] << 8 ) | (uint16_t) FlowParseBuffer[23];
			IntegralGyroFormFlow.x     = ((uint16_t) FlowParseBuffer[24] << 8 ) | (uint16_t) FlowParseBuffer[25];
			IntegralGyroFormFlow.x     = ((uint16_t) FlowParseBuffer[26] << 8 ) | (uint16_t) FlowParseBuffer[27];
			fIntegralGyroFormFlow_mrad.x = ((float) IntegralGyroFormFlow.x) * 0.1f;
			fIntegralGyroFormFlow_mrad.y = ((float) IntegralGyroFormFlow.y) * 0.1f;
			fIntegralGyroFormFlow_mrad.z = ((float) IntegralGyroFormFlow.z) * 0.1f;
			
			IntegralFlowX       = ((uint16_t) FlowParseBuffer[28] << 8 ) | (uint16_t) FlowParseBuffer[29];
			IntegralFlowY       = ((uint16_t) FlowParseBuffer[30] << 8 ) | (uint16_t) FlowParseBuffer[31];
			fIntegralFlowX_mrad = ((float) IntegralFlowX) * 0.1f;
			fIntegralFlowX_mrad = ((float) IntegralFlowY) * 0.1f;
			
			ReliabilityOfFlow   = ((uint16_t) FlowParseBuffer[32] << 8 ) | (uint16_t) FlowParseBuffer[33];	
			
			copyOpticalFlowValueToEKF();	
      return 1;			
		}
		else 
		{
			return 0;
		}
	}
	else 
	{
		return 0;
	}
	
}

void OPTICAL_FLOW::copyOpticalFlowValueToEKF()
{
//	uint64_t timestamp = hrt_absolute_time();
//	
//	gpsPosition.timestamp_time = timestamp;
//	gpsPosition.time_utc_usec = 0;  ////////////////////////////////////
//	gpsPosition.timestamp_position = timestamp;
//	gpsPosition.lat = GPS_Position.Ublox_GPS_lat;
//	gpsPosition.lon = GPS_Position.Ublox_GPS_lon;
//	gpsPosition.alt = GPS_Position.Ublox_GPS_hMSL;
//	gpsPosition.eph = (float)GPS_Position.Ublox_GPS_hAcc * 0.1f;
//	gpsPosition.epv = (float)GPS_Position.Ublox_GPS_vAcc * 0.1f;
//	gpsPosition.timestamp_variance = timestamp;
//	gpsPosition.s_variance_m_s = 5.0f;
//	
//	gpsPosition.timestamp_velocity = timestamp;
//	gpsPosition.vel_m_s = (float)GPS_Position.Ublox_GPS_gSpeed * 1e-3f;
//	gpsPosition.vel_n_m_s = (float)GPS_Position.Ublox_GPS_velN * 1e-3f;
//	gpsPosition.vel_e_m_s = (float)GPS_Position.Ublox_GPS_velE * 1e-3f;
//	gpsPosition.vel_d_m_s = (float)GPS_Position.Ublox_GPS_velD * 1e-3f;
//	gpsPosition.vel_ned_valid = true;
//	gpsPosition.cog_rad = GPS_Position.Ublox_GPS_headMot * DEG_TO_RAD * 1e-2f;  
//	gpsPosition.fix_type = GPS_Position.Ublox_GPS_fixType;
//	gpsPosition.satellites_used = GPS_Position.Ublox_GPS_numSV; 
}


