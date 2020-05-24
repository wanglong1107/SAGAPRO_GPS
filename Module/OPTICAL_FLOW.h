#ifndef _OPTICAL_FLOW
#define _OPTICAL_FLOW

#include <string.h>

#include "USART_Driver.h"
#include "AP_Math.h"
#define FLOW_SIZE  35

class OPTICAL_FLOW: public USART_Device
{
public:
	Vector3ui  GyroRawFromFlow;       // unit 1000*rad/s
	uint16_t   DistancRaweFromFlow;   // unit 1000*m;
	uint16_t   FlowXRaw;              // unit 1*unknown
	uint16_t   FlowYRaw;              // unit 1*unknown
	uint16_t   FuseFlowX;             // unit 1000*unknown
	uint16_t   FuseFlowY;             // unit 1000*unknown
	uint16_t   TempFromFlow;          // unit 100*.C(degree)
	Vector3ui  IntegralGyroFormFlow;  // unit 10*mrad
	uint16_t   IntegralFlowX;         // unit 10*mrad
	uint16_t   IntegralFlowY;         // unit 10*mrad
	uint16_t   ReliabilityOfFlow;     //  0~255


	Vector3f   fGyroRawFromFlow_rad;       // unit rad/s
	float      fDistancRaweFromFlow_m;   // unit m;
	float      fFlowXRaw;              // unit unknown
	float      fFlowYRaw;              // unit unknown
	float      fFuseFlowX;             // unit unknown
	float      fFuseFlowY;             // unit *unknown
	float      fTempFromFlow;          // unit (degree)
	Vector3f   fIntegralGyroFormFlow_mrad;  // unit mrad
	float      fIntegralFlowX_mrad;         // unit mrad
	float      fIntegralFlowY_mrad;         // unit mrad

	Usart* USART;

  OPTICAL_FLOW( Usart* _usart )
	{
//		memset(&GPS_Time, 0, sizeof(GPS_Time));
//		memset(&GPS_Position, 0, sizeof(GPS_Position));
		
		USART = _usart;
	}
	
	int OpticalFlowParse(void);
	int OpticalFlowDataParse(void);
	void copyOpticalFlowValueToEKF(void);

};

#endif //_OPTICAL_FLOW
