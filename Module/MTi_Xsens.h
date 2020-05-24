#ifndef _MTI_XSENS_H_
#define _MTI_XSENS_H_

#include "board.h"
#include "UART_Driver.h"
#include <string.h>

//弧度和角度转换
#define Deg2Rad (0.01745329251994329576923690768489f)
#define Rad2Deg (57.295779513082320876798154814105f)

//MTI30:50,MTI10:41, MTI-700 55
#define MTI_DATA_SIZE 40

typedef struct _ahrs
{
  uint16_t crcErrorCnt;
  
  uint16_t packetCounter;
  uint16_t lostPacketCounter;

  // angular
  float roll;
  float pitch;
  float yaw;
  float Oyaw;
  // angular acceloration
  float xAcc;
  float yAcc;
  float zAcc;
  
  // angular velocity
  float xGyr;
  float yGyr;
  float zGyr;
	
	// altitude lat lon
  float altitude;
  float lat;
  float lon;
	
	//  velocity
  float vx;
  float vy;
  float vz;
  
}_ahrs;

class XSENS: public UART_Device
{
public:
	_ahrs t_MTi_Xsens;
	Uart* UART;

XSENS(Uart* _uart):UART_Device()
	{
		UART = _uart;
	}

	void GetEulerangle(int index);
  void GetAcceleration(int index);
  void GetGyroscope(int index);
	void GetXYZ(int index);
	void GetXYZvel(int index);
  int MTi_XsensUpdate(void);
  int XSENS_Parse(void);
};


#endif //_MTI_XSENS_H_
