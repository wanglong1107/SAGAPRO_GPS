#include "MTi_Xsens.h"
#include "globalVariables.h"

uint8_t MTi_XsensReadData[MTI_DATA_SIZE];

int MTi_XsensFresh = 0;



void XSENS::GetEulerangle(int index)
{
  int tmp;
  /*roll*/
  tmp = (MTi_XsensReadData[index+0] << 24) + (MTi_XsensReadData[index+1] << 16) +
        (MTi_XsensReadData[index+2] << 8 ) + (MTi_XsensReadData[index+3] << 0 );
  t_MTi_Xsens.roll = *(float*)(&tmp);
  
  /*pitch*/
  tmp = (MTi_XsensReadData[index+4] << 24) + (MTi_XsensReadData[index+5] << 16) +
        (MTi_XsensReadData[index+6] << 8 ) + (MTi_XsensReadData[index+7] << 0 );
  t_MTi_Xsens.pitch = -*(float*)(&tmp);
  
  /*yaw*/
  tmp = (MTi_XsensReadData[index+8] << 24) + (MTi_XsensReadData[index+9] << 16) +
        (MTi_XsensReadData[index+10]<< 8 ) + (MTi_XsensReadData[index+11]<< 0 );
  

	t_MTi_Xsens.Oyaw = -*(float *)(&tmp);
	
	float yawError = 0.0f;
	
	yawError = -*(float *)(&tmp) + yawOrigialAngler;
	
//	if( yawError > 180)
//	{
//		t_MTi_Xsens.yaw = yawError - 360;
//	}
//	else if(yawError < -180)
//	{
//		t_MTi_Xsens.yaw = yawError + 360;
//	}else
//	{
//	   t_MTi_Xsens.yaw = yawError;
//	}

	if(yawError < 0)
	{
		t_MTi_Xsens.yaw = yawError + 360;
	}else
	{
	   t_MTi_Xsens.yaw = yawError;
	}
	eulerAnglesDegree_XSENS[0]= t_MTi_Xsens.roll;
	eulerAnglesDegree_XSENS[1]= t_MTi_Xsens.pitch;
	eulerAnglesDegree_XSENS[2]= t_MTi_Xsens.yaw;
	
}
/*



*/
void XSENS::GetAcceleration(int index)
{
  int tmp;
  
  /*xAcc*/
  tmp = (MTi_XsensReadData[index+0] << 24) + (MTi_XsensReadData[index+1] << 16) +
        (MTi_XsensReadData[index+2] << 8 ) + (MTi_XsensReadData[index+3] << 0 );
  t_MTi_Xsens.xAcc = *(float*)(&tmp);
  
  /*yAcc*/
  tmp = (MTi_XsensReadData[index+4] << 24) + (MTi_XsensReadData[index+5] << 16) +
        (MTi_XsensReadData[index+6] << 8 ) + (MTi_XsensReadData[index+7] << 0 );
  t_MTi_Xsens.yAcc = -*(float*)(&tmp);
  
  /*zAcc*/
  tmp = (MTi_XsensReadData[index+8] << 24) + (MTi_XsensReadData[index+9] << 16) +
        (MTi_XsensReadData[index+10]<< 8 ) + (MTi_XsensReadData[index+11]<< 0 );
  t_MTi_Xsens.zAcc = -*(float*)(&tmp);
}
/*



*/
void XSENS::GetGyroscope(int index)
{
  int tmp;
  
  /*xAcc*/
  tmp = (MTi_XsensReadData[index+0] << 24) + (MTi_XsensReadData[index+1] << 16) +
        (MTi_XsensReadData[index+2] << 8 ) + (MTi_XsensReadData[index+3] << 0 );
  t_MTi_Xsens.xGyr = *(float*)(&tmp) * Rad2Deg;
  
  /*yAcc*/
  tmp = (MTi_XsensReadData[index+4] << 24) + (MTi_XsensReadData[index+5] << 16) +
        (MTi_XsensReadData[index+6] << 8 ) + (MTi_XsensReadData[index+7] << 0 );
  t_MTi_Xsens.yGyr = -*(float*)(&tmp) * Rad2Deg;

  /*zAcc*/
  tmp = (MTi_XsensReadData[index+8] << 24) + (MTi_XsensReadData[index+9] << 16) +
        (MTi_XsensReadData[index+10]<< 8 ) + (MTi_XsensReadData[index+11]<< 0 ); 
  t_MTi_Xsens.zGyr = -*(float*)(&tmp) * Rad2Deg;
}
/*




*/
void XSENS::GetXYZ(int index)
{
  int tmp;
  
  /*xAcc*/
  tmp = (MTi_XsensReadData[index+0] << 24) + (MTi_XsensReadData[index+1] << 16) +
        (MTi_XsensReadData[index+2] << 8 ) + (MTi_XsensReadData[index+3] << 0 );
  t_MTi_Xsens.lat = *(float*)(&tmp);
  
  /*yAcc*/
  tmp = (MTi_XsensReadData[index+4] << 24) + (MTi_XsensReadData[index+5] << 16) +
        (MTi_XsensReadData[index+6] << 8 ) + (MTi_XsensReadData[index+7] << 0 );
  t_MTi_Xsens.lon = *(float*)(&tmp);

  /*zAcc*/
  tmp = (MTi_XsensReadData[index+11] << 24) + (MTi_XsensReadData[index+12] << 16) +
        (MTi_XsensReadData[index+13]<< 8 ) + (MTi_XsensReadData[index+14]<< 0 ); 
  t_MTi_Xsens.altitude = -*(float*)(&tmp);
}
/*




*/

void XSENS::GetXYZvel(int index)
{
  int tmp;
  
  /*vx*/
  tmp = (MTi_XsensReadData[index+0] << 24) + (MTi_XsensReadData[index+1] << 16) +
        (MTi_XsensReadData[index+2] << 8 ) + (MTi_XsensReadData[index+3] << 0 );
  t_MTi_Xsens.vx = *(float*)(&tmp);
  
  /*vy*/
  tmp = (MTi_XsensReadData[index+4] << 24) + (MTi_XsensReadData[index+5] << 16) +
        (MTi_XsensReadData[index+6] << 8 ) + (MTi_XsensReadData[index+7] << 0 );
  t_MTi_Xsens.vy = -*(float*)(&tmp);

  /*vz*/
  tmp = (MTi_XsensReadData[index+8] << 24) + (MTi_XsensReadData[index+9] << 16) +
        (MTi_XsensReadData[index+10]<< 8 ) + (MTi_XsensReadData[index+11]<< 0 ); 
  t_MTi_Xsens.vz = -*(float*)(&tmp);
}
/*




*/

int XSENS::MTi_XsensUpdate(void)
{
	char crc;
	int i;
	uint16_t curCnt;
	

	crc = 0;
	for(i=1; i<MTI_DATA_SIZE; i++)
	{
		crc += MTi_XsensReadData[i];
	}
	if(crc) 
	{
		t_MTi_Xsens.crcErrorCnt++;
		return 0;     //Ð£ÑéÊ§°Ü
	}
	
	curCnt = ( MTi_XsensReadData[7] << 8) + (MTi_XsensReadData[8] );
	t_MTi_Xsens.lostPacketCounter = curCnt - t_MTi_Xsens.packetCounter;
	t_MTi_Xsens.packetCounter = curCnt;


	GetEulerangle(12);
//	GetAcceleration(27);
	GetGyroscope(27);
//	GetXYZ(46);
//	GetXYZvel(64);

	return 1;
	
	
}

int XSENS::XSENS_Parse( )
{	
	while( 0xFA != MTi_Queue.pBase[MTi_Queue.front])
	{
		if(!MTi_Queue.DeQueue(1))
		{
			return 0;
		}
	}
	
	if( !MTi_Queue.ReadQueue(MTi_XsensReadData, MTI_DATA_SIZE) )
	{
		return 0;
	}
	
 if(this->MTi_XsensUpdate())
 {
	 return 1;
 }
 else
 {
	 return 0;
 }
}
