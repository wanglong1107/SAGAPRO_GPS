#include "ublox_F9P.h"
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <hal_rtos.h>
#include "rtos_start.h"
#include "driver_init.h"

extern HAL_UART UartToFC;
extern uint8_t sendData[6][128];
extern uint8_t sendNmeaData[2][128];
extern uint8_t sendDataFlg[6];
extern uint8_t sendNmeaDataFlg[2];
extern uint32_t nmeaLength[2];
extern int DataSend(uint8_t i);
extern uint8_t D_RTK;
extern uint16_t numA;

void Ublox::ConfigOutputMessage(uint32_t _keyID, uint8_t _layers, uint8_t _rate)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBX_CFG_VALSET_CLASS;
	sendBuffer[cnt++] = UBX_CFG_VALSET_ID;	
	
	sendBuffer[cnt++] = 0x09;		// length lsb
	sendBuffer[cnt++] = 0x00;		// length msb
	
	sendBuffer[cnt++] = 0x00;		//version
	
	sendBuffer[cnt++] = _layers;	//layers
	
	sendBuffer[cnt++] = 0x00;		//reserved	
	sendBuffer[cnt++] = 0x00;		//reserved
	
	sendBuffer[cnt++] = _keyID ;
	sendBuffer[cnt++] = _keyID >> 8;
	sendBuffer[cnt++] = _keyID >> 16;
	sendBuffer[cnt++] = _keyID >> 24;
	
	sendBuffer[cnt++] = _rate;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		ubloxData.ubloxTxCK_A += sendBuffer[i];
		ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
	}
	
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_A;
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_B;
	dev->writeData( sendBuffer, cnt);
	
	delay_ms(UBLOX_WAIT_MS);
}

void Ublox::ConfigBuadrate(uint32_t _keyID, uint8_t _layers, uint32_t _buadRate)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBX_CFG_VALSET_CLASS;
	sendBuffer[cnt++] = UBX_CFG_VALSET_ID;	
	
	sendBuffer[cnt++] = 0x0c;		// length lsb
	sendBuffer[cnt++] = 0x00;		// length msb
	
	sendBuffer[cnt++] = 0x00;		//version
	
	sendBuffer[cnt++] = _layers;	//layers
	
	sendBuffer[cnt++] = 0x00;		//reserved
	sendBuffer[cnt++] = 0x00;		//reserved
	
	sendBuffer[cnt++] = _keyID ;
	sendBuffer[cnt++] = _keyID >> 8;
	sendBuffer[cnt++] = _keyID >> 16;
	sendBuffer[cnt++] = _keyID >> 24;
	
	sendBuffer[cnt++] = _buadRate;
	sendBuffer[cnt++] = _buadRate >> 8;
	sendBuffer[cnt++] = _buadRate >> 16;
	sendBuffer[cnt++] = _buadRate >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		ubloxData.ubloxTxCK_A += sendBuffer[i];
		ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
	}
	
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_A;
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_B;
	dev->writeData( sendBuffer, cnt);
	
	delay_ms(UBLOX_WAIT_MS);
}

void Ublox::ConfigSignal(uint32_t _gnssKeyID, uint8_t _layers, bool _enable)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBX_CFG_VALSET_CLASS;
	sendBuffer[cnt++] = UBX_CFG_VALSET_ID;	
	
	sendBuffer[cnt++] = 0x09;		// length lsb
	sendBuffer[cnt++] = 0x00;		// length msb
	
	sendBuffer[cnt++] = 0x00;		//version
	
	sendBuffer[cnt++] = _layers;	//layers
	
	sendBuffer[cnt++] = 0x00;		//reserved
	sendBuffer[cnt++] = 0x00;		//reserved
	
	sendBuffer[cnt++] = _gnssKeyID ;
	sendBuffer[cnt++] = _gnssKeyID >> 8;
	sendBuffer[cnt++] = _gnssKeyID >> 16;
	sendBuffer[cnt++] = _gnssKeyID >> 24;
	
	sendBuffer[cnt++] = _enable;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		ubloxData.ubloxTxCK_A += sendBuffer[i];
		ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
	}
	
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_A;
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_B;
	dev->writeData( sendBuffer, cnt);
	
	delay_ms(UBLOX_WAIT_MS);
}

void Ublox::ConfigRate(uint32_t _keyID, uint8_t _layers, uint32_t _ms)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBX_CFG_VALSET_CLASS;
	sendBuffer[cnt++] = UBX_CFG_VALSET_ID;	
	
	sendBuffer[cnt++] = 0x0c;		// length lsb
	sendBuffer[cnt++] = 0x00;		// length msb
	
	sendBuffer[cnt++] = 0x00;		//version
	
	sendBuffer[cnt++] = _layers;	//layers
	
	sendBuffer[cnt++] = 0x00;		//reserved
	sendBuffer[cnt++] = 0x00;		//reserved
	
	sendBuffer[cnt++] = _keyID ;
	sendBuffer[cnt++] = _keyID >> 8;
	sendBuffer[cnt++] = _keyID >> 16;
	sendBuffer[cnt++] = _keyID >> 24;
	
	sendBuffer[cnt++] = _ms;
	sendBuffer[cnt++] = _ms >> 8;
	sendBuffer[cnt++] = _ms >> 16;
	sendBuffer[cnt++] = _ms >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		ubloxData.ubloxTxCK_A += sendBuffer[i];
		ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
	}
	
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_A;
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_B;
	dev->writeData( sendBuffer, cnt);
	
	delay_ms(UBLOX_WAIT_MS);
}

void Ublox::ubloxInit(void)
{
    ubloxSendSetup();
}

void Ublox::ubloxSendSetup(void) 
{
	ConfigOutputMessage(UART1OUTPROT_NMEA, SAVE_TO_RAM, 1);	//1					//开启串口1的NMEA协议输出
	ConfigOutputMessage(UART1OUTPROT_UBX, SAVE_TO_RAM, 1);						//开启串口1的UBX协议输出
	ConfigOutputMessage(UART1OUTPROT_RTCM3X, SAVE_TO_RAM, 0);					//关闭串口1的RTCM协议输出
	
	ConfigOutputMessage(MSGOUT_NMEA_ID_DTM_UART1, SAVE_TO_RAM, 0);				//关闭串口1的DTM协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GBS_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GBS协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GGA_UART1, SAVE_TO_RAM, 50);				//开启串口1的GGA协议输出,10秒输出一次
	ConfigOutputMessage(MSGOUT_NMEA_ID_GLL_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GLL协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GNS_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GNS协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GRS_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GRS协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GSA_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GSA协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GST_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GST协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_GSV_UART1, SAVE_TO_RAM, 0);				//关闭串口1的GSV协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_RMC_UART1, SAVE_TO_RAM, 0);				//关闭串口1的RMC协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_VLW_UART1, SAVE_TO_RAM, 0);				//关闭串口1的VLW协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_VTG_UART1, SAVE_TO_RAM, 0);				//关闭串口1的VTG协议输出
	ConfigOutputMessage(MSGOUT_NMEA_ID_ZDA_UART1, SAVE_TO_RAM, 0);				//关闭串口1的ZDA协议输出
	
	
}

void Ublox::ConfigRtcmOutputMesseage(void) 
{
    ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1005_UART2, SAVE_TO_RAM, 0);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1077_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1087_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1097_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1127_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE1230_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE4072_0_UART2, SAVE_TO_RAM, 1);
	
	ConfigOutputMessage(MSGOUT_RTCM_3X_TYPE4072_1_UART2, SAVE_TO_RAM, 1);
}

void Ublox::ConfigGnssSystem(void) 
{
	ConfigSignal(SIGNAL_BDS_B2_ENA, SAVE_TO_RAM, 1);							//开启北斗B2，默认只开启了B1
}


int Ublox::receiveData(uint8_t *pData, int len)
{
	int ret = 0;
	uint8_t available = 0;
	uint8_t freeSpace = 0;
	
	freeSpace = rxBuffer.space();
	available = rxBuffer.available();
	
	if ( freeSpace == 0 )
	{
		rxBuffer.clear();
		return -1;
	}
	
	rxBuffer.write( pData, len);
	
    return 0;
}

//void Ublox::UbloxDataSend(void)
//{
//	uint8_t sendGpsData[128]={0xB5,0x62};
//	uint8_t len;
//	
//	if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_DOP_ID)
//	{
//		sendGpsData[2]=UBX_NAV_CLASS;
//		sendGpsData[3]=UBX_NAV_DOP_ID;
//		
//		UartToFC.writeData( sendGpsData, len);
//	}
//}

unsigned char Ublox::ubloxPublish(void)
{
    int ret = 0;	
		
	if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_DOP_ID)
	{
		gpsData.iTOW = ubloxData.payload.dop.iTOW;
		gpsData.gDOP = ubloxData.payload.dop.gDOP;
        gpsData.pDOP = ubloxData.payload.dop.pDOP;
        gpsData.hDOP = ubloxData.payload.dop.hDOP;
        gpsData.vDOP = ubloxData.payload.dop.vDOP;
        gpsData.tDOP = ubloxData.payload.dop.tDOP;
        gpsData.nDOP = ubloxData.payload.dop.nDOP;
        gpsData.eDOP = ubloxData.payload.dop.eDOP;
		
		gpsData.DopUpdateFlag = 1;
		ret = 1;	
    }
	/**
    else if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_POSLLH_ID)
	{
		gpsData.iTOW = ubloxData.payload.dop.iTOW;
		gpsData.lon = ubloxData.payload.posllh.lon;
		gpsData.lat = ubloxData.payload.posllh.lat;	
		gpsData.height = ubloxData.payload.posllh.hMSL;
		gpsData.hAcc = ubloxData.payload.posllh.hAcc;
		gpsData.vAcc = ubloxData.payload.posllh.vAcc;
	
		gpsData.PosllhUpdateFlag = 1;
		ret = 2;
    }
	*/
	else if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_PVT_ID)
	{
		gpsData.iTOW = ubloxData.payload.pvt.iTOW;
		gpsData.year = ubloxData.payload.pvt.year;
		gpsData.month = ubloxData.payload.pvt.month;
		gpsData.day = ubloxData.payload.pvt.day;
		gpsData.hour = ubloxData.payload.pvt.hour;
		gpsData.min = ubloxData.payload.pvt.min;
		gpsData.sec = ubloxData.payload.pvt.sec;
		gpsData.valid = ubloxData.payload.pvt.valid;
		gpsData.tAcc = ubloxData.payload.pvt.tAcc;
		gpsData.nano = ubloxData.payload.pvt.nano;
		gpsData.fixType = ubloxData.payload.pvt.fixType;
		gpsData.pvtFlags = ubloxData.payload.pvt.flags;
		gpsData.carrSoln = ubloxData.payload.pvt.flags>>6;
		gpsData.pvtFlags2 = ubloxData.payload.pvt.flags2;
		gpsData.numSV = ubloxData.payload.pvt.numSV;
		gpsData.lon = ubloxData.payload.pvt.lon;
		gpsData.lat = ubloxData.payload.pvt.lat;
		gpsData.height = ubloxData.payload.pvt.height;
		gpsData.hMSL = ubloxData.payload.pvt.hMSL;
		gpsData.hAcc = ubloxData.payload.pvt.hAcc;
		gpsData.vAcc = ubloxData.payload.pvt.vAcc;
		gpsData.velN = ubloxData.payload.pvt.velN;
		gpsData.velE = ubloxData.payload.pvt.velE;
		gpsData.velD = ubloxData.payload.pvt.velD;
		gpsData.gSpeed = ubloxData.payload.pvt.gSpeed;
		gpsData.headMot = ubloxData.payload.pvt.headMot;
		gpsData.sAcc = ubloxData.payload.pvt.sAcc;
		gpsData.headAcc = ubloxData.payload.pvt.headAcc;
		gpsData.pDOP = ubloxData.payload.pvt.pDOP;
//		gpsData.pvtReserved1 = ubloxData.payload.pvt.reserved1;
//		gpsData.pvtReserved2 = ubloxData.payload.pvt.reserved2;		
		gpsData.headVeh = ubloxData.payload.pvt.headVeh;
		gpsData.magDec = ubloxData.payload.pvt.magDec;
		gpsData.magAcc = ubloxData.payload.pvt.magAcc;
		
		gpsData.PvtUpdateFlag = 1;
		ret = 3;	
    }
	else if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_RELPOSNED_ID)
	{
		gpsData.relVersion = ubloxData.payload.relposned.version;
//		gpsData.relReserved1 = ubloxData.payload.relposned.reserved1;
		gpsData.refStationId = ubloxData.payload.relposned.refStationId;
		gpsData.iTOW = ubloxData.payload.relposned.iTOW;
		gpsData.relPosN = ubloxData.payload.relposned.relPosN;
		gpsData.relPosE = ubloxData.payload.relposned.relPosN;
		gpsData.relPosD = ubloxData.payload.relposned.relPosN;		
		gpsData.relPosLength = ubloxData.payload.relposned.relPosLength;
		gpsData.relPosHeading = ubloxData.payload.relposned.relPosHeading;
//		gpsData.relReserved2 = ubloxData.payload.relposned.reserved2;		
		gpsData.relPosHPN = ubloxData.payload.relposned.relPosHPN;	
		gpsData.relPosHPE = ubloxData.payload.relposned.relPosHPE;
		gpsData.relPosHPD = ubloxData.payload.relposned.relPosHPD;
		gpsData.relPosHPD = ubloxData.payload.relposned.relPosHPLength;	
		gpsData.accN = ubloxData.payload.relposned.accN;	
		gpsData.accE = ubloxData.payload.relposned.accE;
		gpsData.accD = ubloxData.payload.relposned.accD;
		gpsData.accLength = ubloxData.payload.relposned.accLength;
		gpsData.accHeading = ubloxData.payload.relposned.accHeading;
//		gpsData.relReserved3 = ubloxData.payload.relposned.reserved3;
		gpsData.relFlags = ubloxData.payload.relposned.flags;		
		gpsData.relGnssFixOK = ubloxData.payload.relposned.flags & (uint32_t)0x01;
		gpsData.relDiffSoln = (ubloxData.payload.relposned.flags >> 1) & (uint32_t)0x01;
		gpsData.relPosValid = (ubloxData.payload.relposned.flags >> 2) & (uint32_t)0x01;
		gpsData.relCarrSoln = (ubloxData.payload.relposned.flags >> 3) & (uint32_t)0x03;
		gpsData.relHeadingValid = ubloxData.payload.relposned.flags >> 8;

		gpsData.RelposnedUpdateFlag = 1;
        ret = 4;
    }
	/**
    else if (ubloxData.classId == UBX_NAV_CLASS && ubloxData.id == UBX_NAV_VELNED_ID)
	{
		gpsData.iTOW = ubloxData.payload.velned.iTOW;		
		gpsData.velN = ubloxData.payload.velned.velN;
		gpsData.velE = ubloxData.payload.velned.velE;
		gpsData.velD = ubloxData.payload.velned.velD;
		gpsData.speed = ubloxData.payload.velned.speed;
		gpsData.gSpeed = ubloxData.payload.velned.gSpeed;
		gpsData.heading = ubloxData.payload.velned.heading;
		gpsData.sAcc = ubloxData.payload.velned.sAcc;
		gpsData.cAcc = ubloxData.payload.velned.cAcc;
	
		gpsData.VelnedUpdateFlag = 1;
        ret = 5;
    }
	*/	
	else if(ubloxData.classId == UBX_RXM_CLASS && ubloxData.id == UBX_RXM_RTCM_ID)
	{
		gpsData.crcFailedFlags = ubloxData.payload.rtcm.flags;
		gpsData.subType = ubloxData.payload.rtcm.subType;
		gpsData.refStation = ubloxData.payload.rtcm.refStation;
		gpsData.msgType = ubloxData.payload.rtcm.msgType;
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.subType == 0) && (gpsData.msgType == 4072))
		{
			gpsData.count_4072_0++;
			
			if(gpsData.count_4072_0 > 20000)
			{
				gpsData.count_4072_0 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.subType == 1) && (gpsData.msgType == 4072))
		{
			gpsData.count_4072_1++;
			
			if(gpsData.count_4072_1 > 20000)
			{
				gpsData.count_4072_1 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1005))
		{
			gpsData.count_1005++;
			
			if(gpsData.count_1005 > 20000)
			{
				gpsData.count_1005 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1077))
		{
			gpsData.count_1077++;
			
			if(gpsData.count_1077 > 20000)
			{
				gpsData.count_1077 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1087))
		{
			gpsData.count_1087++;
			
			if(gpsData.count_1087 > 20000)
			{
				gpsData.count_1087 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1097))
		{
			gpsData.count_1097++;
			
			if(gpsData.count_1097 > 20000)
			{
				gpsData.count_1097 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1127))
		{
			gpsData.count_1127++;
			
			if(gpsData.count_1127 > 20000)
			{
				gpsData.count_1127 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1230))
		{
			gpsData.count_1230++;
			
			if(gpsData.count_1230 > 20000)
			{
				gpsData.count_1230 = 0;
			}
		}
		
		ret = 6;
		gpsData.RxmRtcmUpdateFlag = 1;
	}
	
    return ret;		
}

void Ublox::parseData(void)
{
	uint8_t ubloxAvailableBuffer[UBLOX_MAX_PAYLOAD];
	uint8_t headIndexBuffer[UBLOX_MAX_PAYLOAD/8];
	
	availableLength = 0;
	index = 0;
	headCnt = 0;
	payloadLength = 0;
	uint8_t CK_A=0, CK_B=0;
	
	availableLength = rxBuffer.available();
	if(availableLength > UBLOX_MAX_PAYLOAD - 2)
	{
		rxBuffer.clear();
		return;
	}
	availableLength = rxBuffer.peekbytes(ubloxAvailableBuffer, availableLength);
	if( availableLength>8 )
	{
		for(index=0; index<(availableLength-1); index++)
		{
			if( ((ubloxAvailableBuffer[index] == 0xb5) && (ubloxAvailableBuffer[index+1] == 0x62)) 
				|| ((ubloxAvailableBuffer[index] == 0x24) && (ubloxAvailableBuffer[index+3] == 0x47) && (ubloxAvailableBuffer[index+4] == 0x47) && (ubloxAvailableBuffer[index+5] == 0x41)) )
			{
				headIndexBuffer[headCnt] = index;
				headCnt++;
			}
		}
		
		if( headCnt == 0)
		{
			rxBuffer.advance(index);
			frameHeadError ++;
		}
		else if( headCnt==1 )
		{
			rxBuffer.advance(headIndexBuffer[0]);
		}
		else
		{
			rxBuffer.advance(headIndexBuffer[headCnt-1]);

			for(uint32_t k=0; k<(headCnt-1); k++)
			{
				CK_A = 0;
				CK_B = 0;
				for(int i=2; i < (headIndexBuffer[k+1]-headIndexBuffer[k]-2); i++)
				{
					CK_A = CK_A + ubloxAvailableBuffer[headIndexBuffer[k]+i];
					CK_B = CK_B + CK_A;
				}
				
				if(	((CK_A == ubloxAvailableBuffer[headIndexBuffer[k+1]-2]) && (CK_B == ubloxAvailableBuffer[headIndexBuffer[k+1]-1]))
				  ||((0x0D == ubloxAvailableBuffer[headIndexBuffer[k+1]-2]) && (0x0A == ubloxAvailableBuffer[headIndexBuffer[k+1]-1])) )
				{
					frameCheckOK ++;
					numA = 0x01;
				}
				else
				{	
					frameCheckError ++;
					continue;
				}
				ubloxData.classId = ubloxAvailableBuffer[headIndexBuffer[k]+2];
				ubloxData.id = ubloxAvailableBuffer[headIndexBuffer[k]+3];
				payloadLength = (ubloxAvailableBuffer[headIndexBuffer[k]+5]<<8) | (ubloxAvailableBuffer[headIndexBuffer[k]+4]);
				ubloxData.nmeaHead = ubloxAvailableBuffer[headIndexBuffer[k]];
				
				/////////////////////////////////////////////////////new add
				if(((*dev).hw == &USART_3) && (ubloxData.id == 0x07)) 
				{
					D_RTK = 1;
				}
				else if(ubloxData.nmeaHead == 0x24)
				{
					uint8_t lon = 0;
					for(uint32_t i=1; i<=128; i++)
					{
						if((ubloxAvailableBuffer[headIndexBuffer[k]+i-1] == 0x0D) && (ubloxAvailableBuffer[headIndexBuffer[k]+i] == 0x0A))
						{
							lon = i+1;
							break;
						}
					}
					
					for(uint8_t j=0;j<2;j++)
					{
						if(sendNmeaDataFlg[j]==0)
						{
							nmeaLength[j] = lon;
							memcpy( sendNmeaData[j], ubloxAvailableBuffer+headIndexBuffer[k], nmeaLength[j] );
							sendNmeaDataFlg[j]=1;
							break;
						}	
					}	
				}
				else
				{
					for(uint8_t i=0;i<6;i++)
					{
						if(sendDataFlg[i]==0)
						{
							memcpy( sendData[i], ubloxAvailableBuffer+headIndexBuffer[k], payloadLength+8 );
							sendDataFlg[i]=0x01;
							break;
						}
						
					}
				}
//				DataSend(0);
				////////////////////////////////////////////////////new add
				if(payloadLength > UBLOX_MAX_PAYLOAD)
				{
					rxBuffer.clear();
					continue;
				}
				memcpy( &ubloxData.payload, ubloxAvailableBuffer+headIndexBuffer[k]+6, payloadLength );
				ubloxPublish();
			}
		}
	}
}

void Ublox::SendDopData(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x04;
	SendGpsData[cnt++] = 18;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.iTOW;
	SendGpsData[cnt++] = gpsData.iTOW >> 8;
	SendGpsData[cnt++] = gpsData.iTOW >> 16;
	SendGpsData[cnt++] = gpsData.iTOW >> 24;
	
	SendGpsData[cnt++] = gpsData.gDOP;
	SendGpsData[cnt++] = gpsData.gDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.pDOP;
	SendGpsData[cnt++] = gpsData.pDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.tDOP;
	SendGpsData[cnt++] = gpsData.tDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.vDOP;
	SendGpsData[cnt++] = gpsData.vDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.hDOP;
	SendGpsData[cnt++] = gpsData.hDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.nDOP;
	SendGpsData[cnt++] = gpsData.nDOP >> 8;
	
	SendGpsData[cnt++] = gpsData.eDOP;
	SendGpsData[cnt++] = gpsData.eDOP >> 8;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	//UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendPosllhData(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x02;
	SendGpsData[cnt++] = 28;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.iTOW;
	SendGpsData[cnt++] = gpsData.iTOW >> 8;
	SendGpsData[cnt++] = gpsData.iTOW >> 16;
	SendGpsData[cnt++] = gpsData.iTOW >> 24;
	
	SendGpsData[cnt++] = gpsData.lon;
	SendGpsData[cnt++] = gpsData.lon >> 8;
	SendGpsData[cnt++] = gpsData.lon >> 16;
	SendGpsData[cnt++] = gpsData.lon >> 24;
	
	SendGpsData[cnt++] = gpsData.lat;
	SendGpsData[cnt++] = gpsData.lat >> 8;
	SendGpsData[cnt++] = gpsData.lat >> 16;
	SendGpsData[cnt++] = gpsData.lat >> 24;
	
	SendGpsData[cnt++] = gpsData.height;
	SendGpsData[cnt++] = gpsData.height >> 8;
	SendGpsData[cnt++] = gpsData.height >> 16;
	SendGpsData[cnt++] = gpsData.height >> 24;
	
	SendGpsData[cnt++] = gpsData.hMSL;
	SendGpsData[cnt++] = gpsData.hMSL >> 8;
	SendGpsData[cnt++] = gpsData.hMSL >> 16;
	SendGpsData[cnt++] = gpsData.hMSL >> 24;
	
	SendGpsData[cnt++] = gpsData.hAcc;
	SendGpsData[cnt++] = gpsData.hAcc >> 8;
	SendGpsData[cnt++] = gpsData.hAcc >> 16;
	SendGpsData[cnt++] = gpsData.hAcc >> 24;
	
	SendGpsData[cnt++] = gpsData.vAcc;
	SendGpsData[cnt++] = gpsData.vAcc >> 8;
	SendGpsData[cnt++] = gpsData.vAcc >> 16;
	SendGpsData[cnt++] = gpsData.vAcc >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendPvtData(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x07;
	SendGpsData[cnt++] = 92;
	SendGpsData[cnt++] = 0x00;

	SendGpsData[cnt++] = gpsData.iTOW;
	SendGpsData[cnt++] = gpsData.iTOW >> 8;
	SendGpsData[cnt++] = gpsData.iTOW >> 16;
	SendGpsData[cnt++] = gpsData.iTOW >> 24;
	SendGpsData[cnt++] = gpsData.year;
	SendGpsData[cnt++] = gpsData.year >> 8;
	SendGpsData[cnt++] = gpsData.month;
	SendGpsData[cnt++] = gpsData.day;
	SendGpsData[cnt++] = gpsData.hour;
	SendGpsData[cnt++] = gpsData.min;
	SendGpsData[cnt++] = gpsData.sec;
	SendGpsData[cnt++] = gpsData.valid;
	SendGpsData[cnt++] = gpsData.tAcc;
	SendGpsData[cnt++] = gpsData.tAcc >> 8;
	SendGpsData[cnt++] = gpsData.tAcc >> 16;
	SendGpsData[cnt++] = gpsData.tAcc >> 24;
	SendGpsData[cnt++] = gpsData.nano;
	SendGpsData[cnt++] = gpsData.nano >> 8;
	SendGpsData[cnt++] = gpsData.nano >> 16;
	SendGpsData[cnt++] = gpsData.nano >> 24;
	SendGpsData[cnt++] = gpsData.fixType;
	SendGpsData[cnt++] = gpsData.pvtFlags;
	SendGpsData[cnt++] = gpsData.pvtFlags2;
	SendGpsData[cnt++] = gpsData.numSV;
	SendGpsData[cnt++] = gpsData.lon;
	SendGpsData[cnt++] = gpsData.lon >> 8;
	SendGpsData[cnt++] = gpsData.lon >> 16;
	SendGpsData[cnt++] = gpsData.lon >> 24;
	SendGpsData[cnt++] = gpsData.lat;
	SendGpsData[cnt++] = gpsData.lat >> 8;
	SendGpsData[cnt++] = gpsData.lat >> 16;
	SendGpsData[cnt++] = gpsData.lat >> 24;
	SendGpsData[cnt++] = gpsData.height;
	SendGpsData[cnt++] = gpsData.height >> 8;
	SendGpsData[cnt++] = gpsData.height >> 16;
	SendGpsData[cnt++] = gpsData.height >> 24;
	SendGpsData[cnt++] = gpsData.hMSL;
	SendGpsData[cnt++] = gpsData.hMSL >> 8;
	SendGpsData[cnt++] = gpsData.hMSL >> 16;
	SendGpsData[cnt++] = gpsData.hMSL >> 24;
	SendGpsData[cnt++] = gpsData.hAcc;
	SendGpsData[cnt++] = gpsData.hAcc >> 8;
	SendGpsData[cnt++] = gpsData.hAcc >> 16;
	SendGpsData[cnt++] = gpsData.hAcc >> 24;
	SendGpsData[cnt++] = gpsData.vAcc;
	SendGpsData[cnt++] = gpsData.vAcc >> 8;
	SendGpsData[cnt++] = gpsData.vAcc >> 16;
	SendGpsData[cnt++] = gpsData.vAcc >> 24;
	SendGpsData[cnt++] = gpsData.velN;
	SendGpsData[cnt++] = gpsData.velN >> 8;
	SendGpsData[cnt++] = gpsData.velN >> 16;
	SendGpsData[cnt++] = gpsData.velN >> 24;
	SendGpsData[cnt++] = gpsData.velE;
	SendGpsData[cnt++] = gpsData.velE >> 8;
	SendGpsData[cnt++] = gpsData.velE >> 16;
	SendGpsData[cnt++] = gpsData.velE >> 24;
	SendGpsData[cnt++] = gpsData.velD;
	SendGpsData[cnt++] = gpsData.velD >> 8;
	SendGpsData[cnt++] = gpsData.velD >> 16;
	SendGpsData[cnt++] = gpsData.velD >> 24;
	SendGpsData[cnt++] = gpsData.gSpeed;
	SendGpsData[cnt++] = gpsData.gSpeed >> 8;
	SendGpsData[cnt++] = gpsData.gSpeed >> 16;
	SendGpsData[cnt++] = gpsData.gSpeed >> 24;
	SendGpsData[cnt++] = gpsData.headMot;
	SendGpsData[cnt++] = gpsData.headMot >> 8;
	SendGpsData[cnt++] = gpsData.headMot >> 16;
	SendGpsData[cnt++] = gpsData.headMot >> 24;
	SendGpsData[cnt++] = gpsData.sAcc;
	SendGpsData[cnt++] = gpsData.sAcc >> 8;
	SendGpsData[cnt++] = gpsData.sAcc >> 16;
	SendGpsData[cnt++] = gpsData.sAcc >> 24;
	SendGpsData[cnt++] = gpsData.headAcc;
	SendGpsData[cnt++] = gpsData.headAcc >> 8;
	SendGpsData[cnt++] = gpsData.headAcc >> 16;
	SendGpsData[cnt++] = gpsData.headAcc >> 24;
	SendGpsData[cnt++] = gpsData.pDOP;
	SendGpsData[cnt++] = gpsData.pDOP >> 8;
	cnt += 6;										//reserved1
	SendGpsData[cnt++] = gpsData.headVeh;
	SendGpsData[cnt++] = gpsData.headVeh >> 8;
	SendGpsData[cnt++] = gpsData.headVeh >> 16;
	SendGpsData[cnt++] = gpsData.headVeh >> 24;
	SendGpsData[cnt++] = gpsData.magDec;
	SendGpsData[cnt++] = gpsData.magDec >> 8;
	SendGpsData[cnt++] = gpsData.magAcc;
	SendGpsData[cnt++] = gpsData.magAcc >> 8;
		
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendRelposnedData(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x3c;
	SendGpsData[cnt++] = 64;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.relVersion;
	
	SendGpsData[cnt++] = 0;								//reserved1
	
	SendGpsData[cnt++] = gpsData.refStationId;
	SendGpsData[cnt++] = gpsData.refStationId >> 8;
	
	SendGpsData[cnt++] = gpsData.iTOW;
	SendGpsData[cnt++] = gpsData.iTOW >> 8;
	SendGpsData[cnt++] = gpsData.iTOW >> 16;
	SendGpsData[cnt++] = gpsData.iTOW >> 24;
	
	SendGpsData[cnt++] = gpsData.relPosN;
	SendGpsData[cnt++] = gpsData.relPosN >> 8;
	SendGpsData[cnt++] = gpsData.relPosN >> 16;
	SendGpsData[cnt++] = gpsData.relPosN >> 24;
	
	SendGpsData[cnt++] = gpsData.relPosE;
	SendGpsData[cnt++] = gpsData.relPosE >> 8;
	SendGpsData[cnt++] = gpsData.relPosE >> 16;
	SendGpsData[cnt++] = gpsData.relPosE >> 24;
	
	SendGpsData[cnt++] = gpsData.relPosD;
	SendGpsData[cnt++] = gpsData.relPosD >> 8;
	SendGpsData[cnt++] = gpsData.relPosD >> 16;
	SendGpsData[cnt++] = gpsData.relPosD >> 24;
	
	SendGpsData[cnt++] = gpsData.relPosLength;
	SendGpsData[cnt++] = gpsData.relPosLength >> 8;
	SendGpsData[cnt++] = gpsData.relPosLength >> 16;
	SendGpsData[cnt++] = gpsData.relPosLength >> 24;
	
	SendGpsData[cnt++] = gpsData.relPosHeading;
	SendGpsData[cnt++] = gpsData.relPosHeading >> 8;
	SendGpsData[cnt++] = gpsData.relPosHeading >> 16;
	SendGpsData[cnt++] = gpsData.relPosHeading >> 24;
	
	cnt+=4;												//reserved2
	
	SendGpsData[cnt++] = gpsData.relPosHPN;
	SendGpsData[cnt++] = gpsData.relPosHPE;
	SendGpsData[cnt++] = gpsData.relPosHPD;
	SendGpsData[cnt++] = gpsData.relPosHPLength;
	
	SendGpsData[cnt++] = gpsData.accN;
	SendGpsData[cnt++] = gpsData.accN >> 8;
	SendGpsData[cnt++] = gpsData.accN >> 16;
	SendGpsData[cnt++] = gpsData.accN >> 24;
	
	SendGpsData[cnt++] = gpsData.accE;
	SendGpsData[cnt++] = gpsData.accE >> 8;
	SendGpsData[cnt++] = gpsData.accE >> 16;
	SendGpsData[cnt++] = gpsData.accE >> 24;
	
	SendGpsData[cnt++] = gpsData.accD;
	SendGpsData[cnt++] = gpsData.accD >> 8;
	SendGpsData[cnt++] = gpsData.accD >> 16;
	SendGpsData[cnt++] = gpsData.accD >> 24;
	
	SendGpsData[cnt++] = gpsData.accLength;
	SendGpsData[cnt++] = gpsData.accLength >> 8;
	SendGpsData[cnt++] = gpsData.accLength >> 16;
	SendGpsData[cnt++] = gpsData.accLength >> 24;
	
	SendGpsData[cnt++] = gpsData.accHeading;
	SendGpsData[cnt++] = gpsData.accHeading >> 8;
	SendGpsData[cnt++] = gpsData.accHeading >> 16;
	SendGpsData[cnt++] = gpsData.accHeading >> 24;
	
	cnt+=4;											//reserved3
	
	SendGpsData[cnt++] = gpsData.relFlags;
	SendGpsData[cnt++] = gpsData.relFlags >> 8;
	SendGpsData[cnt++] = gpsData.relFlags >> 16;
	SendGpsData[cnt++] = gpsData.relFlags >> 24;
	
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendVelnedData(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x12;
	SendGpsData[cnt++] = 36;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.iTOW;
	SendGpsData[cnt++] = gpsData.iTOW >> 8;
	SendGpsData[cnt++] = gpsData.iTOW >> 16;
	SendGpsData[cnt++] = gpsData.iTOW >> 24;
	
	SendGpsData[cnt++] = gpsData.velN;
	SendGpsData[cnt++] = gpsData.velN >> 8;
	SendGpsData[cnt++] = gpsData.velN >> 16;
	SendGpsData[cnt++] = gpsData.velN >> 24;
	
	SendGpsData[cnt++] = gpsData.velE;
	SendGpsData[cnt++] = gpsData.velE >> 8;
	SendGpsData[cnt++] = gpsData.velE >> 16;
	SendGpsData[cnt++] = gpsData.velE >> 24;
	
	SendGpsData[cnt++] = gpsData.velD;
	SendGpsData[cnt++] = gpsData.velD >> 8;
	SendGpsData[cnt++] = gpsData.velD >> 16;
	SendGpsData[cnt++] = gpsData.velD >> 24;
	
	SendGpsData[cnt++] = gpsData.speed;
	SendGpsData[cnt++] = gpsData.speed >> 8;
	SendGpsData[cnt++] = gpsData.speed >> 16;
	SendGpsData[cnt++] = gpsData.speed >> 24;
	
	SendGpsData[cnt++] = gpsData.gSpeed;
	SendGpsData[cnt++] = gpsData.gSpeed >> 8;
	SendGpsData[cnt++] = gpsData.gSpeed >> 16;
	SendGpsData[cnt++] = gpsData.gSpeed >> 24;
	
	SendGpsData[cnt++] = gpsData.heading;
	SendGpsData[cnt++] = gpsData.heading >> 8;
	SendGpsData[cnt++] = gpsData.heading >> 16;
	SendGpsData[cnt++] = gpsData.heading >> 24;
	
	SendGpsData[cnt++] = gpsData.sAcc;
	SendGpsData[cnt++] = gpsData.sAcc >> 8;
	SendGpsData[cnt++] = gpsData.sAcc >> 16;
	SendGpsData[cnt++] = gpsData.sAcc >> 24;
	
	SendGpsData[cnt++] = gpsData.cAcc;
	SendGpsData[cnt++] = gpsData.cAcc >> 8;
	SendGpsData[cnt++] = gpsData.cAcc >> 16;
	SendGpsData[cnt++] = gpsData.cAcc >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendRxmRtcmData1(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB1;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x02;
	SendGpsData[cnt++] = 0x32;
	SendGpsData[cnt++] = 32;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.count_1005;
	SendGpsData[cnt++] = gpsData.count_1005 >> 8;
	SendGpsData[cnt++] = gpsData.count_1005 >> 16;
	SendGpsData[cnt++] = gpsData.count_1005 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1077;
	SendGpsData[cnt++] = gpsData.count_1077 >> 8;
	SendGpsData[cnt++] = gpsData.count_1077 >> 16;
	SendGpsData[cnt++] = gpsData.count_1077 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1087;
	SendGpsData[cnt++] = gpsData.count_1087 >> 8;
	SendGpsData[cnt++] = gpsData.count_1087 >> 16;
	SendGpsData[cnt++] = gpsData.count_1087 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1097;
	SendGpsData[cnt++] = gpsData.count_1097 >> 8;
	SendGpsData[cnt++] = gpsData.count_1097 >> 16;
	SendGpsData[cnt++] = gpsData.count_1097 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1127;
	SendGpsData[cnt++] = gpsData.count_1127 >> 8;
	SendGpsData[cnt++] = gpsData.count_1127 >> 16;
	SendGpsData[cnt++] = gpsData.count_1127 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1230;
	SendGpsData[cnt++] = gpsData.count_1230 >> 8;
	SendGpsData[cnt++] = gpsData.count_1230 >> 16;
	SendGpsData[cnt++] = gpsData.count_1230 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_4072_0;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 8;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 16;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_4072_1;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 8;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 16;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}

void Ublox::SendRxmRtcmData2(void)
{
	uint8_t CK_A = 0, CK_B = 0, cnt = 0;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB2;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x02;
	SendGpsData[cnt++] = 0x32;
	SendGpsData[cnt++] = 32;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.count_1005;
	SendGpsData[cnt++] = gpsData.count_1005 >> 8;
	SendGpsData[cnt++] = gpsData.count_1005 >> 16;
	SendGpsData[cnt++] = gpsData.count_1005 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1077;
	SendGpsData[cnt++] = gpsData.count_1077 >> 8;
	SendGpsData[cnt++] = gpsData.count_1077 >> 16;
	SendGpsData[cnt++] = gpsData.count_1077 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1087;
	SendGpsData[cnt++] = gpsData.count_1087 >> 8;
	SendGpsData[cnt++] = gpsData.count_1087 >> 16;
	SendGpsData[cnt++] = gpsData.count_1087 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1097;
	SendGpsData[cnt++] = gpsData.count_1097 >> 8;
	SendGpsData[cnt++] = gpsData.count_1097 >> 16;
	SendGpsData[cnt++] = gpsData.count_1097 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1127;
	SendGpsData[cnt++] = gpsData.count_1127 >> 8;
	SendGpsData[cnt++] = gpsData.count_1127 >> 16;
	SendGpsData[cnt++] = gpsData.count_1127 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_1230;
	SendGpsData[cnt++] = gpsData.count_1230 >> 8;
	SendGpsData[cnt++] = gpsData.count_1230 >> 16;
	SendGpsData[cnt++] = gpsData.count_1230 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_4072_0;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 8;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 16;
	SendGpsData[cnt++] = gpsData.count_4072_0 >> 24;
	
	SendGpsData[cnt++] = gpsData.count_4072_1;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 8;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 16;
	SendGpsData[cnt++] = gpsData.count_4072_1 >> 24;
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	UartToFC.writeData( SendGpsData, cnt);
	delay_ms(10);
}