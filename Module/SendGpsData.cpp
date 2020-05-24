#include "SendGpsData.h"
#include <stdlib.h>
#include <string.h>

extern HAL_UART adsp;

void SendGpsData::DopData(void)
{
	uint8_t CK_A, CK_B, cnt;
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
	adsp.writeData( SendGpsData, cnt);
}

void SendGpsData::PosllhData(void)
{
	uint8_t CK_A, CK_B, cnt;
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
	adsp.writeData( SendGpsData, cnt);
}

void SendGpsData::PvtData(void)
{
	uint8_t CK_A, CK_B, cnt;
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
	
	SendGpsData[cnt++] = 0;
	
	SendGpsData[cnt++] = gpsData.tAcc;
	SendGpsData[cnt++] = gpsData.tAcc >> 8;
	SendGpsData[cnt++] = gpsData.tAcc >> 16;
	SendGpsData[cnt++] = gpsData.tAcc >> 24;
	
	cnt+=4;
	
	SendGpsData[cnt++] = gpsData.fixType;
	SendGpsData[cnt++] = gpsData.carrSoln;
	SendGpsData[cnt++] = 0;
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
	
	cnt+=4;
	
	SendGpsData[cnt++] = gpsData.sAcc;
	SendGpsData[cnt++] = gpsData.sAcc >> 8;
	SendGpsData[cnt++] = gpsData.sAcc >> 16;
	SendGpsData[cnt++] = gpsData.sAcc >> 24;
	
	cnt+=20;
	
	
	for(uint8_t i=2; i<cnt; i++)
	{
		CK_A += SendGpsData[i];
		CK_B += CK_A;
	}
	
	SendGpsData[cnt++] = CK_A;
	SendGpsData[cnt++] = CK_B;
	adsp.writeData( SendGpsData, cnt);
}

void SendGpsData::RelposnedData(void)
{
	uint8_t CK_A, CK_B, cnt;
	uint8_t SendGpsData[120];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[cnt++] = 0xB5;
	SendGpsData[cnt++] = 0x62;
	SendGpsData[cnt++] = 0x01;
	SendGpsData[cnt++] = 0x3c;
	SendGpsData[cnt++] = 64;
	SendGpsData[cnt++] = 0x00;
	
	SendGpsData[cnt++] = gpsData.relVersion;
	
	SendGpsData[cnt++] = 0;
	
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
	
	cnt+=4;
	
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
	
	cnt+=4;
	
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
	adsp.writeData( SendGpsData, cnt);
}

void SendGpsData::VelnedData(void)
{
	uint8_t CK_A, CK_B, cnt;
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
	adsp.writeData( SendGpsData, cnt);
}


