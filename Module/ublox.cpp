#include "ublox.h"
#include <stdlib.h>
#include <string.h>

//extern HAL_UART adsp;
extern uint8_t useRtkHeadFlag;
extern double rtkHead;

void Ublox::ubloxEnableMessage(unsigned char c, unsigned char i, unsigned char rate)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_MSG;		// RTATE	
	
	sendBuffer[cnt++] = 0x03;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	sendBuffer[cnt++] = c;		// class id
	sendBuffer[cnt++] = i;		// id
	sendBuffer[cnt++] = rate;	// rate
	
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

void Ublox::ubloxSetRate(unsigned short int ms)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_RATE;	// RATE	
	
	sendBuffer[cnt++] = 0x06;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	sendBuffer[cnt++] = (uint8_t)(ms);		// rate
	sendBuffer[cnt++] = (uint8_t)(ms>>8);
	sendBuffer[cnt++] = 0x01;	// cycles
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x01;	// timeRef    0 == UTC, 1 == GPS time
	sendBuffer[cnt++] = 0x00;
	
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

void Ublox::ubloxSetMode(void)
{
	uint8_t sendBuffer[80];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_NAV5;		// NAV5	
	
	sendBuffer[cnt++] = 0x24;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	sendBuffer[cnt++] = 0xff;	// mask LSB (fixMode, dyn)
	sendBuffer[cnt++] = 0xff;	// mask MSB (reserved)
	sendBuffer[cnt++] = 0x00;	// /**< 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
	sendBuffer[cnt++] = 0x03;	// fixMode (2 == 3D only)

	// the rest of the packet is ignored due to the above mask
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x10;
	sendBuffer[cnt++] = 0x27;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x0A;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0xFA;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0xFA;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x64;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x5E;
	sendBuffer[cnt++] = 0x01;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x3C;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
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

void Ublox::ubloxSetTimepulse(void)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_TP;		// TP	
	
	sendBuffer[cnt++] = 0x14;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	sendBuffer[cnt++] = (uint8_t)(1000000);		// interval (us)
	sendBuffer[cnt++] = (uint8_t)(1000000>>8);
	sendBuffer[cnt++] = (uint8_t)(1000000>>16);
	sendBuffer[cnt++] = (uint8_t)(1000000>>24);
	sendBuffer[cnt++] = (uint8_t)(100000);			// length (us)
	sendBuffer[cnt++] = (uint8_t)(100000>>8);
	sendBuffer[cnt++] = (uint8_t)(100000>>16);
	sendBuffer[cnt++] = (uint8_t)(100000>>24);

#ifdef GPS_LATENCY
	sendBuffer[cnt++] = 0x00;			// config setting (0 == off)
#else
	sendBuffer[cnt++] = 0x01;			// config setting (1 == +polarity)
#endif

	sendBuffer[cnt++] = 0x01;                 // alignment reference time (GPS)
	sendBuffer[cnt++] = 0x00;                 // bitmask (syncmode 0)
	sendBuffer[cnt++] = 0x00;                 // reserved
	sendBuffer[cnt++] = 0x00;                 // antenna delay
	sendBuffer[cnt++] = 0x00;                 // rf group delay
	sendBuffer[cnt++] = 0x00;                 // user delay
	
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

void Ublox::ubloxSetSBAS(uint8_t enable) 
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	// second bit of mode field is diffCorr
	enable = (enable > 0);
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_SBAS;		// SBAS
	
	sendBuffer[cnt++] = 0x08;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	sendBuffer[cnt++] = enable;		// enable
	sendBuffer[cnt++] = 0x03;		// mode
	sendBuffer[cnt++] = 0x03;		// # SBAS tracking channels
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = UBLOX_SBAS_AUTO;	// ANY SBAS system
	
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

void Ublox::ubloxSetNMEA(void) 
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_NMEA;		// NMEA
	
	sendBuffer[cnt++] = 0x14;	
	sendBuffer[cnt++] = 0x00;	
	
	sendBuffer[cnt++] = 0x00;		
	sendBuffer[cnt++] = 0x41;		
	sendBuffer[cnt++] = 0x00;		
	sendBuffer[cnt++] = 0x0A;				//High percision mode
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x01;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	
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

void Ublox::ubloxInitGpsUsart1(uint32_t _baud)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_PRT;		// PRT
	
	sendBuffer[cnt++] = 0x14;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb

	sendBuffer[cnt++] = 0x01;	// portId UART1
	sendBuffer[cnt++] = 0x00;	// reserved
	sendBuffer[cnt++] = 0x00;	// txRead
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0xd0;	// mode 8n1
	sendBuffer[cnt++] = 0x08;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = (uint8_t)(_baud);    // baudRate
	sendBuffer[cnt++] = (uint8_t)(_baud>>8);
	sendBuffer[cnt++] = (uint8_t)(_baud>>16);
	sendBuffer[cnt++] = (uint8_t)(_baud>>24);
	sendBuffer[cnt++] = 0x03;	// inProtoMask UBX+NMEA
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x01;	// outProtoMask UBX
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;	// flags
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;	// reserved
	sendBuffer[cnt++] = 0x00;
	
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

void Ublox::ubloxInitGpsUsart2(uint32_t _baud)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_CFG_CLASS;	// CFG
	sendBuffer[cnt++] = UBLOX_CFG_PRT;		// PRT
	
	sendBuffer[cnt++] = 0x14;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb

	sendBuffer[cnt++] = 0x02;	// portId UART2
	sendBuffer[cnt++] = 0x00;	// reserved
	sendBuffer[cnt++] = 0x00;	// txRead
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0xd0;	// mode 8n1
	sendBuffer[cnt++] = 0x08;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = (uint8_t)(_baud);    // baudRate
	sendBuffer[cnt++] = (uint8_t)(_baud>>8);
	sendBuffer[cnt++] = (uint8_t)(_baud>>16);
	sendBuffer[cnt++] = (uint8_t)(_baud>>24);
	sendBuffer[cnt++] = 0x20;	// inProtoMask RTCM3
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x20;	// outProtoMask RTCM3
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;	// flags
	sendBuffer[cnt++] = 0x00;
	sendBuffer[cnt++] = 0x00;	// reserved
	sendBuffer[cnt++] = 0x00;
	
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

void Ublox::ubloxPollVersion(void)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_MON_CLASS;	// MON
	sendBuffer[cnt++] = UBLOX_MON_VER;		// VER
	
	sendBuffer[cnt++] = 0x00;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
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

void Ublox::ubloxPollVersionNoDelay(void)
{
	uint8_t sendBuffer[50];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	ubloxData.ubloxTxCK_A = 0;
	ubloxData.ubloxTxCK_B = 0;
	
	sendBuffer[cnt++] = 0xb5;
	sendBuffer[cnt++] = 0x62;
	
	sendBuffer[cnt++] = UBLOX_MON_CLASS;	// MON
	sendBuffer[cnt++] = UBLOX_MON_VER;		// VER
	
	sendBuffer[cnt++] = 0x00;	// length lsb
	sendBuffer[cnt++] = 0x00;	// length msb
	
	for(uint8_t i=2; i<cnt; i++)
	{
		ubloxData.ubloxTxCK_A += sendBuffer[i];
		ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
	}
	
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_A;
	sendBuffer[cnt++] = ubloxData.ubloxTxCK_B;
	dev->writeData( sendBuffer, cnt);
}

void Ublox::ubloxVersionSpecific(int ver) {

//    AQ_PRINTF("gps hw num: %u \n", ubloxData.hwVer);

    if (ver > 7) {
        // 5Hz for ver 8 using multiple GNSS
        ubloxSetRate((uint16_t)200);
    }
    else if (ver > 6) {
        // 10Hz for ver 7
        ubloxSetRate((uint16_t)100);
        // SBAS screwed up on v7 modules w/ v1 firmware
        ubloxSetSBAS(0);	// disable SBAS
    }
    else {
        // 5Hz=200
        ubloxSetRate((uint16_t)200);//ubloxSetRate((uint16_t)200); 
    }
}

void Ublox::ubloxSendSetup(void) 
{
//    yield(UBLOX_WAIT_MS);
	
	// disable NMEA Protocol
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_DTM_ID, 0);
//	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GBQ_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GBS_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GGA_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GLL_ID, 0);
//	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GLQ_ID, 0);
//	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GNQ_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GNS_ID, 0);
//	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GPQ_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GRS_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GSA_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GST_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_GSV_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_RMC_ID, 0);
//	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_TXT_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_VLW_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_VTG_ID, 0);
	ubloxEnableMessage(NMEA_STD_CLASSID, NMEA_ZDA_ID, 0);
	
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_POSITION_DATA_ID, 0);
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_UTM_POSITION_ID, 0);
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_SATELLITE_DATA_ID, 0);
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_TIME_OF_DAY_ID, 0);
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_EKF_STATUS_ID, 0);
	ubloxEnableMessage(NMEA_PUBX_CLASSID, NMEA_GPS_ONLY_ON_EKF_PRODUCTS_ID, 0);
	
	
//    ubloxSetTimepulse();
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_VELNED, 0);   // NAV VALNED
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_POSLLH, 1);   // NAV POSLLH
    ubloxEnableMessage(UBLOX_TIM_CLASS, UBLOX_TIM_TP, 0);       // TIM TP
	
	ubloxEnableMessage(UBLOX_RXM_CLASS, UBLOX_RXM_RTCM, 0);     // RXM RTCM
	ubloxEnableMessage(UBLOX_NAV_CLASS, UBX_ID_NAV_PVT, 1);     // for fix_type and sta num
	ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_RELPOSNED, 0);
//    ubloxEnableMessage(UBLOX_TIM_CLASS, UBX_ID_NAV_SOL, 0);     // for fix_type and sta num
    
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_DOP, 5);      // NAV DOP
//    ubloxEnableMessage(UBLOX_AID_CLASS, UBLOX_AID_REQ, 0);      // AID REQ
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_TIMEUTC, 0);  // NAV TIMEUTC
#ifdef GPS_DO_RTK
//    ubloxEnableMessage(UBLOX_RXM_CLASS, UBLOX_RXM_RAW, 0);      // RXM RAW
//    ubloxEnableMessage(UBLOX_RXM_CLASS, UBLOX_RXM_SFRB, 0);     // RXM SFRB
	
#endif
#ifdef GPS_DEBUG
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_SVINFO, 1);   // NAV SVINFO
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_SBAS, 1);     // NAV SBAS
    ubloxEnableMessage(UBLOX_MON_CLASS, UBLOX_MON_HW, 1);       // MON HW
#endif

    
//    ubloxSetMode();                                             // 3D, airborne
//    ubloxPollVersion();
//    ubloxVersionSpecific(ubloxData.hwVer);
}


void Ublox::configF9p1(void)
{
	uint8_t enableRtcm1077[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0xCE,0x02,0x91,0x20,0x01,0x22,0x63};
	uint8_t enableRtcm1087[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0xD3,0x02,0x91,0x20,0x01,0x27,0x7C};
	uint8_t enableRtcm1097[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0x1A,0x03,0x91,0x20,0x01,0x6F,0xE3};
	uint8_t enableRtcm1127[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0xD8,0x02,0x91,0x20,0x01,0x2C,0x95};
	uint8_t enableRtcm1230[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0x05,0x03,0x91,0x20,0x01,0x5A,0x7A};
	uint8_t enableRtcm4072_0[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0x00,0x03,0x91,0x20,0x01,0x55,0x61};
	uint8_t enableRtcm4072_1[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x00,0x07,0x00,0x00,0x83,0x03,0x91,0x20,0x01,0xD8,0xF0};
	uint8_t setGNSS[] = {0xB5,0x62,0x06,0x8A,0x4F,0x00,0x01,0x07,0x00,0x00,0x01,0x00,0x31,0x10,0x01,0x03,0x00,0x31,0x10,0x01,0x07,0x00,0x31,0x10,0x01,0x0A,0x00,0x31,0x10,0x01,0x0D,0x00,0x31,0x10,0x01,0x0E,0x00,0x31,0x10,0x01,0x12,0x00,0x31,0x10,0x01,0x15,0x00,0x31,0x10,0x01,0x18,0x00,0x31,0x10,0x01,0x1A,0x00,0x31,0x10,0x01,0x1F,0x00,0x31,0x10,0x01,0x21,0x00,0x31,0x10,0x01,0x22,0x00,0x31,0x10,0x01,0x24,0x00,0x31,0x10,0x01,0x25,0x00,0x31,0x10,0x01,0xF9,0xBF };

	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
		
	dev->writeData( enableRtcm1077, sizeof(enableRtcm1077) );
	delay_ms(2);
	
	dev->writeData( enableRtcm1087, sizeof(enableRtcm1087) );
	delay_ms(2);

	dev->writeData( enableRtcm1097, sizeof(enableRtcm1097) );
	delay_ms(2);
	
	dev->writeData( enableRtcm1127, sizeof(enableRtcm1127) );
	delay_ms(2);
	
	dev->writeData( enableRtcm1230, sizeof(enableRtcm1230) );
	delay_ms(2);
	
	dev->writeData( enableRtcm4072_0, sizeof(enableRtcm4072_0) );
	delay_ms(2);
	
	dev->writeData( enableRtcm4072_1, sizeof(enableRtcm4072_1) );	
	delay_ms(2);						
}
void Ublox::configF9p2(void)
{
	uint8_t setGNSS[] = {0xB5,0x62,0x06,0x8A,0x4F,0x00,0x01,0x07,0x00,0x00,0x01,0x00,0x31,0x10,0x01,0x03,0x00,0x31,0x10,0x01,0x07,0x00,0x31,0x10,0x01,0x0A,0x00,0x31,0x10,0x01,0x0D,0x00,0x31,0x10,0x01,0x0E,0x00,0x31,0x10,0x01,0x12,0x00,0x31,0x10,0x01,0x15,0x00,0x31,0x10,0x01,0x18,0x00,0x31,0x10,0x01,0x1A,0x00,0x31,0x10,0x01,0x1F,0x00,0x31,0x10,0x01,0x21,0x00,0x31,0x10,0x01,0x22,0x00,0x31,0x10,0x01,0x24,0x00,0x31,0x10,0x01,0x25,0x00,0x31,0x10,0x01,0xF9,0xBF };
	
	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
		
	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
		
	dev->writeData( setGNSS, sizeof(setGNSS) );
	delay_ms(20);
}

void Ublox::ubloxInit(void)
{
    //memset((void *)&ubloxData, 0, sizeof(ubloxData));
    ubloxData.state = UBLOX_WAIT_SYNC1;
	ubloxData.hwVer = UBLOX_HWVERSION;
    ubloxSendSetup();
}

unsigned char Ublox::ubloxPublish(void)
{
    unsigned char ret = 0;
    
    // don't allow preemption
   
//    int priority = 1;
//	rt_thread_control(&gpsTask,RT_THREAD_CTRL_CHANGE_PRIORITY,&priority);
	 
    if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_NAV_POSLLH)
	{
        // work around uBlox's inability to give new data on each report sometimes
        if (ubloxData.lastLat != ubloxData.payload.posllh.lat || ubloxData.lastLon != ubloxData.payload.posllh.lon)
		{
			ubloxData.lastLat = ubloxData.payload.posllh.lat;
			ubloxData.lastLon = ubloxData.payload.posllh.lon;

			gpsData.delta_lat = ubloxData.payload.posllh.lat - gpsData.lat_int;
			gpsData.delta_lon = ubloxData.payload.posllh.lon - gpsData.lon_int;
			gpsData.lat_int = ubloxData.payload.posllh.lat;
			gpsData.lon_int = ubloxData.payload.posllh.lon;
			gpsData.height_int = ubloxData.payload.posllh.height;

			gpsData.iTOW = ubloxData.payload.posllh.iTOW;
			gpsData.lat = (double)ubloxData.payload.posllh.lat * (double)1e-7;
			gpsData.lon = (double)ubloxData.payload.posllh.lon * (double)1e-7;
			gpsData.height = ubloxData.payload.posllh.hMSL * 0.001f;    // mm => m
			gpsData.hAcc = ubloxData.payload.posllh.hAcc * 0.001f;      // mm => m
			gpsData.vAcc = ubloxData.payload.posllh.vAcc * 0.001f;      // mm => m

#ifdef GPS_LATENCY
            gpsData.lastPosUpdate = hrt_absolute_time() - GPS_LATENCY;
#else
            gpsData.lastPosUpdate = gpsData.lastTimepulse + (ubloxData.payload.posllh.iTOW - gpsData.TPtowMS) * 1000;
#endif
            // position update
            ret = 1;
			if((gpsData.delta_lat < 500) && (gpsData.delta_lon < 300) )
			{
				gpsData.pos_updateFlag = 1;
			}
        }
    }
    else if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_NAV_VELNED)
	{
		gpsData.iTOW = ubloxData.payload.valned.iTOW;

		gpsData.delta_velN = ubloxData.payload.valned.velN * 0.01f - gpsData.velN;
		gpsData.delta_velE = ubloxData.payload.valned.velE * 0.01f - gpsData.velE;
		gpsData.delta_velD = ubloxData.payload.valned.velD * 0.01f - gpsData.velD;
		
		gpsData.velN_int = ubloxData.payload.valned.velN;           // cm
		gpsData.velE_int = ubloxData.payload.valned.velE;           // cm
		gpsData.velD_int = ubloxData.payload.valned.velD;           // cm

		gpsData.velN = ubloxData.payload.valned.velN * 0.01f;           // cm => m
		gpsData.velE = ubloxData.payload.valned.velE * 0.01f;           // cm => m
		gpsData.velD = ubloxData.payload.valned.velD * 0.01f;           // cm => m
		gpsData.speed = ubloxData.payload.valned.gSpeed * 0.01f;        // cm/s => m/s
		gpsData.heading = ubloxData.payload.valned.heading * 1e-5f;
		gpsData.sAcc = ubloxData.payload.valned.sAcc * 0.01f;           // cm/s => m/s
		gpsData.cAcc = ubloxData.payload.valned.cAcc * 1e-5f;

#ifdef GPS_LATENCY
        gpsData.lastVelUpdate = hrt_absolute_time() - GPS_LATENCY;
#else
        gpsData.lastVelUpdate = gpsData.lastTimepulse + (ubloxData.payload.valned.iTOW - gpsData.TPtowMS) * 1000;
#endif

        // velocity update
        ret = 2;
		if((gpsData.delta_velN < 2.0f) && (gpsData.delta_velE < 2.0f) && (gpsData.delta_velD < 2.0f))
		{
			gpsData.vel_updateFlag = 1;
		}
    }
    else if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBX_ID_NAV_PVT)
	{
		//Check if position fix flag is good
		if ((ubloxData.payload.pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1)
		{
			gpsData.fix_type = ubloxData.payload.pvt.fixType;
			gpsData.carrSoln = ubloxData.payload.pvt.flags>>6;
		}
		else
		{
			gpsData.fix_type = 0;
				
		}
		
//		if ((ubloxData.payload.pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) == 1)
//		{
//			gpsData.carrSoln = ubloxData.payload.pvt.flags>>6;
//		}
//		else
//		{
//			gpsData.carrSoln = 0;
//		}
		

		gpsData.satellites_used	= ubloxData.payload.pvt.numSV;

    }
//    else if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBX_ID_NAV_SOL)
//	{	
//		//已经没有这一条协议
//		gpsData.fix_type = ubloxData.payload.sol.gpsFix;			
//		gpsData.satellites_used	= ubloxData.payload.sol.numSV;
//    }
    else if (ubloxData.classId == UBLOX_TIM_CLASS && ubloxData.id == UBLOX_TIM_TP)
	{
        gpsData.lastReceivedTPtowMS = ubloxData.payload.tp.towMS;
    }
    else if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_NAV_DOP)
	{
        gpsData.pDOP = ubloxData.payload.dop.pDOP * 0.01f;
        gpsData.hDOP = ubloxData.payload.dop.hDOP * 0.01f;
        gpsData.vDOP = ubloxData.payload.dop.vDOP * 0.01f;
        gpsData.tDOP = ubloxData.payload.dop.tDOP * 0.01f;
        gpsData.nDOP = ubloxData.payload.dop.nDOP * 0.01f;
        gpsData.eDOP = ubloxData.payload.dop.eDOP * 0.01f;
        gpsData.gDOP = ubloxData.payload.dop.gDOP * 0.01f;
    }
	else if(ubloxData.classId == UBLOX_RXM_CLASS && ubloxData.id == UBLOX_RXM_RTCM)
	{
		gpsData.crcFailedFlags = ubloxData.payload.rtcm.crcFailedFlags;
		gpsData.subType = ubloxData.payload.rtcm.subType;
		gpsData.refStation = ubloxData.payload.rtcm.refStation;
		gpsData.msgType = ubloxData.payload.rtcm.msgType;
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.subType == 0) && (gpsData.msgType == 4072))
		{
			gpsData.count_4072_0++;
			
			if(gpsData.count_4072_0 > 2000000)
			{
				gpsData.count_4072_0 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.subType == 1) && (gpsData.msgType == 4072))
		{
			gpsData.count_4072_1++;
			
			if(gpsData.count_4072_1 > 2000000)
			{
				gpsData.count_4072_1 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1005))
		{
			gpsData.count_1005++;
			
			if(gpsData.count_1005 > 2000000)
			{
				gpsData.count_1005 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1077))
		{
			gpsData.count_1077++;
			
			if(gpsData.count_1077 > 2000000)
			{
				gpsData.count_1077 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1087))
		{
			gpsData.count_1087++;
			
			if(gpsData.count_1087 > 2000000)
			{
				gpsData.count_1087 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1097))
		{
			gpsData.count_1097++;
			
			if(gpsData.count_1097 > 2000000)
			{
				gpsData.count_1097 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1127))
		{
			gpsData.count_1127++;
			
			if(gpsData.count_1127 > 2000000)
			{
				gpsData.count_1127 = 0;
			}
		}
		
		if((gpsData.crcFailedFlags == 0) && (gpsData.msgType == 1230))
		{
			gpsData.count_1230++;
			
			if(gpsData.count_1230 > 2000000)
			{
				gpsData.count_1230 = 0;
			}
		}
	}
	// end of high priority section
//    priority=GPS_PRIORITY;
//	rt_thread_control(&gpsTask,RT_THREAD_CTRL_CHANGE_PRIORITY,&priority);	

    if (ubloxData.classId == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_NAV_TIMEUTC && (ubloxData.payload.timeutc.valid & 0b100))
	{
        // if setting the RTC succeeds, disable the TIMEUTC message
//        if (rtcSetDataTime(ubloxData.payload.timeutc.year, ubloxData.payload.timeutc.month, ubloxData.payload.timeutc.day,
//                ubloxData.payload.timeutc.hour, ubloxData.payload.timeutc.min, ubloxData.payload.timeutc.sec))
        ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_TIMEUTC, 1);
		gpsData.year = ubloxData.payload.timeutc.year;
		gpsData.month = ubloxData.payload.timeutc.month;
		gpsData.day = ubloxData.payload.timeutc.day;
		gpsData.hour = ubloxData.payload.timeutc.hour;
		gpsData.min = ubloxData.payload.timeutc.min;
		gpsData.sec = ubloxData.payload.timeutc.sec;
		
    }
    else if (ubloxData.classId == UBLOX_MON_CLASS && ubloxData.id == UBLOX_MON_VER)
	{
        ubloxData.hwVer = atoi(ubloxData.payload.ver.hwVersion) / 10000;
        ubloxVersionSpecific(ubloxData.hwVer); 
//        AQ_NOTICE("gps hw get \n");
    }
    return ret;		
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
	if(availableLength > UBLOX_MAX_PAYLOAD-2)
	{
		rxBuffer.clear();
		return;
	}
	availableLength = rxBuffer.peekbytes(ubloxAvailableBuffer, availableLength);
	if( availableLength>8 )
	{
		for(index=0; index<(availableLength-1); index++)
		{
			if( (ubloxAvailableBuffer[index] == 0xb5) && (ubloxAvailableBuffer[index+1] == 0x62) )
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
				if( (CK_A!=ubloxAvailableBuffer[headIndexBuffer[k+1]-2]) || (CK_B!=ubloxAvailableBuffer[headIndexBuffer[k+1]-1]) )
				{
					frameCheckError ++;
					continue;
				}
				else
				{
					frameCheckOK ++;
				}
				ubloxData.classId = ubloxAvailableBuffer[headIndexBuffer[k]+2];
				ubloxData.id = ubloxAvailableBuffer[headIndexBuffer[k]+3];
				payloadLength = (ubloxAvailableBuffer[headIndexBuffer[k]+5]<<8) | (ubloxAvailableBuffer[headIndexBuffer[k]+4]);
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

void Ublox::sendGpsData(void)
{
	int gpsDataSendFlag = 0;
	unsigned char verify;
	uint8_t SendGpsData[113];
	memset( SendGpsData, 0, sizeof(SendGpsData) );
	
	SendGpsData[0] = 0x0B;
	SendGpsData[1] = 113;	//181
	SendGpsData[2] = 0x49;
	ulong2buf((char *)(SendGpsData+3), &gpsData.iTOW);
	double2buf((char *)(SendGpsData+7), &gpsData.lat);
	double2buf((char *)(SendGpsData+15), &gpsData.lon);
	float2buf((char *)(SendGpsData+23), &gpsData.height);
	float2buf((char *)(SendGpsData+27), &gpsData.hAcc);
	float2buf((char *)(SendGpsData+31), &gpsData.vAcc);
	float2buf((char *)(SendGpsData+35), &gpsData.velN);
	float2buf((char *)(SendGpsData+39), &gpsData.velE);
	float2buf((char *)(SendGpsData+43), &gpsData.velD);
	float2buf((char *)(SendGpsData+47), &gpsData.speed);
	float2buf((char *)(SendGpsData+51), &gpsData.heading);	
	float2buf((char *)(SendGpsData+55), &gpsData.sAcc);
	float2buf((char *)(SendGpsData+59), &gpsData.cAcc);
	float2buf((char *)(SendGpsData+63), &gpsData.pDOP);
	float2buf((char *)(SendGpsData+67), &gpsData.hDOP);
	float2buf((char *)(SendGpsData+71), &gpsData.vDOP);
	float2buf((char *)(SendGpsData+75), &gpsData.tDOP);
	float2buf((char *)(SendGpsData+79), &gpsData.nDOP);
	float2buf((char *)(SendGpsData+83), &gpsData.eDOP);
	float2buf((char *)(SendGpsData+87), &gpsData.gDOP);
	
	SendGpsData[91] = (uint8_t)((int)gpsData.year&0x000000ff);
	SendGpsData[92] = (uint8_t)(((int)gpsData.year>>8)&0x000000ff);
	SendGpsData[93] = gpsData.month;
	SendGpsData[94] = gpsData.day;
	SendGpsData[95] = gpsData.hour;
	SendGpsData[96] = gpsData.min;
	SendGpsData[97] = gpsData.sec;
	
	SendGpsData[98] = gpsData.fix_type;
	SendGpsData[99] = gpsData.satellites_used;
	SendGpsData[100] = gpsData.carrSoln;
	SendGpsData[101] = gpsData.vel_updateFlag;
	SendGpsData[102] = gpsData.pos_updateFlag;
	
	SendGpsData[103] = useRtkHeadFlag;
	double2buf((char *)(SendGpsData+104), &rtkHead);
	
	verify = 0;
	
	for(int i=0; i<112; i++)
	{
		verify ^= SendGpsData[i];
	}
	SendGpsData[112] = verify;
	
//	adsp.writeData( SendGpsData, 113);
	
//	delay_ms(10);
}
