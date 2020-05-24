#ifndef ublox_F9P_h
#define ublox_F9P_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>

#include <hal_delay.h>

#define UBLOX_MAX_PAYLOAD   255
#define UBLOX_WAIT_MS	    50


/*******  Configuration Item  **********  Key ID  **********/

/**********NMEA MESSEAGE*********************/
#define MSGOUT_NMEA_ID_DTM_I2C			0x209100a6
#define MSGOUT_NMEA_ID_DTM_SPI			0x209100aa
#define MSGOUT_NMEA_ID_DTM_UART1		0x209100a7
#define MSGOUT_NMEA_ID_DTM_UART2		0x209100a8
#define MSGOUT_NMEA_ID_DTM_USB			0x209100a9

#define MSGOUT_NMEA_ID_GBS_I2C			0x209100dd
#define MSGOUT_NMEA_ID_GBS_SPI			0x209100e1
#define MSGOUT_NMEA_ID_GBS_UART1		0x209100de
#define MSGOUT_NMEA_ID_GBS_UART2		0x209100df
#define MSGOUT_NMEA_ID_GBS_USB			0x209100e0

#define MSGOUT_NMEA_ID_GGA_I2C			0x209100ba
#define MSGOUT_NMEA_ID_GGA_SPI			0x209100be
#define MSGOUT_NMEA_ID_GGA_UART1		0x209100bb
#define MSGOUT_NMEA_ID_GGA_UART2		0x209100bc
#define MSGOUT_NMEA_ID_GGA_USB			0x209100bd

#define MSGOUT_NMEA_ID_GLL_I2C			0x209100c9
#define MSGOUT_NMEA_ID_GLL_SPI			0x209100cd
#define MSGOUT_NMEA_ID_GLL_UART1		0x209100ca
#define MSGOUT_NMEA_ID_GLL_UART2		0x209100cb
#define MSGOUT_NMEA_ID_GLL_USB			0x209100cc

#define MSGOUT_NMEA_ID_GNS_I2C			0x209100b5
#define MSGOUT_NMEA_ID_GNS_SPI			0x209100b9
#define MSGOUT_NMEA_ID_GNS_UART1		0x209100b6
#define MSGOUT_NMEA_ID_GNS_UART2		0x209100b7
#define MSGOUT_NMEA_ID_GNS_USB			0x209100b8

#define MSGOUT_NMEA_ID_GRS_I2C			0x209100ce
#define MSGOUT_NMEA_ID_GRS_SPI			0x209100d2
#define MSGOUT_NMEA_ID_GRS_UART1		0x209100cf
#define MSGOUT_NMEA_ID_GRS_UART2		0x209100d0
#define MSGOUT_NMEA_ID_GRS_USB			0x209100d1

#define MSGOUT_NMEA_ID_GSA_I2C			0x209100bf
#define MSGOUT_NMEA_ID_GSA_SPI			0x209100c3
#define MSGOUT_NMEA_ID_GSA_UART1		0x209100c0
#define MSGOUT_NMEA_ID_GSA_UART2		0x209100c1
#define MSGOUT_NMEA_ID_GSA_USB			0x209100c2

#define MSGOUT_NMEA_ID_GST_I2C			0x209100d3
#define MSGOUT_NMEA_ID_GST_SPI			0x209100d7
#define MSGOUT_NMEA_ID_GST_UART1		0x209100d4
#define MSGOUT_NMEA_ID_GST_UART2		0x209100d5
#define MSGOUT_NMEA_ID_GST_USB			0x209100d6

#define MSGOUT_NMEA_ID_GSV_I2C			0x209100c4
#define MSGOUT_NMEA_ID_GSV_SPI			0x209100c8
#define MSGOUT_NMEA_ID_GSV_UART1		0x209100c5
#define MSGOUT_NMEA_ID_GSV_UART2		0x209100c6
#define MSGOUT_NMEA_ID_GSV_USB			0x209100c7

#define MSGOUT_NMEA_ID_RMC_I2C			0x209100ab
#define MSGOUT_NMEA_ID_RMC_SPI			0x209100af
#define MSGOUT_NMEA_ID_RMC_UART1		0x209100ac
#define MSGOUT_NMEA_ID_RMC_UART2		0x209100ad
#define MSGOUT_NMEA_ID_RMC_USB			0x209100ae

#define MSGOUT_NMEA_ID_VLW_I2C			0x209100e7
#define MSGOUT_NMEA_ID_VLW_SPI			0x209100eb
#define MSGOUT_NMEA_ID_VLW_UART1		0x209100e8
#define MSGOUT_NMEA_ID_VLW_UART2		0x209100e9
#define MSGOUT_NMEA_ID_VLW_USB			0x209100ea

#define MSGOUT_NMEA_ID_VTG_I2C			0x209100b0
#define MSGOUT_NMEA_ID_VTG_SPI			0x209100b4
#define MSGOUT_NMEA_ID_VTG_UART1		0x209100b1
#define MSGOUT_NMEA_ID_VTG_UART2		0x209100b2
#define MSGOUT_NMEA_ID_VTG_USB			0x209100b3

#define MSGOUT_NMEA_ID_ZDA_I2C			0x209100d8
#define MSGOUT_NMEA_ID_ZDA_SPI			0x209100dc
#define MSGOUT_NMEA_ID_ZDA_UART1		0x209100d9
#define MSGOUT_NMEA_ID_ZDA_UART2		0x209100da
#define MSGOUT_NMEA_ID_ZDA_USB			0x209100db

/**********RTCM3X MESSEAGE*********************/
#define MSGOUT_RTCM_3X_TYPE1005_I2C		0x209102bd
#define MSGOUT_RTCM_3X_TYPE1005_SPI		0x209102c1
#define MSGOUT_RTCM_3X_TYPE1005_UART1	0x209102be
#define MSGOUT_RTCM_3X_TYPE1005_UART2	0x209102bf
#define MSGOUT_RTCM_3X_TYPE1005_USB		0x209102c0

#define MSGOUT_RTCM_3X_TYPE1077_I2C		0x209102cc
#define MSGOUT_RTCM_3X_TYPE1077_SPI		0x209102d0
#define MSGOUT_RTCM_3X_TYPE1077_UART1	0x209102cd
#define MSGOUT_RTCM_3X_TYPE1077_UART2	0x209102ce
#define MSGOUT_RTCM_3X_TYPE1077_USB		0x209102cf

#define MSGOUT_RTCM_3X_TYPE1087_I2C		0x209102d1
#define MSGOUT_RTCM_3X_TYPE1087_SPI		0x209102d5
#define MSGOUT_RTCM_3X_TYPE1087_UART1	0x209102d2
#define MSGOUT_RTCM_3X_TYPE1087_UART2	0x209102d3
#define MSGOUT_RTCM_3X_TYPE1087_USB		0x209102d4

#define MSGOUT_RTCM_3X_TYPE1097_I2C		0x20910318
#define MSGOUT_RTCM_3X_TYPE1097_SPI		0x2091031c
#define MSGOUT_RTCM_3X_TYPE1097_UART1	0x20910319
#define MSGOUT_RTCM_3X_TYPE1097_UART2	0x2091031a
#define MSGOUT_RTCM_3X_TYPE1097_USB		0x2091031b

#define MSGOUT_RTCM_3X_TYPE1127_I2C		0x209102d6
#define MSGOUT_RTCM_3X_TYPE1127_SPI		0x209102da
#define MSGOUT_RTCM_3X_TYPE1127_UART1	0x209102d7
#define MSGOUT_RTCM_3X_TYPE1127_UART2	0x209102d8
#define MSGOUT_RTCM_3X_TYPE1127_USB		0x209102d9

#define MSGOUT_RTCM_3X_TYPE1230_I2C		0x20910303
#define MSGOUT_RTCM_3X_TYPE1230_SPI		0x20910307
#define MSGOUT_RTCM_3X_TYPE1230_UART1	0x20910304
#define MSGOUT_RTCM_3X_TYPE1230_UART2	0x20910305
#define MSGOUT_RTCM_3X_TYPE1230_USB		0x20910306

#define MSGOUT_RTCM_3X_TYPE4072_0_I2C	0x209102fe
#define MSGOUT_RTCM_3X_TYPE4072_0_SPI	0x20910302
#define MSGOUT_RTCM_3X_TYPE4072_0_UART1	0x209102ff
#define MSGOUT_RTCM_3X_TYPE4072_0_UART2	0x20910300
#define MSGOUT_RTCM_3X_TYPE4072_0_USB	0x20910301

#define MSGOUT_RTCM_3X_TYPE4072_1_I2C	0x20910381
#define MSGOUT_RTCM_3X_TYPE4072_1_SPI	0x20910385
#define MSGOUT_RTCM_3X_TYPE4072_1_UART1	0x20910382
#define MSGOUT_RTCM_3X_TYPE4072_1_UART2	0x20910383
#define MSGOUT_RTCM_3X_TYPE4072_1_USB	0x20910384

/**********UBX MESSEAGE*********************/
#define UBX_NAV_CLASS					0x01

#define MSGOUT_UBX_NAV_DOP_I2C			0x20910038
#define MSGOUT_UBX_NAV_DOP_SPI			0x2091003c
#define MSGOUT_UBX_NAV_DOP_UART1		0x20910039
#define MSGOUT_UBX_NAV_DOP_UART2		0x2091003a
#define MSGOUT_UBX_NAV_DOP_USB			0x2091003b
#define UBX_NAV_DOP_ID					0x04

#define MSGOUT_UBX_NAV_POSLLH_I2C		0x20910029
#define MSGOUT_UBX_NAV_POSLLH_SPI		0x2091002d
#define MSGOUT_UBX_NAV_POSLLH_UART1		0x2091002a
#define MSGOUT_UBX_NAV_POSLLH_UART2		0x2091002b
#define MSGOUT_UBX_NAV_POSLLH_USB		0x2091002c
#define UBX_NAV_POSLLH_ID				0x02

#define MSGOUT_UBX_NAV_PVT_I2C			0x20910006
#define MSGOUT_UBX_NAV_PVT_SPI			0x2091000a
#define MSGOUT_UBX_NAV_PVT_UART1		0x20910007
#define MSGOUT_UBX_NAV_PVT_UART2		0x20910008
#define MSGOUT_UBX_NAV_PVT_USB			0x20910009
#define UBX_NAV_PVT_ID					0x07

#define MSGOUT_UBX_NAV_RELPOSNED_I2C	0x2091008d
#define MSGOUT_UBX_NAV_RELPOSNED_SPI	0x20910091
#define MSGOUT_UBX_NAV_RELPOSNED_UART1	0x2091008e
#define MSGOUT_UBX_NAV_RELPOSNED_UART2	0x2091008f
#define MSGOUT_UBX_NAV_RELPOSNED_USB	0x20910090
#define UBX_NAV_RELPOSNED_ID			0x3C

#define MSGOUT_UBX_NAV_VELNED_I2C		0x20910042
#define MSGOUT_UBX_NAV_VELNED_SPI		0x20910046
#define MSGOUT_UBX_NAV_VELNED_UART1		0x20910043
#define MSGOUT_UBX_NAV_VELNED_UART2		0x20910044
#define MSGOUT_UBX_NAV_VELNED_USB		0x20910045
#define UBX_NAV_VELNED_ID				0x12

#define UBX_RXM_CLASS					0x02

#define MSGOUT_UBX_RXM_RTCM_I2C			0x20910268
#define MSGOUT_UBX_RXM_RTCM_SPI			0x2091026c
#define MSGOUT_UBX_RXM_RTCM_UART1		0x20910269
#define MSGOUT_UBX_RXM_RTCM_UART2		0x2091026a
#define MSGOUT_UBX_RXM_RTCM_USB			0x2091026b
#define UBX_RXM_RTCM_ID					0x32

/**********CFG-NMEA*********************/
#define NMEA_HIGHPREC					0x10930006

/**********CFG-NAVHPG*********************/
#define NAVHPG_DGNSSMODE				0x20140011

/**********CFG-RATE*********************/
#define RATE_MEAS						0x30210001

/**********CFG-SIGNAL*********************/
#define SIGNAL_GPS_ENA					0x1031001f
#define SIGNAL_GPS_L1CA_ENA				0x10310001
#define SIGNAL_GPS_L2C_ENA				0x10310003

#define SIGNAL_GAL_ENA					0x10310021
#define SIGNAL_GAL_E1_ENA				0x10310007
#define SIGNAL_GAL_E5B_ENA				0x1031000a

#define SIGNAL_BDS_ENA					0x10310022
#define SIGNAL_BDS_B1_ENA				0x1031000d
#define SIGNAL_BDS_B2_ENA				0x1031000e

#define SIGNAL_QZSS_ENA					0x10310024
#define SIGNAL_QZSS_L1CA_ENA			0x10310012
#define SIGNAL_QZSS_L2C_ENA				0x10310015

#define SIGNAL_GLO_ENA					0x10310025
#define SIGNAL_GLO_L1_ENA				0x10310018
#define SIGNAL_GLO_L2_ENA				0x1031001a

/**********CFG-TMODE*********************/
#define TMODE_MODE						0x20030001
#define TMODE_SVIN_MIN_DUR				0x40030010
#define TMODE_SVIN_ACC_LIMIT			0x40030011

/**********CFG-UART1*********************/
#define UART1_BAUDRATE					0x40520001

#define UART1INPROT_UBX					0x10730001
#define UART1INPROT_NMEA				0x10730002
#define UART1INPROT_RTCM3X				0x10730004

#define UART1OUTPROT_UBX				0x10740001
#define UART1OUTPROT_NMEA				0x10740002
#define UART1OUTPROT_RTCM3X				0x10740004

/**********CFG-UART2*********************/
#define UART2_BAUDRATE					0x40530001

#define UART2INPROT_UBX					0x10750001
#define UART2INPROT_NMEA				0x10750002
#define UART2INPROT_RTCM3X				0x10750004

#define UART2OUTPROT_UBX				0x10760001
#define UART2OUTPROT_NMEA				0x10760002
#define UART2OUTPROT_RTCM3X				0x10760004

/**********CFG-VALSET*********************/
#define UBX_CFG_VALSET_CLASS			0x06
#define UBX_CFG_VALSET_ID				0x8A

#define SAVE_TO_RAM						0x01
#define SAVE_TO_BBR						0x02
#define SAVE_TO_FLASH					0x04

#define RTK_FLOAT						0x02
#define RTK_FIXED						0x03


/*******UBX-NAV-DOP(Dilution of precision)********/
typedef struct {			/*   Description				Scaling		Uint	*/
    uint32_t iTOW;			/* GPS Time of Week			  	  -			 ms		*/
    uint16_t gDOP;			/* Geometric DOP			 	 0.01		 -		*/
    uint16_t pDOP;			/* Position DOP				     0.01		 -		*/
    uint16_t tDOP;			/* Time DOP					 	 0.01		 -		*/
    uint16_t vDOP;			/* Vertical DOP				 	 0.01		 -		*/
    uint16_t hDOP;			/* Horizontal DOP			 	 0.01		 -		*/
    uint16_t nDOP;			/* Northing DOP				 	 0.01		 -		*/
    uint16_t eDOP;			/* Easting DOP				 	 0.01		 -		*/
} __attribute__((packed)) ubloxStructDOP_t;

/*******UBX-NAV-POSLLH(Geodetic Position Solution)********/
typedef struct {			/*   Description				Scaling		Uint	*/
    uint32_t iTOW;	    	/* GPS Time of Week			 	   -		 ms		*/
    int  lon;    			/* Longitude		  	  	      1e-7		 deg	*/
    int  lat;    			/* Latitude			  			  1e-7		 deg	*/
    int	 height;   			/* Height above ellipsoid		   -		 mm		*/
    int  hMSL;    			/* Height sea level				   -		 mm		*/
    uint32_t hAcc;    		/* Horizontal accuracy estimate	   -		 mm		*/
    uint32_t vAcc;    		/* Vertical accuracy estimate	   -		 mm		*/
} __attribute__((packed)) ubloxStructPOSLLH_t;

/*******UBX-NAV-PVT(Position Velocity Time Solution)********/ 
typedef struct {			/*   Description				Scaling		Uint	*/
    uint32_t iTOW;			/* GPS Time of Week			 	   -		 ms		*/
    uint16_t year;			/* Year(UTC)				   	   -		 y		*/
    uint8_t month;			/* Month(UTC,1-12)			  	   -	     month	*/
    uint8_t day;			/* Day(UTC,1-31)		 	  	   -	     d		*/
    uint8_t hour;			/* Hour(UTC,0-23)		 	  	   -	     h		*/
    uint8_t min;			/* Minute(UTC,0-59)		 	  	   -	     min	*/
    uint8_t sec;			/* Second(UTC,0-60)		 	  	   -	     s		*/
    uint8_t valid;
    uint32_t tAcc;			/* Time accuracy estimate(UTC)	   -	     ns		*/
    int nano;
    uint8_t fixType;   		/* GNSSfix Type			  	 	   -		 -		*/
    uint8_t flags;			/* Fix status flags			  	   -		 -		*/
    uint8_t flags2;
    uint8_t numSV;      	/* Number of satellites		  	   -		 -		*/
    int lon;				/* Longitude		  	  	      1e-7		 deg	*/
    int lat;				/* Latitude			  			  1e-7		 deg	*/
    int height;				/* Height above ellipsoid		   -		 mm		*/
    int hMSL;				/* Height sea level				   -		 mm		*/
    uint32_t hAcc;			/* Horizontal accuracy estimate	   -		 mm		*/
    uint32_t vAcc;			/* Vertical accuracy estimate	   -		 mm		*/
    int velN;				/*  NED north velocity	   		   -		 mm/s	*/
    int velE;				/*  NED east velocity	   		   -		 mm/s	*/
    int velD;				/*  NED down velocity	   		   -		 mm/s	*/
    int gSpeed;				/*  Ground Speed (2-D)	   		   -		 mm/s	*/
    int headMot;			/*  Heading of motion (2-D)	   	  1e-5		 deg	*/
    uint32_t sAcc;			/*  Speed accuracy estimate	   	   -		 mm/s	*/
    uint32_t headAcc;		/*  Heading accuracy estimate 	  1e-5		 deg	*/
    uint16_t pDOP;			/*  Position DOP				  0.01		 -		*/
    uint16_t reserved1;
    uint32_t reserved2;
	int headVeh;			/*  Heading of vehicle (2-D) 	  1e-5		 deg	*/
    short int magDec;		/*  Magnetic declination 		  1e-2		 deg	*/
	uint16_t magAcc;		/*  Magnetic declination accuracy 1e-2		 deg	*/
} __attribute__((packed)) ubloxStructPVT_t;

/*******UBX-NAV-RELPOSNED(Relative Positioning Information in NED frame)********/
typedef struct {			/*  Description				Scaling		Uint		*/
    uint8_t version;		/* Message version		  	  -		 	 -			*/
    uint8_t reserved1;
    uint16_t refStationId;	/* Reference Station ID		  -		 	 -			*/
    uint32_t iTOW;			/* GPS Time of Week			  -		 	 ms			*/
    int relPosN;			/* N relative position vector -			 cm			*/
    int relPosE;			/* E relative position vector -			 cm			*/
    int relPosD;			/* D relative position vector -			 cm			*/
	int relPosLength;		/* Length of antenna		  -			 cm			*/
	int relPosHeading;		/* Heading of  antenna		  -			 deg		*/
    uint32_t reserved2;				
	char relPosHPN;			/* relPosHPN				 0.1		 mm			*/
	char relPosHPE;			/* relPosHPE		  		 0.1		 mm			*/
	char relPosHPD;			/* relPosHPD				 0.1		 mm			*/
	char relPosHPLength;	/* relPosHPLength			 0.1		 mm			*/
	uint32_t accN;			/* accN						 0.1		 mm			*/
	uint32_t accE;			/* accE						 0.1		 mm			*/
	uint32_t accD;			/* accD						 0.1		 mm			*/
	uint32_t accLength;		/* accLength				 0.1		 mm			*/
	uint32_t accHeading;	/* accHeading				 1e-5		 deg		*/
	uint32_t reserved3;
	uint32_t flags;			/* Fix status flags			  	   -		 -		*/		
} __attribute__((packed)) ubloxStructRELPOSNED_t;

/*******UBX-NAV-VELNED(Velocity Solution in NED)********/
typedef struct {			/*   Description				Scaling		Uint	*/
    uint16_t iTOW;			/* GPS Time of Week			 	   -		 ms		*/
    int 	 velN;			/*  NED north velocity	   		   -		 cm/s	*/
    int 	 velE;			/*  NED east velocity	   		   -		 cm/s	*/
    int 	 velD;			/*  NED down velocity	   		   -		 cm/s	*/
    uint32_t speed;			/*  Speed (3-D)	   		   		   -		 cm/s	*/
    uint32_t gSpeed;		/*  Ground Speed (2-D)	   		   -		 cm/s	*/
    int      heading;		/*  Heading of motion (2-D)	   	  1e-5		 deg	*/
    uint32_t sAcc;			/*  Speed accuracy estimate	   	   -		 cm/s	*/
    uint32_t cAcc;			/*  Course/Heading accuracy est   1e-5		 deg	*/
} __attribute__((packed)) ubloxStructVELNED_t;

/*******UBX-RXM_RTCM(RTCM input status)********/
typedef struct {			/*   Description				Scaling		Uint	*/
    uint8_t  version;		/* Message version		  	  	   -		 -		*/
    uint8_t  flags;			/* RTCM input status flags		   -		 -		*/
    uint16_t subType;		/* Message subtype				   -		 -		*/
    uint16_t refStation;	/* Reference station ID			   -		 -		*/
    uint16_t msgType;		/* Message type					   -		 -		*/
} __attribute__((packed)) ubloxStructRTCM_t;

typedef struct 
{
    union
	{
		ubloxStructDOP_t dop;
		ubloxStructPOSLLH_t posllh;
		ubloxStructPVT_t pvt;
		ubloxStructRELPOSNED_t relposned;
		ubloxStructVELNED_t velned;
		
		ubloxStructRTCM_t rtcm;
		
		uint8_t other[UBLOX_MAX_PAYLOAD];
    } payload;

    uint8_t  state;
    uint32_t count;
    uint8_t  classId;
    uint8_t  id;
	uint8_t  nmeaHead;
    uint32_t length;
    uint32_t checksumErrors;

    uint8_t  ubloxRxCK_A;
    uint8_t  ubloxRxCK_B;

    uint8_t  ubloxTxCK_A;
    uint8_t  ubloxTxCK_B;
} ubloxStruct_t __attribute__((aligned));

typedef struct {
	//NAC-DOP
//	uint32_t iTOW;
	uint16_t gDOP;     	// Geometric DOP
//	uint16_t pDOP;     	// Position Dilution of Precision
	uint16_t hDOP;     	// Horizontal DOP
	uint16_t vDOP;     	// Vertical DOP
	uint16_t tDOP;     	// Time DOP
	uint16_t nDOP;     	// Northing DOP
	uint16_t eDOP;     	// Easting DOP
	
//	//NAV-POSLLH
//	uint32_t iTOW;
//	int lon;
//	int lat;
//	int height;
//	int hMSL;   		// above mean sea level (m)
//	uint32_t hAcc;     	// horizontal accuracy est (m)
//	uint32_t vAcc;     	// vertical accuracy est (m)
	
	//NAV-PVT
	uint32_t iTOW;
	uint16_t year;    	// Year, range 1999..2099 (UTC)
	uint8_t month;   	// Month, range 1..12 (UTC)
	uint8_t day;	    // Day of Month, range 1..31 (UTC)
	uint8_t hour;	    // Hour of Day, range 0..23 (UTC)
	uint8_t min;	    // Minute of Hour, range 0..59 (UTC)
	uint8_t sec;	    // Second of Minute, range 0..59 (UTC)
	uint8_t valid;
	uint32_t tAcc;		//Time accuracy estimate
	int nano;
	uint8_t fixType;
	uint8_t pvtFlags;
	uint8_t pvtFlags2;
	uint8_t numSV;
	int lon;
	int lat;
	int height;
	int hMSL;
	uint32_t hAcc;
	uint32_t vAcc;
	int velN;     		// north velocity (m/s)
	int velE;     		// east velocity (m/s)
	int velD;     		// down velocity (m/s)
	int gSpeed;    		// ground speed (m/s)
	int headMot;		//Heading of motion (2-D)
	uint32_t sAcc;
	uint32_t headAcc;	//Heading accuracy estimate (both motion andvehicle)
	uint16_t pDOP;
//	uint16_t pvtReserved1;
//	uint32_t pvtReserved1;
	int headVeh;
	short int magDec;
	uint16_t magAcc;
	
	uint8_t carrSoln;
	
//	//NAV-VELNED
//	int velN;     		// north velocity (m/s)
//	int velE;     		// east velocity (m/s)
//	int velD;     		// down velocity (m/s)
	uint32_t speed;		//Speed (3-D)
//	uint32_t gSpeed;    // ground speed (m/s)
	int heading;  		// Heading of motion 2-D
//	uint32_t sAcc;     	// speed accuracy est (m/s)
	uint32_t cAcc;     	// Course / Heading accuracy estimate
	
	//NAV-RELPOSNED
	uint8_t relVersion;
//	uint8_t relReserved1
	uint16_t refStationId;
//	uint32_t iTOW
	int relPosN;
	int relPosE;
	int relPosD;
	int relPosLength;
	int relPosHeading;
//	uint32_t relReserved2
	uint8_t relPosHPN;
	uint8_t relPosHPE;
	uint8_t relPosHPD;
	uint8_t relPosHPLength;
	uint32_t accN;
	uint32_t accE;
	uint32_t accD;
	uint32_t accLength;
	uint32_t accHeading;
	uint32_t relFlags;
	uint32_t relGnssFixOK;
	uint32_t relDiffSoln;
	uint32_t relPosValid;
	uint32_t relCarrSoln;
	uint32_t relHeadingValid;
	
	
	//RXM-RTCM
//	uint8_t rtcmVersion
	uint8_t crcFailedFlags;
    uint16_t subType;
    uint16_t refStation;
    uint16_t msgType;
	uint32_t count_1005;
	uint32_t count_1077;
	uint32_t count_1087;
	uint32_t count_1097;
	uint32_t count_1127;
	uint32_t count_1230;
	uint32_t count_4072_0;
	uint32_t count_4072_1;
	
	uint8_t DopUpdateFlag;
	uint8_t PosllhUpdateFlag;
	uint8_t PvtUpdateFlag;
	uint8_t RelposnedUpdateFlag;
	uint8_t VelnedUpdateFlag;
	uint8_t RxmRtcmUpdateFlag;

} gpsStruct_t;

class Ublox
{
public:
	Ublox( HAL_UART *_dev )
	{ 
		dev = _dev;
		
		availableLength = 0;
		index = 0;
		headCnt = 0;
		payloadLength = 0;
	
		frameHeadError = 0;
		frameCheckError = 0;
		frameCheckOK = 0 ;
	}
		
	HAL_UART *dev;
	
	friend class SendGpsData;
	
	void ConfigOutputMessage(uint32_t _keyID, uint8_t _layers, uint8_t _rate);
	
	void ConfigBuadrate(uint32_t _keyID, uint8_t _layers, uint32_t _buadRate);
	
	void ConfigSignal(uint32_t _gnssKeyID, uint8_t _layers, bool _enable);
	
	void ConfigRate(uint32_t _keyID, uint8_t _layers, uint32_t _ms);
	
	void ubloxInit(void);
	
	void ubloxSendSetup(void);
	
	void ConfigRtcmOutputMesseage(void);
	
	void ConfigGnssSystem(void);
	
	uint8_t ubloxPublish(void);	
	
	void UbloxDataSend(void);
	
	int receiveData(uint8_t *pData, int len);
	
	void parseData(void);
	
	void SendDopData(void);

	void SendPosllhData(void);

	void SendPvtData(void);

	void SendRelposnedData(void);

	void SendVelnedData(void);

	void SendRxmRtcmData1(void);
	
	void SendRxmRtcmData2(void);
	
	uint16_t availableLength;
	uint32_t index;
	uint32_t headCnt;
	uint32_t payloadLength;
	
	uint16_t frameHeadError;
	uint16_t frameCheckError;
	uint32_t frameCheckOK;
	
	ByteBuffer rxBuffer{UBLOX_MAX_PAYLOAD};
	ubloxStruct_t ubloxData;
	
	gpsStruct_t gpsData;
};

#endif