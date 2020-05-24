#ifndef ublox_h
#define ublox_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>

//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#include <hal_rtos.h>
#include <hal_delay.h>

#include "type_conversion.h"

#define GPS_DO_RTK


//#define GPS_BAUD_RATE           38400

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)
#define UBLOX_HWVERSION			8u


#define UBLOX_SYNC1	    0xB5
#define UBLOX_SYNC2	    0x62
/* Message Classes */
#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_CFG_CLASS	    0x06
#define UBLOX_MON_CLASS	    0x0a
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d

/* Message IDs */
#define UBLOX_NAV_POSLLH    0x02
#define UBLOX_NAV_DOP	    0x04
#define UBLOX_NAV_RELPOSNED	0x3C

#define UBX_ID_NAV_SOL	    0x06
#define UBX_ID_NAV_PVT	    0x07

#define UBLOX_NAV_VELNED    0x12
#define UBLOX_NAV_TIMEUTC   0x21
#define UBLOX_NAV_SBAS	    0x32
#define UBLOX_NAV_SVINFO    0x30

#define UBLOX_AID_REQ	    0x00

#define UBLOX_RXM_RAW	    0x10
#define UBLOX_RXM_SFRB	    0x11
#define UBLOX_RXM_RTCM	    0x32

#define UBLOX_MON_VER	    0x04
#define UBLOX_MON_HW	    0x09

#define UBLOX_TIM_TP	    0x01

#define UBLOX_CFG_PRT       0x00
#define UBLOX_CFG_MSG	    0x01
#define UBLOX_CFG_TP	    0x07
#define UBLOX_CFG_RATE	    0x08
#define UBLOX_CFG_SBAS	    0x16
#define UBLOX_CFG_NMEA		0x17
#define UBLOX_CFG_NAV5	    0x24

#define UBLOX_SBAS_AUTO	    0x00000000
#define UBLOX_SBAS_WAAS	    0x0004E004
#define UBLOX_SBAS_EGNOS    0x00000851
#define UBLOX_SBAS_MSAS	    0x00020200
#define UBLOX_SBAS_GAGAN    0x00000108

#define UBLOX_MAX_PAYLOAD   180
#define UBLOX_WAIT_MS	    50


// NMEA Protocol
// Standard Message
#define NMEA_STD_CLASSID 0xF0
#define NMEA_DTM_ID 0x0A
#define NMEA_GBQ_ID 0x44
#define NMEA_GBS_ID 0x09
#define NMEA_GGA_ID 0x00
#define NMEA_GLL_ID 0x01
#define NMEA_GLQ_ID 0x43
#define NMEA_GNQ_ID 0x42
#define NMEA_GNS_ID 0x0D
#define NMEA_GPQ_ID 0x40
#define NMEA_GRS_ID 0x06
#define NMEA_GSA_ID 0x02
#define NMEA_GST_ID 0x07
#define NMEA_GSV_ID 0x03
#define NMEA_RMC_ID 0x04
#define NMEA_TXT_ID 0x41
#define NMEA_VLW_ID 0x0F
#define NMEA_VTG_ID 0x05
#define NMEA_ZDA_ID 0x08
// PUBX Message
#define NMEA_PUBX_CLASSID 0xF1
#define NMEA_POSITION_DATA_ID 0x00
#define NMEA_UTM_POSITION_ID 0x01
#define NMEA_SATELLITE_DATA_ID 0x03
#define NMEA_TIME_OF_DAY_ID 0x04
#define NMEA_EKF_STATUS_ID 0x05
#define NMEA_GPS_ONLY_ON_EKF_PRODUCTS_ID 0x06

#define NMEA_RATE_ID 0x40
#define NMEA_CONFIG_ID 0x41




/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE		0x01	/**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME		0x02	/**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED	0x04	/**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK		0x01	/**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN		0x02	/**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE		0x1C	/**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID	0x20	/**< headVehValid (Heading of vehicle is valid) */

enum ubloxStates {
    UBLOX_WAIT_SYNC1 = 0,
    UBLOX_WAIT_SYNC2,
    UBLOX_WAIT_CLASS,
    UBLOX_WAIT_ID,
    UBLOX_WAIT_LEN1,
    UBLOX_WAIT_LEN2,
    UBLOX_PAYLOAD,
    UBLOX_CHECK1,
    UBLOX_CHECK2
};

// Geodetic Position Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long lon;	    // Longitude (deg * 1e-7)
    signed long lat;	    // Latitude (deg * 1e-7)
    signed long height;	    // Height above Ellipsoid (mm)
    signed long hMSL;	    // Height above mean sea level (mm)
    unsigned long hAcc;	    // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc;	    // Vertical Accuracy Estimate (mm)
} __attribute__((packed)) ubloxStructPOSLLH_t;

// Dilution of precision
typedef struct {
    unsigned long iTOW;	    // ms GPS Millisecond Time of Week
    unsigned short gDOP;    // Geometric DOP
    unsigned short pDOP;    // Position DOP
    unsigned short tDOP;    // Time DOP
    unsigned short vDOP;    // Vertical DOP
    unsigned short hDOP;    // Horizontal DOP
    unsigned short nDOP;    // Northing DOP
    unsigned short eDOP;    // Easting DOP
} __attribute__((packed)) ubloxStructDOP_t;

// Velocity Solution in NED
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long velN;	    // NED north velocity (cm/s)
    signed long velE;	    // NED east velocity (cm/s)
    signed long velD;	    // NED down velocity (cm/s)
    unsigned long speed;    // Speed (3-D) (cm/s)
    unsigned long gSpeed;   // Ground Speed (2-D) (cm/s)
    signed long heading;    // Heading 2-D (deg * 1e-5)
    unsigned long sAcc;	    // Speed Accuracy Estimate (cm/s)
    unsigned long cAcc;	    // Course / Heading Accuracy Estimate (deg * 1e-5)
} __attribute__((packed)) ubloxStructVALNED_t;

typedef struct {
    uint8_t version;	    // GPS Millisecond Time of Week (ms)
    uint8_t reserved1;	    // NED north velocity (cm/s)
    unsigned short refStationID;	    // NED east velocity (cm/s)
    uint32_t iTOW;    // Speed (3-D) (cm/s)
    int relPosN;   // Ground Speed (2-D) (cm/s)
    int relPosE;    // Heading 2-D (deg * 1e-5)
    int relPosD;	    // Speed Accuracy Estimate (cm/s)
	int relPosLength;
	int relPosHeading;
    uint32_t reserved2;	    // Course / Heading Accuracy Estimate (deg * 1e-5)
	char relPosHPN;
	char relPosHPE;
	char relPosHPD;
	char relPosHPLength;
	uint32_t accN;
	uint32_t accE;
	uint32_t accD;
	uint32_t accLength;
	uint32_t accHeading;
	uint32_t reserved3;
	uint32_t flags;	
} __attribute__((packed)) ubloxStructRELPOSNED_t;

// RTCM input status
typedef struct {
    uint8_t version;
    uint8_t crcFailedFlags;
    unsigned short subType;
    unsigned short refStation;
    unsigned short msgType;
} __attribute__((packed)) ubloxStructRTCM_t;

// Timepulse Timedata
typedef struct {
    unsigned long towMS;
    unsigned long towSubMS;
    signed long qErr;
    unsigned short week;
    unsigned char flags;
    unsigned char res;
} __attribute__((packed)) ubloxStructTP_t;

// UTC Time Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    unsigned long tAcc;	    // Time Accuracy Estimate
    long nano;		    // Nanosecond of second (UTC)
    unsigned short year;    // Year, range 1999..2099 (UTC)
    unsigned char month;    // Month, range 1..12 (UTC)
    unsigned char day;	    // Day of Month, range 1..31 (UTC)
    unsigned char hour;	    // Hour of Day, range 0..23 (UTC)
    unsigned char min;	    // Minute of Hour, range 0..59 (UTC)
    unsigned char sec;	    // Second of Minute, range 0..59 (UTC)
    unsigned char valid;    // Validity Flags
} __attribute__((packed)) ubloxStructTIMEUTC_t;

// Receiver/Software Version
typedef struct {
    char swVersion[30];
    char hwVersion[10];
    char extension[30][7];
} __attribute__((packed)) ubloxStructVER_t;

// Position Velocity Time Solution
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;   /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
    uint8_t flags;
    uint8_t reserved1;
    uint8_t numSV;      /**< Number of SVs used in Nav Solution */
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint16_t reserved2;
    uint32_t reserved3;
} __attribute__((packed)) ubloxStructPVT_t;

typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	int32_t		fTOW;		/**< Fractional part of iTOW (range: +/-500000) [ns] */
	int16_t		week;		/**< GPS week */
	uint8_t		gpsFix;		/**< GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
	uint8_t		flags;
	int32_t		ecefX;
	int32_t		ecefY;
	int32_t		ecefZ;
	uint32_t	pAcc;
	int32_t		ecefVX;
	int32_t		ecefVY;
	int32_t		ecefVZ;
	uint32_t	sAcc;
	uint16_t	pDOP;
	uint8_t		reserved1;
	uint8_t		numSV;		/**< Number of SVs used in Nav Solution */
	uint32_t	reserved2;
} __attribute__((packed)) ubloxStructSOL_t;

typedef struct 
{
    int hwVer;

    signed long lastLat;
	signed long lastLon;
    union
	{
		ubloxStructPOSLLH_t posllh;
		ubloxStructVALNED_t valned;
		ubloxStructDOP_t dop;
		ubloxStructRTCM_t rtcm;
		ubloxStructTP_t tp;
		ubloxStructTIMEUTC_t timeutc;
		ubloxStructVER_t ver;
		ubloxStructPVT_t pvt;
		ubloxStructSOL_t sol;
		char other[UBLOX_MAX_PAYLOAD];
    } payload;

    unsigned char state;
    unsigned int count;
    unsigned char classId;
    unsigned char id;
    unsigned int length;
    unsigned int checksumErrors;

    unsigned char ubloxRxCK_A;
    unsigned char ubloxRxCK_B;

    unsigned char ubloxTxCK_A;
    unsigned char ubloxTxCK_B;
} ubloxStruct_t __attribute__((aligned));


typedef struct {
	unsigned long iTOW;
	int32_t lat_int;
	int32_t lon_int;
	int32_t height_int;
	
	int32_t delta_lat;
	int32_t delta_lon;
	double lat;
	double lon;
	float height;   // above mean sea level (m)
	float hAcc;     // horizontal accuracy est (m)
	float vAcc;     // vertical accuracy est (m)

	float delta_velN;
	float delta_velE;
	float delta_velD;
	int32_t velN_int;
	int32_t velE_int;
	int32_t velD_int;
	float velN;     // north velocity (m/s)
	float velE;     // east velocity (m/s)
	float velD;     // down velocity (m/s)
	float speed;    // ground speed (m/s)
	int32_t heading_int;  // course over ground (deg)
	float heading;  // course over ground (deg)
	float sAcc;     // speed accuracy est (m/s)
	float cAcc;     // course accuracy est (deg)
	float pDOP;     // Position Dilution of Precision
	float hDOP;     // Horizontal DOP
	float vDOP;     // Vertical DOP
	float tDOP;     // Time DOP
	float nDOP;     // Northing DOP
	float eDOP;     // Easting DOP
	float gDOP;     // Geometric DOP
	
	unsigned short year;    // Year, range 1999..2099 (UTC)
	unsigned char month;    // Month, range 1..12 (UTC)
	unsigned char day;	    // Day of Month, range 1..31 (UTC)
	unsigned char hour;	    // Hour of Day, range 0..23 (UTC)
	unsigned char min;	    // Minute of Hour, range 0..59 (UTC)
	unsigned char sec;	    // Second of Minute, range 0..59 (UTC)

	unsigned long TPtowMS;    // timepulse time of week (ms)
	unsigned long lastReceivedTPtowMS;
	unsigned long lastPosUpdate;
	unsigned long lastVelUpdate;
	unsigned long lastMessage;

	uint8_t fix_type;
	uint8_t satellites_used;
	
	uint8_t carrSoln;

	uint8_t vel_updateFlag;
	uint8_t pos_updateFlag;
	
	uint8_t crcFailedFlags;
    unsigned short subType;
    unsigned short refStation;
    unsigned short msgType;
	unsigned int count_1005;
	unsigned int count_1077;
	unsigned int count_1087;
	unsigned int count_1097;
	unsigned int count_1127;
	unsigned int count_1230;
	unsigned int count_4072_0;
	unsigned int count_4072_1;

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
	


	void ubloxSendPreamble(void);
	void ubloxEnableMessage(unsigned char c, unsigned char i, unsigned char rate);
	void ubloxSetRate(unsigned short int ms);
	void ubloxSetMode(void);
	void ubloxSetTimepulse(void);
	void ubloxSetSBAS(uint8_t enable);
	void ubloxSetNMEA(void);
	void ubloxPollVersion(void);
	void ubloxPollVersionNoDelay(void);
	void ubloxVersionSpecific(int ver);
	void ubloxSendPacket(uint8_t commType);
	
	unsigned char ubloxPublish(void);		
	int receiveData(uint8_t *pData, int len);
	void ubloxInit(void);
	void ubloxSendSetup(void);
	void ubloxInitGpsUsart1(uint32_t _baud);
	void ubloxInitGpsUsart2(uint32_t _baud);
	void parseData(void);
	
	
	void configF9p1(void);
	void configF9p2(void);
	
	void sendGpsData(void);
	
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
