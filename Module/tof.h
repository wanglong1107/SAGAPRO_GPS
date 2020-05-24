#ifndef tof_h
#define tof_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <hal_rtos.h>

#include "type_conversion.h"

#define TOF_WAIT_MS			50
#define TOF_MAX_PAYLOAD		80

typedef struct
{
	short int dist;
	short int amp;	
} __attribute__((packed)) tof_measurement_t;

typedef struct {
    char firmwareReserve;
	char firmwareMainVersion;	
	char firmwareSecondaryVersion;	
} __attribute__((packed)) tof_Firmware_t;

typedef struct {
    char hardwareReserve;
	char hardwareMainVersion;	
	char hardwareSecondaryVersion;		
} __attribute__((packed)) tof_Hardware_t;

typedef struct 
{
    union
	{
		tof_measurement_t tofMesurement;
		tof_Firmware_t tofFirmware;
		tof_Hardware_t tofHardware;
		char other[TOF_MAX_PAYLOAD];
    } payload;
	
	float dist;
	short int amp;
	uint8_t command;
	char firmwareReserve;
	char firmwareMainVersion;	
	char firmwareSecondaryVersion;	
	char hardwareReserve;
	char hardwareMainVersion;	
	char hardwareSecondaryVersion;
	uint8_t dist_updateFlag;
	uint8_t firmwareVersion_updateFlag;
	uint8_t hardwareVersion_updateFlag;
	short int lastDist;	
	short int deltaDist;
	short int dist_int;
} tofStruct_t __attribute__((aligned));

typedef struct 
{
	float dist;
	short int amp;
	char firmwareReserve;
	char firmwareMainVersion;	
	char firmwareSecondaryVersion;	
	char hardwareReserve;
	char hardwareMainVersion;	
	char hardwareSecondaryVersion;
	uint8_t dist_updateFlag;
	uint8_t firmwareVersion_updateFlag;
	uint8_t hardwareVersion_updateFlag;
	
} tofDataStruct_t;

class Tof
{
public:
	Tof( HAL_UART *_dev )
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
	
	void tofInit(uint32_t _baud);
	void tofSetRate(unsigned short int _ms);
	void tofSetDisInit(void);
	void tofPollFrameVersion(void);
	void tofPollHardwareVersion(void);
	int receiveData(uint8_t *pData, int len);
	void parseData(void);
	unsigned char tofPublish(void);

	void sendTofData(void);
	
	uint16_t availableLength;
	uint32_t index;
	uint32_t headCnt;
	uint32_t payloadLength;
	
	uint16_t frameHeadError;
	uint16_t frameCheckError;
	uint32_t frameCheckOK;
	
	tofStruct_t tofData;
	
	tofDataStruct_t tofOutputData;
	
	ByteBuffer rxBuffer{TOF_MAX_PAYLOAD};
};

#endif
