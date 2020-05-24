#ifndef sbus_h
#define sbus_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>
#include <os.h>


#define SBUS_BAUD_RATE		115200 // 115200 8E2N
#define SBUS_MAX_PAYLOAD	256
#define SBUS_DATA_SIZE		50

#define Deg2Rad (0.01745329251994329576923690768489f)
#define Rad2Deg (57.295779513082320876798154814105f)

typedef struct
{
	uint16_t sbusChannel[16];
	uint8_t connectStatus;
	uint8_t operateMode;
	
	uint8_t key_1;
	uint8_t key_2;
	uint8_t key_3;
	uint8_t key_4;
	uint8_t key_5;
	uint8_t key_6;
	
	uint8_t Digital1;
	uint8_t Digital2;
	uint8_t FailSafe;
	uint8_t FrameLost;
	
}Sbus_t;

class SBUS
{
public:
	SBUS( HAL_UART *_dev )
	{ 
		dev = _dev;
		
		sbus_ch.key_1 = 0;
		sbus_ch.key_2 = 0;
		sbus_ch.key_3 = 0;
		sbus_ch.key_4 = 0;
		sbus_ch.key_5 = 0;
		sbus_ch.key_6 = 0;
		
		
		sbus_ch.FailSafe = 1;
		sbus_ch.FrameLost = 1;
		
		
	}
		
	HAL_UART *dev;
	
	int initialize(void);
	int receiveData(uint8_t *pData, int len);
	int parseData(void);
	int ubloxPublish(uint8_t *pData, int len);
	
	uint32_t availableLength = 0;
	uint32_t index = 0;
	uint32_t headCnt = 0;
	uint32_t payloadLength = 0;
	
	uint32_t frameError;
	
	Sbus_t sbus_ch;
	
	ByteBuffer rxBuffer{SBUS_MAX_PAYLOAD};	
};

#endif //sbus_h
