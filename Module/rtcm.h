#ifndef rtcm_h
#define rtcm_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <hal_rtos.h>

#define RTCM_WAIT_MS			50
#define RTCM_MAX_PAYLOAD		100

class Rtcm
{
public:
	Rtcm( HAL_UART *_dev )
	{
		dev = _dev;
		
		availableLength = 0;
		index = 0;
		headCnt = 0;
		payloadLength = 0;
		
	}
	
	HAL_UART *dev;

	int receiveData(uint8_t *pData, int len);
	void rtcmSend(void);
	uint16_t availableLength;
	uint32_t index;
	uint32_t headCnt;
	uint32_t payloadLength;
	
	ByteBuffer rxBuffer{TOF_MAX_PAYLOAD};	
};





















#endif
