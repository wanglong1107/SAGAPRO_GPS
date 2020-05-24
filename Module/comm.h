#ifndef comm_h
#define comm_h

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>
#include <aq_ukf/sensors_ukf.h>

#define COMM_BAUD_RATE		115200 // 115200 8E2N
#define COMM_MAX_PAYLOAD	256
#define COMM_DATA_SIZE		10



class COMM
{
public:
	COMM(HAL_UART *_dev);
	
	int initialize(void);
	int receiveData(uint8_t *pData, int len);
	void parseData(void);
	
	uint32_t availableLength = 0;
	uint32_t index = 0;
	uint32_t headCnt = 0;
	uint32_t payloadLength = 0;
	
	uint32_t frameError;
	uint32_t frameOK;
	
	ByteBuffer rxBuffer{COMM_MAX_PAYLOAD};
	
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
	
	float pwm1;
	float pwm2;
	float pwm3;
	float pwm4;
	
	
	uint8_t escCtlOK;
	uint32_t escCnt;
	
	uint8_t headLedColor;
	uint8_t tailLedColor;
	
	HAL_UART *dev;
};

#endif //esc
