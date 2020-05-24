#ifndef xsens_h
#define xsens_h

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>
#include <os.h>


#define XSENS_BAUD_RATE		115200
#define XSENS_MAX_PAYLOAD	256

#define MTI_700_DATA_SIZE 50
#define Deg2Rad (0.01745329251994329576923690768489f)
#define Rad2Deg (57.295779513082320876798154814105f)

typedef struct
{
	uint16_t crcErrorCnt;
	
	uint16_t packetCounter;
	uint16_t lostPacketCounter;

	// angular
	float roll;
	float pitch;
	float yaw;
	float Oyaw;
	// angular acceloration
	float xAcc;
	float yAcc;
	float zAcc;
	
	// angular velocity
	float xGyr;
	float yGyr;
	float zGyr;
	
	// altitude lat lon
	float altitude;
	float lat;
	float lon;
	
	//  velocity
	float vx;
	float vy;
	float vz;
	
}Xsens_ahrs_t;

class Xsens
{
public:
	Xsens( HAL_UART *_dev )
	{ 
		dev = _dev;
	}
		
	HAL_UART *dev;
	
	int initialize(void);
	int receiveData(uint8_t *pData, int len);
	int parseData(void);
	
	void GetEulerangle(uint8_t *pData, int index);
	void GetAcceleration(uint8_t *pData, int index);
	void GetGyroscope(uint8_t *pData, int index);
	void GetXYZ(uint8_t *pData, int index);
	void GetXYZvel(uint8_t *pData, int index);
	
	uint32_t availableLength = 0;
	uint32_t index = 0;
	uint32_t headCnt = 0;
	uint32_t payloadLength = 0;
	
	Xsens_ahrs_t ahrs;
	
	ByteBuffer rxBuffer{XSENS_MAX_PAYLOAD};	
};

#endif //xsens_h
