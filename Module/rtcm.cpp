#include "rtcm.h"

int Rtcm::receiveData(uint8_t *pData, int len)
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

void rtcmSend(void)
{
	uint8_t RtcmAvailableBuffer[RTCM_MAX_PAYLOAD];
	
	availableLength = 0;
	
	availableLength = rxBuffer.available();
	availableLength = rxBuffer.peekbytes(ubloxAvailableBuffer, availableLength);
}

