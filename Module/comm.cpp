#include "comm.h"

COMM::COMM(HAL_UART *_dev)
{
	dev = _dev;
}

int COMM::initialize(void)
{
	dev->initialize();
	
	return 0;
}


int COMM::receiveData(uint8_t *pData, int len)
{
	int ret = 0;
	uint32_t available = 0;
	uint32_t freeSpace = 0;
	
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

	
void COMM::parseData(void)
{
	uint8_t availableBuffer[COMM_MAX_PAYLOAD];
	uint32_t headIndexBuffer[COMM_MAX_PAYLOAD/8];
	uint8_t xorCheck = 0;
	
	availableLength = 0;
	index = 0;
	headCnt = 0;
	payloadLength = 0;

	
	availableLength = rxBuffer.available();
	if(availableLength > COMM_MAX_PAYLOAD-2)
	{
		rxBuffer.clear();
		return;
	}
	availableLength = rxBuffer.peekbytes(availableBuffer, availableLength);
	if( availableLength>10 )
	{
		for(index=0; index<(availableLength-1); index++)
		{
			if( (availableBuffer[index] == 0x55) && (availableBuffer[index+1] == 0x55) )
			{
				headIndexBuffer[headCnt] = index;
				headCnt++;
			}
		}
		
		if( headCnt == 0)
		{
			rxBuffer.advance(index);
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
				xorCheck = 0x00;
				for(int i=2; i < (headIndexBuffer[k+1]-headIndexBuffer[k]-2); i++)
				{
					xorCheck ^= availableBuffer[headIndexBuffer[k]+i];
				}
				
				if( xorCheck != availableBuffer[headIndexBuffer[k+1]-2] )
				{
					frameError ++;
					continue;
				}
				else
				{
					frameOK ++;
				}
				
				motor1 = (uint16_t(availableBuffer[headIndexBuffer[k]+5]))<<8 | (availableBuffer[headIndexBuffer[k]+4]);
				motor2 = (uint16_t(availableBuffer[headIndexBuffer[k]+7]))<<8 | (availableBuffer[headIndexBuffer[k]+6]);
				motor3 = (uint16_t(availableBuffer[headIndexBuffer[k]+9]))<<8 | (availableBuffer[headIndexBuffer[k]+8]);
				motor4 = (uint16_t(availableBuffer[headIndexBuffer[k]+11]))<<8 | (availableBuffer[headIndexBuffer[k]+10]);
				
				
				headLedColor = availableBuffer[headIndexBuffer[k]+12];
				tailLedColor = availableBuffer[headIndexBuffer[k]+13];
				
				pwm1 = motor1 *1.3333f + 1100.0f;
				pwm2 = motor2 *1.3333f + 1100.0f;
				pwm3 = motor3 *1.3333f + 1100.0f;
				pwm4 = motor4 *1.3333f + 1100.0f;
				
				escCtlOK = 1;
				escCnt = 0;
				
			}
		}
	}
}