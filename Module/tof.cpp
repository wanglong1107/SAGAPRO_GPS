#include "tof.h"
#include <stdlib.h>
#include <string.h>

extern HAL_UART adsp;

//set TOF baudrate
void Tof::tofInit(uint32_t _baud)
{
	int check = 0;
	if(_baud == 9600)
	{
		_baud = 0x01;
	}
	else if(_baud == 115200)
	{
		_baud = 0x02;
	}
	else if(_baud == 230400)
	{
		_baud = 0x03;
	}
	else if(_baud == 921600)
	{
		_baud = 0x04;
	}
	
	uint8_t sendBuffer[10];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	sendBuffer[cnt++] = 0x56;
	sendBuffer[cnt++] = (0x10 | _baud);
	sendBuffer[cnt++] = 0x00;
	
	for(uint8_t i=0; i<cnt; i++)
	{
		check ^= sendBuffer[i];
	}
	
	sendBuffer[cnt++] = check;
	
	dev->writeData(sendBuffer, cnt);
	
	vTaskDelay(TOF_WAIT_MS);
}

//set measuring speed
void Tof::tofSetRate(unsigned short int _ms)
{
	int check = 0;
	if(_ms == 100)
	{
		_ms = 0x01;
	}
	else if(_ms == 50)
	{
		_ms = 0x02;
	}
	else if(_ms == 20)
	{
		_ms = 0x03;
	}
	else if(_ms == 10)
	{
		_ms = 0x04;
	}
	else if(_ms == 5)
	{
		_ms = 0x05;
	}
	
	uint8_t sendBuffer[4];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	sendBuffer[cnt++] = 0x56;
	sendBuffer[cnt++] = (0x30 | _ms);
	sendBuffer[cnt++] = 0x00;
	
	for(uint8_t i=0; i<cnt; i++)
	{
		check ^= sendBuffer[i];
	}
	
	sendBuffer[cnt++] = check;
	
	dev->writeData(sendBuffer, cnt);
	
	vTaskDelay(TOF_WAIT_MS);
}

//disabled measuring
void Tof::tofSetDisInit()
{
	int check = 0;
	uint8_t sendBuffer[4];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	sendBuffer[cnt++] = 0x56;
	sendBuffer[cnt++] = 0x06;
	sendBuffer[cnt++] = 0x00;
	
	for(uint8_t i=0; i<cnt; i++)
	{
		check ^= sendBuffer[i];
	}
	
	sendBuffer[cnt++] = check;
	
	dev->writeData(sendBuffer, cnt);
	
	vTaskDelay(TOF_WAIT_MS);
}

//get tofPollFrameVersion
void Tof::tofPollFrameVersion(void)
{
	int check = 0;
	uint8_t sendBuffer[4];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	sendBuffer[cnt++] = 0x56;
	sendBuffer[cnt++] = 0x0e;
	sendBuffer[cnt++] = 0x00;
	
	for(uint8_t i=0; i<cnt; i++)
	{
		check ^= sendBuffer[i];
	}
	
	sendBuffer[cnt++] = check;
	
	dev->writeData(sendBuffer, cnt);
	
	vTaskDelay(TOF_WAIT_MS);
}

//get HardwareVersion
void Tof::tofPollHardwareVersion(void)
{
	int check = 0;
	uint8_t sendBuffer[4];
	uint8_t cnt = 0;
	memset( sendBuffer, 0, sizeof(sendBuffer) );
	
	sendBuffer[cnt++] = 0x56;
	sendBuffer[cnt++] = 0x08;
	sendBuffer[cnt++] = 0x00;
	
	for(uint8_t i=0; i<cnt; i++)
	{
		check ^= sendBuffer[i];
	}
	
	sendBuffer[cnt++] = check;
	
	dev->writeData(sendBuffer, cnt);
	
	vTaskDelay(TOF_WAIT_MS);
}

//receive TOF data 
int Tof::receiveData(uint8_t *pData, int len)
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

//parse TOF data
void Tof::parseData(void)
{
	uint8_t tofAvailableBuffer[TOF_MAX_PAYLOAD];
	uint8_t headIndexBuffer[TOF_MAX_PAYLOAD/8];
	
	availableLength = 0;
	index = 0;
	headCnt = 0;
	payloadLength = 0;
	uint8_t receiveCheck = 0;
	
	availableLength = rxBuffer.available();
	if(availableLength > TOF_MAX_PAYLOAD -2)
	{
		rxBuffer.clear();
		return;
	}
	availableLength = rxBuffer.peekbytes(tofAvailableBuffer, availableLength);
	if( availableLength > 3 )
	{
		for(index=0; index<(availableLength-1); index++)
		{
			if(tofAvailableBuffer[index] == 0x89)
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
				receiveCheck = 0;
				for(int i=0; i < (headIndexBuffer[k+1]-1); i++)
				{
					receiveCheck ^= tofAvailableBuffer[headIndexBuffer[k]+i];
				}
				
				if(receiveCheck != tofAvailableBuffer[headIndexBuffer[k+1]-1])
				{
					frameCheckError ++;
					continue;
				}
				else
				{
					frameCheckOK ++;
				}
				
				tofData.command = tofAvailableBuffer[headIndexBuffer[k]+1];	
				
				if(tofData.command == 0x81)
				{
					payloadLength = 4;
				}
				else if(tofData.command == 0x0e)
				{
					payloadLength = 3;
				}
				else if(tofData.command == 0x08)
				{
					payloadLength = 3;
				}
				
				memcpy( &tofData.payload, tofAvailableBuffer+headIndexBuffer[k]+2, payloadLength );
				tofPublish();
			}
		}
	}
}

unsigned char Tof::tofPublish(void)
{
	unsigned char ret = 0;
	if(tofData.command == 0x81)
	{
		// tof distance update
		tofData.lastDist = tofData.payload.tofMesurement.dist;

		tofData.deltaDist = tofData.payload.tofMesurement.dist - tofData.dist_int;	
		
		tofData.dist_int = tofData.payload.tofMesurement.dist;					// mm
		
		tofData.dist = tofData.payload.tofMesurement.dist * 0.1f;				// mm => cm
		tofOutputData.dist = tofData.dist;
		
		tofData.amp = tofData.payload.tofMesurement.amp;
		tofOutputData.amp = tofData.amp;		
    
		ret = 1;
		
		if(tofData.deltaDist < 500 )
		{
			tofData.dist_updateFlag = 1;
			tofOutputData.dist_updateFlag = tofData.dist_updateFlag;
		}
	}
	else if(tofData.command == 0x0e)
	{
		tofData.firmwareReserve = tofData.payload.tofFirmware.firmwareReserve;
		tofOutputData.firmwareReserve = tofData.firmwareReserve;
		
		tofData.firmwareMainVersion = tofData.payload.tofFirmware.firmwareMainVersion;
		tofOutputData.firmwareMainVersion = tofData.firmwareMainVersion;
		
		tofData.firmwareSecondaryVersion = tofData.payload.tofFirmware.firmwareSecondaryVersion;
		tofOutputData.firmwareSecondaryVersion = tofData.firmwareSecondaryVersion;
		
		tofData.firmwareVersion_updateFlag = 1;
		tofOutputData.firmwareVersion_updateFlag = tofData.firmwareVersion_updateFlag;
	}
	else if(tofData.command == 0x08)
	{
		tofData.hardwareReserve = tofData.payload.tofHardware.hardwareReserve;
		tofOutputData.hardwareReserve = tofData.hardwareReserve;
		
		tofData.hardwareMainVersion = tofData.payload.tofHardware.hardwareMainVersion;
		tofOutputData.hardwareMainVersion = tofData.hardwareMainVersion;
		
		tofData.hardwareSecondaryVersion = tofData.payload.tofHardware.hardwareSecondaryVersion;
		tofOutputData.hardwareSecondaryVersion = tofData.hardwareSecondaryVersion;
		
		tofData.hardwareVersion_updateFlag = 1;
		tofOutputData.hardwareVersion_updateFlag = tofData.hardwareVersion_updateFlag;
	}
    return ret;
}

void Tof::sendTofData(void)
{
	unsigned char verify;
	uint8_t SendTofData[19];
	memset( SendTofData, 0, sizeof(SendTofData) );
	
	SendTofData[0] = 0x0B;
	SendTofData[1] = 19;	//19
	SendTofData[2] = 0x50;
	float2buf((char *)(SendTofData+3), &tofOutputData.dist);
	
	SendTofData[7] = (uint8_t)((int)tofOutputData.amp&0x000000ff);
	SendTofData[8] = (uint8_t)(((int)tofOutputData.amp>>8)&0x000000ff);

	SendTofData[9] = tofOutputData.firmwareReserve;
	SendTofData[10] = tofOutputData.firmwareMainVersion;
	SendTofData[11] = tofOutputData.firmwareSecondaryVersion;
	SendTofData[12] = tofOutputData.hardwareReserve;
	SendTofData[13] = tofOutputData.hardwareMainVersion;
	
	SendTofData[14] = tofOutputData.hardwareSecondaryVersion;
	SendTofData[15] = tofOutputData.dist_updateFlag;
	SendTofData[16] = tofOutputData.firmwareVersion_updateFlag;
	SendTofData[17] = tofOutputData.hardwareVersion_updateFlag;
	
	verify = 0;
	
	for(int i=0; i<18; i++)
	{
		verify ^= SendTofData[i];
	}
	SendTofData[18] = verify;
	
	adsp.writeData( SendTofData, 19);
	
	vTaskDelay(10);
}