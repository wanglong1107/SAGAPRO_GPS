/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ?2011-2014  Bill Nesbitt
*/


#include "sbus.h"
#include <stdlib.h>
#include <string.h>

int SBUS::initialize(void)
{
	int ret = 0;
	ret = dev->initialize();
	if(ret<0)
	{
		return -1;
	}
	
	return 0;
}

int SBUS::receiveData(uint8_t *pData, int len)
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



int SBUS::parseData(void)
{
	uint8_t sbusAvailableBuffer[SBUS_MAX_PAYLOAD];
	uint32_t headIndexBuffer[SBUS_MAX_PAYLOAD/8];
	
	availableLength = 0;
	index = 0;
	headCnt = 0;
	payloadLength = 0;
	
	uint8_t xor_crc = 0x00;
	uint32_t frameLength = 0;
	
	
	availableLength = rxBuffer.available();
	if(availableLength > SBUS_MAX_PAYLOAD)
	{
		rxBuffer.clear();
		return -1;
	}

	availableLength = rxBuffer.peekbytes(sbusAvailableBuffer, availableLength);
	if( availableLength > 5 )
	{
		for(index=0; index<(availableLength); index++)
		{
			if( sbusAvailableBuffer[index] == 0x55 )
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
				xor_crc = 0;
				
				payloadLength= sbusAvailableBuffer[headIndexBuffer[k]+1];
				frameLength = headIndexBuffer[k+1]-headIndexBuffer[k];
				
				if( payloadLength != (frameLength-4) )
				{
					continue;
				}
				
				for(int i=0; i<(frameLength-3); i++)
				{
					xor_crc ^= sbusAvailableBuffer[headIndexBuffer[k]+1+i];
				}
				if(xor_crc == sbusAvailableBuffer[headIndexBuffer[k+1]-2])
				{
					ubloxPublish( sbusAvailableBuffer+headIndexBuffer[k], frameLength);
				}
				else
				{
					frameError ++;
					return -1;     //校验失败
				}
			}
		}
		return 0;
	}
	return -2;
}

int SBUS::ubloxPublish(uint8_t *pData, int len)
{
	uint16_t tempData[16] = {0};
	
	if( pData[3] == 0x01)
	{
		sbus_ch.sbusChannel[0] = (pData[5]<<8) | pData[4];
		sbus_ch.sbusChannel[1] = (pData[7]<<8) | pData[6];
		sbus_ch.sbusChannel[2] = (pData[9]<<8) | pData[8];
		sbus_ch.sbusChannel[3] = (pData[11]<<8) | pData[10];
		
		sbus_ch.sbusChannel[4] = pData[12];
		
		
		sbus_ch.key_1 = pData[15]&0x02;
		sbus_ch.key_2 = pData[15]&0x04;
		sbus_ch.key_3 = pData[15]&0x08;
		sbus_ch.key_4 = pData[15]&0x10;
		sbus_ch.key_5 = pData[15]&0x20;
		sbus_ch.key_6 = pData[15]&0x40;
		
		
	}
	else if( pData[3] == 0x04)
	{
		sbus_ch.connectStatus = pData[4];
		if(sbus_ch.connectStatus == 3)
		{
			sbus_ch.sbusChannel[0] = 1500;
			sbus_ch.sbusChannel[1] = 1500;	
			sbus_ch.sbusChannel[2] = 1400;
			sbus_ch.sbusChannel[3] = 1500;
			
			
			sbus_ch.key_1 = 0;
			sbus_ch.key_2 = 0;
			sbus_ch.key_3 = 0;
			sbus_ch.key_4 = 0;
			sbus_ch.key_5 = 0;
			sbus_ch.key_6 = 0;
			
			
			sbus_ch.FailSafe = 1;
			sbus_ch.FrameLost = 1;
		}
		
		
	}
	else if(pData[3] == 0x85)
	{
		sbus_ch.operateMode = pData[4];
	}
	else
	{
	}
	
	return 0;
}

