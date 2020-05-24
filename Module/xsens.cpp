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


#include "xsens.h"
#include <stdlib.h>
#include <string.h>

int Xsens::initialize(void)
{
	dev->initialize();
	
	return 0;
}

int Xsens::receiveData(uint8_t *pData, int len)
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



int Xsens::parseData(void)
{
	uint8_t xsensAvailableBuffer[XSENS_MAX_PAYLOAD];
	uint32_t headIndexBuffer[XSENS_MAX_PAYLOAD/8];
	
	availableLength = 0;
	index = 0;
	headCnt = 0;
	payloadLength = 0;
	
	char crc;
	
	
	availableLength = rxBuffer.available();
	if(availableLength > XSENS_MAX_PAYLOAD)
	{
		rxBuffer.clear();
		return -1;
	}
	availableLength = rxBuffer.peekbytes(xsensAvailableBuffer, availableLength);
	if( availableLength>8 )
	{
		for(index=0; index<(availableLength-1); index++)
		{
			if( (xsensAvailableBuffer[index] == 0xfa)&&(xsensAvailableBuffer[index+1] == 0xff) )
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
				crc = 0;
				for(int i=1; i<MTI_700_DATA_SIZE; i++)
				{
					crc += xsensAvailableBuffer[headIndexBuffer[k]+i];
				}
				if(crc) 
				{
					ahrs.crcErrorCnt++;
					return -1;     //校验失败
				}
				GetEulerangle(xsensAvailableBuffer+headIndexBuffer[k], 7);
				GetAcceleration(xsensAvailableBuffer+headIndexBuffer[k], 22);
				GetGyroscope(xsensAvailableBuffer+headIndexBuffer[k], 37);
//				ubloxData.classId = availableBuffer[headIndexBuffer[k]+2];
//				ubloxData.id = availableBuffer[headIndexBuffer[k]+3];
//				payloadLength = (availableBuffer[headIndexBuffer[k]+5]<<8) | (availableBuffer[headIndexBuffer[k]+4]);
//				if(payloadLength > XSENS_MAX_PAYLOAD)
//				{
//					rxBuffer.clear();
//					return;
//				}
//				memcpy( &ubloxData.payload, availableBuffer+headIndexBuffer[k]+6, payloadLength );
				//ubloxPublish();
				return 0;
			}
		}
	}
	return -2;
}

void Xsens::GetEulerangle(uint8_t *pData, int index)
{
	int tmp;
	/*roll*/
	tmp = (pData[index+0] << 24) + (pData[index+1] << 16) +
		  (pData[index+2] << 8 ) + (pData[index+3] << 0 );
	ahrs.pitch = *(float*)(&tmp);
	
	/*pitch*/
	tmp = (pData[index+4] << 24) + (pData[index+5] << 16) +
		  (pData[index+6] << 8 ) + (pData[index+7] << 0 );
	ahrs.roll = *(float*)(&tmp);
	
	/*yaw*/
	tmp = (pData[index+8] << 24) + (pData[index+9] << 16) +
		  (pData[index+10]<< 8 ) + (pData[index+11]<< 0 );
	ahrs.yaw = -*(float *)(&tmp);

}

void Xsens::GetAcceleration(uint8_t *pData, int index)
{
	int tmp;
	
	/*xAcc*/
	tmp = (pData[index+0] << 24) + (pData[index+1] << 16) +
		  (pData[index+2] << 8 ) + (pData[index+3] << 0 );
	ahrs.xAcc = *(float*)(&tmp);
	
	/*yAcc*/
	tmp = (pData[index+4] << 24) + (pData[index+5] << 16) +
		  (pData[index+6] << 8 ) + (pData[index+7] << 0 );
	ahrs.yAcc = -*(float*)(&tmp);
	
	/*zAcc*/
	tmp = (pData[index+8] << 24) + (pData[index+9] << 16) +
		  (pData[index+10]<< 8 ) + (pData[index+11]<< 0 );
	ahrs.zAcc = -*(float*)(&tmp);
}

void Xsens::GetGyroscope(uint8_t *pData, int index)
{
	int tmp;
	
	/*xAcc*/
	tmp = (pData[index+0] << 24) + (pData[index+1] << 16) +
		  (pData[index+2] << 8 ) + (pData[index+3] << 0 );
	ahrs.xGyr = *(float*)(&tmp) * Rad2Deg;
	
	/*yAcc*/
	tmp = (pData[index+4] << 24) + (pData[index+5] << 16) +
		  (pData[index+6] << 8 ) + (pData[index+7] << 0 );
	ahrs.yGyr = -*(float*)(&tmp) * Rad2Deg;

	/*zAcc*/
	tmp = (pData[index+8] << 24) + (pData[index+9] << 16) +
		  (pData[index+10]<< 8 ) + (pData[index+11]<< 0 ); 
	ahrs.zGyr = -*(float*)(&tmp) * Rad2Deg;
}

void Xsens::GetXYZ(uint8_t *pData, int index)
{
	int tmp;
	
	/*xAcc*/
	tmp = (pData[index+0] << 24) + (pData[index+1] << 16) +
		  (pData[index+2] << 8 ) + (pData[index+3] << 0 );
	ahrs.lat = *(float*)(&tmp);
	
	/*yAcc*/
	tmp = (pData[index+4] << 24) + (pData[index+5] << 16) +
		  (pData[index+6] << 8 ) + (pData[index+7] << 0 );
	ahrs.lon = *(float*)(&tmp);

	/*zAcc*/
	tmp = (pData[index+11] << 24) + (pData[index+12] << 16) +
		  (pData[index+13]<< 8 ) + (pData[index+14]<< 0 ); 
	ahrs.altitude = -*(float*)(&tmp);
}

void Xsens::GetXYZvel(uint8_t *pData, int index)
{
	int tmp;
	
	/*vx*/
	tmp = (pData[index+0] << 24) + (pData[index+1] << 16) +
		  (pData[index+2] << 8 ) + (pData[index+3] << 0 );
	ahrs.vx = *(float*)(&tmp);
	
	/*vy*/
	tmp = (pData[index+4] << 24) + (pData[index+5] << 16) +
		  (pData[index+6] << 8 ) + (pData[index+7] << 0 );
	ahrs.vy = -*(float*)(&tmp);

	/*vz*/
	tmp = (pData[index+8] << 24) + (pData[index+9] << 16) +
		  (pData[index+10]<< 8 ) + (pData[index+11]<< 0 ); 
	ahrs.vz = -*(float*)(&tmp);
}
