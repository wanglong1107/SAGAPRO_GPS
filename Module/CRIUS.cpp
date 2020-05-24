
#include "FreeRTOS.h"
//#include "queue.h"
//#include "semphr.h"
//#include "task.h"

#include "attitudeControlTask.h"

#include "globalVariables.h"
#include "CRIUS.h"
#include <string.h>


//void static parse_PIDpara(uint8_t* _buffer, pid_t*  _para_pos, pid_t*  _para_rate)
//{
//	_para_rate->pidGain.Kp = (float)(_buffer[4]<<8 | _buffer[5]) * 0.001f;
//	
//	_para_rate->pidGain.Ki = (float)(_buffer[6]<<8 | _buffer[7]) * 0.001f;
//	_para_rate->pidGain.Kd = (float)(_buffer[8]<<8 | _buffer[9]) * 0.001f;
//	_para_pos->pidGain.Kp = (float)(_buffer[10]<<8 | _buffer[11]) * 0.001f;
//	_para_pos->pidGain.Kp = (float)(_buffer[12]<<8 | _buffer[13]) * 0.001f;
//	_para_pos->pidGain.Kp = (float)(_buffer[14]<<8 | _buffer[15]) * 0.001f;
//	
//	//_para->rateDamp = (float)(_buffer[16]<<8 | _buffer[17]) * 0.001f;
//	
//}

void CRIUS::CRIUS_SendData( uint8_t* dataBuffer, int len)
{

	UART_SendBuffer( UART, dataBuffer, len );
	//  USART_SendBuffer( USART, dataBuffer, len );
}

void CRIUS::CRIUS_Config()
{
	PIO_Configure( &pin, PIO_LISTSIZE( pin ) );
	PMC_EnablePeripheral(id);
	UART_Configure(UART, UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL, 57600, 150000000); 
	
	NVIC_ClearPendingIRQ(irq);
	NVIC_SetPriority( irq ,1);
	
	/* Enables the UART to transfer and receive data. */
	UART_SetTransmitterEnabled ( UART , 1);
	UART_SetReceiverEnabled ( UART , 1);
	
	UART_EnableIt(UART, (UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME 
			| UART_IER_PARE));
	/* Enable interrupt  */ 
	NVIC_EnableIRQ(irq);
	
}
void CRIUS::CRIUS_ParseXbee()
{
	uint8_t temp[23] = {0};
	uint8_t checksum = 0;
//	uint8_t checksum_ack = 0;
	
	while( 0xAA != xbee_Queue.pBase[xbee_Queue.front])
	{
		if(!xbee_Queue.DeQueue(1))
		{
			return;
		}
	}
	
	if( !xbee_Queue.ReadQueue(temp, 23) )
	{
		return;
	}
	
	for(int i = 0; i<22; i++)
	{
		checksum += temp[i];
	}
	
	
//	if(checksum == temp[22])
//	{
//		switch(temp[21])
//		{
//			case 0x01: parse_PIDpara(temp, &roll_pos_PIDpara, &roll_rate_PIDpara); break;
//			case 0x02: parse_PIDpara(temp, &pitch_pos_PIDpara, &pitch_rate_PIDpara); break;
//			case 0x03: parse_PIDpara(temp, &yaw_pos_PIDpara, &yaw_rate_PIDpara); break;	
//			default: break;
//		}
////		temp[0] = 0x88;
////		temp[1] = 0x88;
////		for(int i = 0; i<22; i++)
////		{
////			checksum_ack += temp[i];
////		}
////		temp[22] = checksum_ack;
////		CRIUS_SendData(temp, 23);
//	}
}
