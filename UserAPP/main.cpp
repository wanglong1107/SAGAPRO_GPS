#include <atmel_start.h>
#include "hpl_dma.h"

#include "hal_delay.h"
#include "hpl_dma.h"
#include "jump.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <hal_rtos.h>
#include "rtos_start.h"

#include "driver_examples.h"

#include "hal_drivers/hal_uart.h"
#include "hal_drivers/hal_i2c.h"
#include "hal_drivers/hal_can.h"
#include <Module/ublox_F9P.h>
#include <Module/qmc5883.h>
#include "jump.h"
#define INITIALIZE_TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define INITIALIZE_TASK_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
TaskHandle_t initializeTaskHandle1;
void initializeTask(void *p);

#define GNSS1_PARSE_TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define GNSS1_PARSE_TASK_STACK_PRIORITY (tskIDLE_PRIORITY + 2)
TaskHandle_t gnssTaskHandle1;
void GnssReceiveTask(void *p);
void UartToUblox1ISR(const struct usart_async_descriptor *const io_descr);

#define GNSS2_PARSE_TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define GNSS2_PARSE_TASK_STACK_PRIORITY (tskIDLE_PRIORITY + 2)
TaskHandle_t gnssTaskHandle2;
void GnssReceiveTask2(void *p);
void UartToUblox2ISR(const struct usart_async_descriptor *const io_descr);

#define qmc5883_PARSE_TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define qmc5883_PARSE_TASK_STACK_PRIORITY (tskIDLE_PRIORITY + 2)
TaskHandle_t qmc5883TaskHandle;
void QMC5883ReadTask(void *p);

HAL_UART UartToUblox1(&USART_5);			//第一个F9P的串口1，单片机默认波特率38400
Ublox Ublox1(&UartToUblox1);

HAL_UART UartToUblox2(&USART_3);			//第一个F9P的串口2，单片机默认波特率38400
Ublox Ublox2(&UartToUblox2);

HAL_UART UartToFC(&USART_4);			//与飞控通信的串口，单片机默认波特率115200
Ublox FCDataCon(&UartToFC);
void UartToFCISR(const struct usart_async_descriptor *const io_descr);

HAL_I2C I2cToQMC5883( &I2C_0, 0x0D);
QMC5883 qmc5883Dev(&I2cToQMC5883);//&I2cToQMC5883

int DataSend(uint8_t i);

HAL_CAN CanToFC(&CAN_0);

uint8_t sendData[6][128];
uint8_t sendNmeaData[2][128];
uint8_t sendDataFlg[6];
uint8_t sendNmeaDataFlg[2];
uint8_t D_RTK = 0;
uint8_t channel=3;//榛樿3锛氶潪閫忎紶宸ヤ綔妯″紡
uint32_t nmeaLength[2];

#define HARDWARE_VERSION 0x0F
#define FIRMWARE_VERSION_YEAR 20
#define FIRMWARE_VERSION_MONTH 04
#define FIRMWARE_VERSION_DAY 12
uint16_t numA = 0;
uint16_t numB = 0;
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	xTaskCreate(
				initializeTask, 
				"initialize task", 
				INITIALIZE_TASK_STACK_SIZE, 
				NULL, 
				INITIALIZE_TASK_STACK_PRIORITY, 
				&initializeTaskHandle1
				);
	
	vTaskStartScheduler();

	while (1) 
	{
	}
}

void initializeTask(void *p)
{
	UartToFC.initialize();
	UartToFC.setBaudRate(115200);													//设置单片机波特率115200
	UartToFC.register_isr( USART_ASYNC_RXC_CB, UartToFCISR );					//注册连接飞机的串口中断函数
	
	delay_ms(1000);
	
	Ublox1.dev->initialize();
	Ublox1.dev->setBaudRate(38400);												//设置单片机波特率为38400，F9P默认波特率为38400
	Ublox1.ConfigBuadrate(UART1_BAUDRATE, SAVE_TO_RAM, 115200);					//设置F9P-1的串口1波特率为115200
	Ublox1.dev->setBaudRate(115200);												//设置单片机波特率为115200
	Ublox1.ConfigBuadrate(UART2_BAUDRATE, SAVE_TO_RAM, 115200);					//设置F9P-1的串口2波特率为115200
	Ublox1.ubloxInit();															//关闭F9P-1串口1的NMEA和RTCM输出，只输出UBX
	Ublox1.ConfigGnssSystem();													//设置星座
	Ublox1.ConfigRate(RATE_MEAS, SAVE_TO_RAM, 100);//200	 							//设置导航数据更新速率5Hz
	Ublox1.ConfigOutputMessage(MSGOUT_UBX_NAV_DOP_UART1, SAVE_TO_RAM, 10);//5		//开启F9P-1串口1的NAV-DOP协议，导航数据更新5次，当前协议更新1次	
	Ublox1.ConfigOutputMessage(MSGOUT_UBX_NAV_PVT_UART1, SAVE_TO_RAM, 1);		//开启F9P-1串口1的NAV-PVT协议，导航数据更新1次，当前协议更新1次
	Ublox1.ConfigRtcmOutputMesseage();											//设置F9P-1串口1输出RTCM协议，用于F9P-2解算航向
	Ublox1.dev->register_isr( USART_ASYNC_RXC_CB, UartToUblox1ISR );				//注册F9P-1串口1的中断函数
	
	delay_ms(50);

	Ublox2.dev->initialize();
	Ublox2.dev->setBaudRate(38400);												//设置单片机波特率为38400，F9P默认波特率为38400
	Ublox2.ConfigBuadrate(UART1_BAUDRATE, SAVE_TO_RAM, 115200);					//设置F9P-1的串口1波特率为115200
	Ublox2.dev->setBaudRate(115200);												//设置单片机波特率为115200
	Ublox2.ConfigBuadrate(UART2_BAUDRATE, SAVE_TO_RAM, 115200);					//设置F9P-1的串口2波特率为115200
	Ublox2.ConfigOutputMessage(UART1OUTPROT_NMEA, SAVE_TO_RAM, 0);				//关闭F9P-2串口1的NMEA协议输出
	Ublox2.ConfigOutputMessage(UART1OUTPROT_UBX, SAVE_TO_RAM, 1);				//开启F9P-2串口1的UBX协议输出
	Ublox2.ConfigOutputMessage(UART1OUTPROT_RTCM3X, SAVE_TO_RAM, 0);				//关闭F9P-2串口1的RTCM协议输出
	Ublox2.ConfigGnssSystem();													//设置星座
	Ublox2.ConfigRate(RATE_MEAS, SAVE_TO_RAM, 100);								//设置导航数据更新速率5Hz
	Ublox2.ConfigOutputMessage(MSGOUT_UBX_NAV_RELPOSNED_UART1, SAVE_TO_RAM, 1);		//开启F9P-1串口1的NAV-POSLLH协议，导航数据更新1次，当前协议更新1次	
	Ublox2.ConfigOutputMessage(MSGOUT_UBX_NAV_PVT_UART1, SAVE_TO_RAM, 10);		//开启F9P-1串口1的NAV-PVT协议，导航数据更新1次，当前协议更新1次
	Ublox2.dev->register_isr( USART_ASYNC_RXC_CB, UartToUblox2ISR );				//注册F9P-1串口1的中断函数
	
	qmc5883Dev.initialize();

	xTaskCreate(
				GnssReceiveTask, 
				"f9p1 receive task", 
				GNSS1_PARSE_TASK_STACK_SIZE, 
				NULL, 
				GNSS1_PARSE_TASK_STACK_PRIORITY, 
				&gnssTaskHandle1
				);
	
	xTaskCreate(
				GnssReceiveTask2, 
				"f9p2 receive task", 
				GNSS2_PARSE_TASK_STACK_SIZE, 
				NULL, 
				GNSS2_PARSE_TASK_STACK_PRIORITY, 
				&gnssTaskHandle2
				);
				
	xTaskCreate(
				QMC5883ReadTask, 
				"qmc5883 read task", 
				qmc5883_PARSE_TASK_STACK_SIZE, 
				NULL, 
				GNSS1_PARSE_TASK_STACK_PRIORITY, 
				&qmc5883TaskHandle
				);		
						
	vTaskDelete(initializeTaskHandle1);   
	
}

void GnssReceiveTask(void *p)
{
	static portTickType xLastWakeTime;
	int ret = 0;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{	
		if(channel==3)
		{
			Ublox1.parseData();	
			ret = DataSend(0);	
		}
		vTaskDelayUntil(&xLastWakeTime,10);
	}
}

void GnssReceiveTask2(void *p)
{
	static portTickType xLastWakeTime;
	int ret = 0;
	xLastWakeTime = xTaskGetTickCount();
	static uint8_t time =0;
	uint8_t GNSS2_State[14]={0xB5,0x62,0x11,0x03,0x06,0x00};
	uint8_t CK_A=0, CK_B=0;
	while (1)
	{	
		if(channel==3)
		{
			if(time>=5)
			{
				CK_A=0, CK_B=0;
				time = 0;
				GNSS2_State[6]  = Ublox2.gpsData.numSV;//0x0A
				GNSS2_State[7]  = Ublox2.gpsData.fixType;
				/*************************add***************************/
				GNSS2_State[8]  = Ublox2.gpsData.carrSoln;
				GNSS2_State[9]  = D_RTK;
				GNSS2_State[10] = qmc5883Dev.magInitialFailFlag;
				GNSS2_State[11] = 0;
				/*************************add***************************/
				for(uint8_t i=2;i<12;i++)
				{
					CK_A = CK_A + GNSS2_State[i];
					CK_B = CK_B + CK_A;
				}
				GNSS2_State[12] = CK_A;
				GNSS2_State[13] = CK_B;
				for(int j=5;j>=0;j--)
				{
					if(sendDataFlg[j]==0)
					{
						sendDataFlg[j]=0x01;
						memcpy( sendData[j], GNSS2_State, 14 );
						break;
					}
				}
			}
			time++;
			Ublox2.parseData();		
		}
		vTaskDelayUntil(&xLastWakeTime,200);
	}
}

int DataSend(uint8_t i)
{
	uint16_t len;
	uint8_t j = i;
	for(j = 0;j <= 5;j++)
	{
		if(sendDataFlg[j] != 0x00)
		{
			if(sendData[j][2]!=0x06)// data send to FC
			{
				if(sendData[j][2] == 0x01)
				{
					numB ++;
				}
				len=(uint16_t)(sendData[j][4])+(uint16_t)(sendData[j][5]<<8);
				UartToFC.writeData(sendData[j], len+8);
				sendDataFlg[j] = 0x00;
				delay_ms(2);
			}
			else                    // data send to Ublox1
			{
				len=(uint16_t)(sendData[j][4])+(uint16_t)(sendData[j][5]<<8);
				UartToUblox1.writeData(sendData[j], len+8);
				sendDataFlg[j] = 0x00;
			}
		}
	}
	
	for(uint8_t k=0;k<2;k++)
	{
		if(sendNmeaDataFlg[k] != 0)
		{
			sendNmeaDataFlg[k] = 0;
			UartToFC.writeData(sendNmeaData[k], nmeaLength[k]);
		}
	}
	return 0;
}
void QMC5883ReadTask(void *p)
{
	static portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static uint8_t time =0;
	uint8_t magState[16]={0xB5,0x62,0x11,0x02,0x06,0x00};
	uint8_t CK_A, CK_B;
	while (1)
	{	
		if(time>=20)
		{
			CK_A=0, CK_B=0;
			time = 0;
			magState[6] =(uint8_t)(0x00FF&qmc5883Dev.magErrorRestTime);
			magState[7] =(uint8_t)((0xFF00&qmc5883Dev.magErrorRestTime)>>8);
			/*************************add***************************/
			magState[8] = HARDWARE_VERSION;
			magState[9] = FIRMWARE_VERSION_YEAR;
			magState[10] = FIRMWARE_VERSION_MONTH;
			magState[11] = FIRMWARE_VERSION_DAY;
			/*************************add***************************/
			for(uint8_t i=2;i<12;i++)
			{
				CK_A = CK_A + magState[i];
				CK_B = CK_B + CK_A;
			}
			magState[12] = CK_A;
			magState[13] = CK_B;
			for(int j=5;j>=0;j--)
			{
				if(sendDataFlg[j]==0)
				{
					sendDataFlg[j]=0x01;
					memcpy( sendData[j], magState, 14);
					break;
				}
			}
		}
		time++;
//		qmc5883Dev.update();	
		vTaskDelayUntil(&xLastWakeTime,50);
	}
}

void UartToUblox1ISR(const struct usart_async_descriptor *const io_descr)
{
	uint8_t tempData[1];
	int ret = 0;
	uint8_t availableLength = 0;
	
	ret = Ublox1.dev->readData( tempData, 1);
	if(channel==1)
	{
		UartToFC.writeData(tempData, 1);
	}
	else if(ret>0)
	{
		Ublox1.receiveData( tempData, ret);
	}	
}

void UartToUblox2ISR(const struct usart_async_descriptor *const io_descr)
{
	uint8_t tempData[1];
	int ret = 0;
	uint8_t availableLength = 0;
	
	ret = Ublox2.dev->readData( tempData, 1);
	if(channel==2)
	{
		UartToFC.writeData(tempData, 1);
	}
	else if(ret>0)
	{
		Ublox2.receiveData( tempData, ret);
	}	
}

void channelCheck(uint8_t *tempData)
{
	uint8_t tempData0[5]={0x11,0x11,0x11,0x11,0x11};
	uint8_t tempData1[5]={0x22,0x22,0x22,0x22,0x22};
	uint8_t tempData2[5]={0x33,0x33,0x33,0x33,0x33};
	static uint8_t num=0;
	switch (num)
	{	//-----閫忎紶1---------閫忎紶2------------宸ヤ綔-----
		//-03 01 31 00 E3-03 01 32 00 A5-03 01 33 00 B3-
		case 0: if(*tempData==0x03) num=1;
				else num=0;        break;
		case 1: if(*tempData==0x01) num=2;
				else num=0;			  break;
		case 2:  if(*tempData==0x31) num=3;
				else if(*tempData==0x32) num=6;
				else if(*tempData==0x33) num=9;
				else num=0;				break;
		case 3: if(*tempData==0x00) num=4;
				else num=0;			  break;
		case 4: if(*tempData==0xE3) num=5;
				else num=0;			  break;
		case 6: if(*tempData==0x00) num=7;
				else num=0;			  break;
		case 7: if(*tempData==0xA5) num=8;
				else num=0;			  break;
		case 9: if(*tempData==0x00) num=10;
				else num=0;			  break;
		case 10: if(*tempData==0xB3) num=11;
				else num=0;			  break;
		default: num=0;			  break;
	}
	if(num==5)
	{
		num=0;
		channel=1;
		UartToFC.writeData(tempData0, 5);
	}
	else if(num==8)
	{
		num=0;
		channel=2;
		UartToFC.writeData(tempData1, 5);
	}
	else if(num==11)
	{
		num=0;
		channel=3;
		UartToFC.writeData(tempData2, 5);
	}
	else;
	
}

void UartToFCISR(const struct usart_async_descriptor *const io_descr)
{
	int ret = 0;
	uint8_t tempData[16];
	uint8_t availableLength = 0;
	uint8_t flg = 0;
	ret = FCDataCon.dev->readData( tempData, 1);
	channelCheck(tempData);
	if(channel==1)
	{
		Ublox1.dev->writeData( tempData,1);
	}
	else if(channel==2)
	{
		Ublox2.dev->writeData( tempData,1);
	}
	
	flg = JumpFun(tempData);
	if(flg == 1)
	{
		__disable_irq();
		NVIC_SystemReset();
	}
	
	if(ret>0)
	{
		FCDataCon.receiveData( tempData, ret);
		
		availableLength = FCDataCon.rxBuffer.available();	
	}
	
}




