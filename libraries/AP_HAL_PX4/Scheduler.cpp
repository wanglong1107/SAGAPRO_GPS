#include "Scheduler.h"

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4Scheduler::PX4Scheduler()
{
}

void PX4Scheduler::init()
{
}

void PX4Scheduler::delay_microseconds(uint32_t usec)
{
	uint32_t delayStart = AP_HAL::micros();
	uint32_t waitTime = usec;

	#define HAL_MAX_DELAY 0xFFFFFFFFU
	/* Add a freq to guarantee minimum wait */
	if (waitTime < HAL_MAX_DELAY)
	{
		waitTime += 1;
	}

	while( (AP_HAL::micros() - delayStart) < waitTime )
	{
	}
}


void PX4Scheduler::delay(uint32_t ms)
{
	OS_ERR  os_err;
	
	uint16_t ss = 1000;
	uint16_t mi = ss * 60;
	uint16_t hh = mi * 60;

	uint16_t hour = ms / hh;
	uint16_t minute = (ms - hour * hh) / mi;
	uint16_t second = (ms - hour * hh - minute * mi) / ss;
	uint32_t milliSecond = ms - hour * hh - minute * mi - second * ss;
	
	OSTimeDlyHMSM( hour, minute, second, milliSecond, OS_OPT_TIME_HMSM_STRICT, &os_err);
}

//int PX4Scheduler::CreateTask( void *func, void *para )
//{
//	OS_ERR   os_err;
//	
//	OSTaskCreate(&initializeTaskTCB,
//                 (CPU_CHAR *)" initialize task",
//				 ( OS_TASK_PTR )func,
//				 para,
//				 APP_INIT_TASK_PRIO,
//				 &initializeTaskStk[0],
//				 initializeTaskStk[APP_INIT_TASK_STK_SIZE / 10u],
//				 APP_INIT_TASK_STK_SIZE,
//				 0u,
//				 0u,
//				 0,
//				 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//				 &os_err);
//	if( os_err != OS_ERR_NONE )
//	{
//		return -1;
//	}
//	return 0;
//}
