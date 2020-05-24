#include "hal_tc.h"

#include <string.h>

int HAL_TC::initialize(void)
{
	if(!initialFlag)
	{
		_timer_init( &(hw->device), TC3 );
		initialFlag = true;
	}
	_timer_set_irq( &(hw->device) );
	
	return 0;
}


int HAL_TC::disInitialize(void)
{
	if(initialFlag)
	{
		timer_deinit(hw);
		initialFlag = false;
	}
	return 0;
}

void HAL_TC::start(void)
{
	timer_start(hw);
}

void HAL_TC::stop(void)
{
	timer_stop(hw);
}

int HAL_TC::register_isrtimer_add_task(timer_cb_t cb)
{
	task.interval = 1000;
	task.cb       = cb;
	task.mode     = TIMER_TASK_REPEAT;
	
	timer_add_task( hw, &task );
	
	return 0;
}
