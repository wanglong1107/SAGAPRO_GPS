#ifndef _HAL_TC_H
#define _HAL_TC_H

#include <stdint.h>
#include "hal_timer.h"

class HAL_TC
{
private:
	struct timer_descriptor *hw;

	struct timer_task task;

public:
	bool initialFlag;

	HAL_TC( void *const _hw)
	{
		hw = (struct timer_descriptor *)_hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	
	void start(void);
	void stop(void);
	
	int register_isrtimer_add_task(timer_cb_t cb);

	
};

#endif //_HAL_TC_H
