#include "Semaphores.h"

using namespace PX4;
extern const AP_HAL::HAL& hal;



bool PX4Semaphore::give() 
{
	OSSemPost( &lock, OS_OPT_POST_1, &err);
	if( OS_ERR_NONE != err)
	{
		return false;
	}
	return true;
	
}

bool PX4Semaphore::take(uint32_t timeout_ms) 
{
    if (OSIntNestingCtr>0) {
        // don't ever wait on a semaphore in interrupt context
        return take_nonblocking();
    }
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) 
	{
		OSSemPend( &lock, 0, OS_OPT_PEND_BLOCKING, NULL, &err );
		if( OS_ERR_NONE != err)
		{
			return false;
		}	
		return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
//        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool PX4Semaphore::take_nonblocking() 
{
	OSSemPend( &lock, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err );
	if( OS_ERR_NONE != err)
	{
		return false;
	}	
	return true;
	
}
