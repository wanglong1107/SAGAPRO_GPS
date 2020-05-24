#include "drv_hrt.h"
#include <compiler.h>

volatile hrt_abstime abstime_s = 0;
volatile hrt_abstime abstime_ms = 0;
volatile hrt_abstime abstime_us = 0;



/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime hrt_absolute_time(void)
{
	
	uint32_t count = 0;
//	count = ((TcChannel*)TC3)->TC_CV; 
	abstime_us = abstime_ms * 1E3 + count*5.33333333E-2f;
	
	return abstime_us;
}


/**
 * Compare a time value with the current time.
 */
hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then)
{
	hrt_abstime delta = hrt_absolute_time() - *then;
	
	return delta;
}
