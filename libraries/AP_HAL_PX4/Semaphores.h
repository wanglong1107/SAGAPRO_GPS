#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"



class PX4::PX4Semaphore : public AP_HAL::Semaphore {
public:
    PX4Semaphore() 
	{
        OSSemCreate(&lock,
					(CPU_CHAR *)"Binary Semaphore",
					1,
					&err);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
	OS_ERR err;
    OS_SEM lock;
};

