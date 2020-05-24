#pragma once

#include "AP_HAL_Namespace.h"

#define HAL_SEMAPHORE_BLOCK_FOREVER 0

class AP_HAL::Semaphore {
public:
	Semaphore(void) {}

    virtual bool take(uint32_t timeout_ms) = 0 ;
    virtual bool take_nonblocking() = 0;
    virtual void take_blocking() { take(HAL_SEMAPHORE_BLOCK_FOREVER); };
    
    virtual bool give() = 0;
    
};
