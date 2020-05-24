#pragma once

#include <stdint.h>

#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"

class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void     init() = 0;
    virtual void     delay(uint32_t ms) = 0;

    virtual void     delay_microseconds(uint32_t us) = 0;


private:


};
