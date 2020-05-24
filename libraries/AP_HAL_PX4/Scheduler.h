#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"

//typedef struct
//{
//	int pro;
//	
//}


class PX4::PX4Scheduler : public AP_HAL::Scheduler {
public:
    PX4Scheduler();

    void init();
    void delay(uint32_t ms);
    void delay_microseconds(uint32_t us);
//    int CreateTask( void *func, void *para );
private:
    bool _initialized;
};

