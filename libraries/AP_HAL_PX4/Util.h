#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"
#include "Semaphores.h"

class PX4::PX4Util : public AP_HAL::Util {
public:
    PX4Util(void);

    /*
      get system identifier (STM32 serial number)
     */
    bool get_system_id(char buf[40]) override;

    uint32_t available_memory(void) override;


	AP_HAL::Semaphore *new_semaphore(void) override { return new PX4::PX4Semaphore(); }

//    // allocate and free DMA-capable memory if possible. Otherwise return normal memory
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type) override;

private:

    struct {
        int8_t *target;
        float integrator;
        uint16_t count;
        float sum;
        uint32_t last_update_ms;
        int fd = -1;
    } _heater;
};
