#pragma once

#include <stdarg.h>
#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"
#include "Semaphores.h"


class AP_HAL::Util {
public:
    /*
      set HW RTC in UTC microseconds
     */
    virtual void set_hw_rtc(uint64_t time_utc_usec);

    /*
      get system clock in UTC microseconds
     */
    virtual uint64_t get_hw_rtc();

    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    virtual bool get_system_id(char buf[40]) { return false; }

    // create a new semaphore
    virtual Semaphore *new_semaphore(void) { return nullptr; }

	// allocate and free DMA-capable memory if possible. Otherwise return normal memory
	enum Memory_Type {
		MEM_DMA_SAFE,
		MEM_FAST
	};
	virtual void *malloc_type(size_t size, Memory_Type mem_type) { return calloc(1, size); }
	virtual void free_type(void *ptr, size_t size, Memory_Type mem_type) { return free(ptr); }

    /**
       how much free memory do we have in bytes. If unknown return 4096
     */
    virtual uint32_t available_memory(void){ return 4096; }
protected:
    // we start soft_armed false, so that actuators don't send any
    // values until the vehicle code has fully started
    bool soft_armed = false;
    uint64_t capabilities = 0;

};
