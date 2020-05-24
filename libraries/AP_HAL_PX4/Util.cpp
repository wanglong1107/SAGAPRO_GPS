#include <AP_HAL/AP_HAL.h>
#include "Util.h"


extern const AP_HAL::HAL& hal;


using namespace PX4;


PX4Util::PX4Util(void) : Util()
{

}


bool PX4Util::get_system_id(char buf[40])
{

    return true;
}

/**
   how much free memory do we have in bytes.
*/
uint32_t PX4Util::available_memory(void) 
{
	return 4096;
}

void *PX4Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    return calloc(1, size);
}
void PX4Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    return free(ptr);
}


