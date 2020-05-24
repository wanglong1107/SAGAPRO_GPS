#include "AP_AHRS_View.h"


AP_AHRS_View *AP_AHRS_View::_s_instance = nullptr;

AP_AHRS_View::AP_AHRS_View()
{
    if (_s_instance)
	{
//        AP_HAL::panic("Too many inertial sensors");
    }
    _s_instance = this;
		
}


/*
 * Get the AP_InertialSensor singleton
 */
AP_AHRS_View *AP_AHRS_View::get_instance()
{
    if (!_s_instance) {
        _s_instance = new AP_AHRS_View();
    }
    return _s_instance;
}


AP_AHRS_View &ahrs()
{
	return *AP_AHRS_View::get_instance();
}
