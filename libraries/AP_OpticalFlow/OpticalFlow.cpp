//#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OpticalFlow.h"
//#include "AP_OpticalFlow_Onboard.h"
//#include "AP_OpticalFlow_SITL.h"
//#include "AP_OpticalFlow_Pixart.h"
#include "AP_OpticalFlow_PX4Flow.h"

extern const AP_HAL::HAL& hal;

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF &ahrs)
    : _ahrs(ahrs),
      _last_update_ms(0)
{
//    AP_Param::setup_object_defaults(this, var_info);

    memset(&_state, 0, sizeof(_state));

    // healthy flag will be overwritten on update
    _flags.healthy = false;
}

void OpticalFlow::init(void)
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    if (!backend) {
#if AP_FEATURE_BOARD_DETECT
        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK ||
            AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2 ||
            AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PCNC1) {
            // possibly have pixhart on external SPI
            backend = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
        }
        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_SP01) {
            backend = AP_OpticalFlow_Pixart::detect("pixartPC15", *this);
        }
        if (backend == nullptr) {
            backend = AP_OpticalFlow_PX4Flow::detect(*this);
        }
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        backend = new AP_OpticalFlow_SITL(*this);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
        backend = new AP_OpticalFlow_Onboard(*this);
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
        backend = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
#endif
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void OpticalFlow::update(void)
{
    if (backend != nullptr) {
        backend->update();
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

