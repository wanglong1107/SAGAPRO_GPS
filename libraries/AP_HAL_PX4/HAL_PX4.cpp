#include <AP_HAL/AP_HAL.h>

//#include <AP_HAL/utility/RCOutput_Tap.h>
//#include <AP_HAL_Empty/AP_HAL_Empty.h>
//#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"
#include "HAL_PX4.h"
#include "Scheduler.h"
#include "UARTDriver.h"
#include "Util.h"
#include "I2CDevice.h"
#include "SPIDevice.h"


#include <stdlib.h>
#include <stdio.h>

#include <hal_drivers/drv_hrt.h>

using namespace PX4;

static PX4Scheduler schedulerInstance;
static PX4Util utilInstance;

HAL_QSPI mpu_20602( &QUAD_SPI, qspi_ncs_1);
HAL_I2C mag_qmc5883( &I2C_0, 0x0d);
HAL_I2C baro_ms5611( &I2C_0, 0x77);


static PX4SPIDevice ins( &mpu_20602 );
static PX4I2CDevice mag( &mag_qmc5883 );
static PX4I2CDevice baro( &baro_ms5611 );





//// 3 UART drivers, for GPS plus two mavlink-enabled devices
//static PX4UARTDriver uartADriver(UARTA_DEFAULT_DEVICE, "APM_uartA");
//static PX4UARTDriver uartBDriver(UARTB_DEFAULT_DEVICE, "APM_uartB");
//static PX4UARTDriver uartCDriver(UARTC_DEFAULT_DEVICE, "APM_uartC");
//static PX4UARTDriver uartDDriver(UARTD_DEFAULT_DEVICE, "APM_uartD");
//static PX4UARTDriver uartEDriver(UARTE_DEFAULT_DEVICE, "APM_uartE");
//static PX4UARTDriver uartFDriver(UARTF_DEFAULT_DEVICE, "APM_uartF");

HAL_PX4::HAL_PX4() :
    AP_HAL::HAL(
//        &uartADriver,
//        &uartBDriver,
//        &uartCDriver,
//        &uartDDriver,
//        &uartEDriver,
//        &uartFDriver,
				&ins,
				&mag,
				&baro,

        &schedulerInstance, /* scheduler */
        &utilInstance /* util */
		)
{}

void HAL_PX4::initialize() const
{
//	mpu_20602.initialize();
//	mag_hmc5883.initialize();
//	baro_ms5611.initialize();
	
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_PX4 hal_px4;
    return hal_px4;
}



