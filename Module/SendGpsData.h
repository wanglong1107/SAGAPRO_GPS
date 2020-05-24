#ifndef SEND_GPS_DATA_H
#define SEND_GPS_DATA_H

#include <stdint.h>

#include <AP_HAL/utility/RingBuffer.h>
#include <hal_drivers/hal_uart.h>
#include <hal_drivers/drv_hrt.h>

#include "Module/ublox_F9P.h"

#include <hal_delay.h>

class SendGpsData
{
public:
	void DopData(void);

	void PosllhData(void);

	void PvtData(void);

	void RelposnedData(void);

	void VelnedData(void);

	void RxmRtcmData(void);

};

#endif