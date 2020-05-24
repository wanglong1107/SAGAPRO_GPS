#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_PX4_Namespace.h"


class HAL_PX4 : public AP_HAL::HAL {
public:
    HAL_PX4();

	virtual void initialize() const override;
};

//void hal_px4_set_priority(uint8_t priority);
