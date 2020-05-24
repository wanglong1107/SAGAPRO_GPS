#ifndef _DRV_PWM_H
#define _DRV_PWM_H

#include "hal_pwm.h"
#include <stdint.h>

class HAL_PWM
{
private:
	struct pwm_descriptor *hw;

public:
	bool initialFlag;

	HAL_PWM( void *const _hw)
	{
		hw = (struct pwm_descriptor*)_hw;
		
		initialFlag = false;
	}
	
	int initialize(void);
	int disInitialize(void);
	void enablePWM(void);
	void disablePWM(void);
	void set_period(const uint32_t period, uint8_t ch);
	void set_duty(const uint32_t duty_cycle, uint8_t ch);
	uint32_t get_period(uint8_t ch);
	uint32_t get_duty(uint8_t ch);

};

#endif //_DRV_PWM_H
