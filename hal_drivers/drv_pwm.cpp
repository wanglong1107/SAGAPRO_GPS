#include "drv_pwm.h"
#include <string.h>
#include <os.h>


int HAL_PWM::initialize(void)
{
	if(!initialFlag)
	{
//		pwm_init(hw);
		initialFlag = true;
	}

	return 0;
}

void HAL_PWM::enablePWM(void)
{
	pwm_enable(hw);
}

void HAL_PWM::disablePWM(void)
{
	pwm_disable(hw);
}

int HAL_PWM::disInitialize(void)
{
	if(initialFlag)
	{
		pwm_deinit(hw);
		initialFlag = false;
	}
	return 0;
}

void HAL_PWM::set_period(const uint32_t period, uint8_t ch)
{
	hri_pwm_write_CPRDUPD_reg(hw->device.hw, ch, period);
}
void HAL_PWM::set_duty(const uint32_t duty_cycle, uint8_t ch)
{
	hri_pwm_write_CDTYUPD_reg(hw->device.hw, ch, duty_cycle);
}
uint32_t HAL_PWM::get_period(uint8_t ch)
{
	return hri_pwm_read_CPRD_reg(hw->device.hw, ch);
}
uint32_t HAL_PWM::get_duty(uint8_t ch)
{
	return hri_pwm_read_CDTY_reg(hw->device.hw, ch);
}
