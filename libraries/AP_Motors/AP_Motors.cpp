#include "AP_Motors.h"
#include "globalVariables.h"

void AP_Motors::AddMotor( uint8_t _num, float _degrees, float _yaw_factor)
{
	motorsParameters[_num].motor_num = _num;
	motorsParameters[_num].rollFactor = cosf(radians(_degrees + 90));
	motorsParameters[_num].pitchFactor = cosf(radians(_degrees));
	motorsParameters[_num].yawFactor = _yaw_factor;
}
	
void AP_Motors::SetMotorsParameters(void)
{
	bool success = false;
	switch(motorFrameclass)
	{
		case MOTOR_FRAME_QUAD:
			switch(motorFrametype)
			{
				case MOTOR_FRAME_TYPE_PLUS:
					AddMotor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
					AddMotor(AP_MOTORS_MOT_2,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
					AddMotor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
					AddMotor(AP_MOTORS_MOT_4,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
					success = true;
					break;
				case MOTOR_FRAME_TYPE_X:
					AddMotor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
					AddMotor(AP_MOTORS_MOT_2,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
					AddMotor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
					AddMotor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
					success = true;
					break;
				default:
					break;
			}
			break;
		case MOTOR_FRAME_HEXA:
			switch(motorFrametype)
			{
            	case MOTOR_FRAME_TYPE_PLUS:
                    AddMotor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
                    AddMotor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
                    AddMotor(AP_MOTORS_MOT_3, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
                    AddMotor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
                    AddMotor(AP_MOTORS_MOT_5,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
                    AddMotor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);             
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
					AddMotor(AP_MOTORS_MOT_1,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
                    AddMotor(AP_MOTORS_MOT_2,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
                    AddMotor(AP_MOTORS_MOT_3, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
                    AddMotor(AP_MOTORS_MOT_4,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
                    AddMotor(AP_MOTORS_MOT_5, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
                    AddMotor(AP_MOTORS_MOT_6, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW);      
                    success = true;
                    break;
                default:
                    break;
			}
		default:
			break;	
	}
}

void AP_Motors::SetFrameClassAndType(AP_MotorFrameClass _frameClass, AP_MotorFrameType _frameType)
{
    motorFrameclass = _frameClass;
    motorFrametype = _frameType;
    SetMotorsParameters();
}

void AP_Motors::SetMotorMode(AP_MotorMode _motorMode)
{
	motorMode = _motorMode;
}

void AP_Motors::OutPut()
{
	uint8_t i;
	float   roll_thrust;                // roll thrust input value, +/- 1.0
	float   pitch_thrust;               // pitch thrust input value, +/- 1.0
	float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
	float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
	float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
	float   rpy_low = 0.0f;             // lowest motor value
	float   rpy_high = 0.0f;            // highest motor value
	float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
	float   unused_range;               // amount of yaw we can fit in the current channel
	float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
	float 	voltage_compensation_gain;
	
	voltage_compensation_gain = Update_Voltage_Compensation_Gain();
	roll_thrust = rollInput*voltage_compensation_gain;
	pitch_thrust = pitchInput*voltage_compensation_gain;
	yaw_thrust = yawInput*voltage_compensation_gain;
	
	throttle_thrust = throttleInput*voltage_compensation_gain;
	throttle_thrust = constrain_float(throttle_thrust, 0.0f, 1.0f);
	
	// calculate roll and pitch for each motor
  // calculate the amount of yaw input that each motor can accept
	for(i=0;i<AP_MOTORS_NUM;i++)
	{
		thrustOutput[i] = motorsParameters[i].rollFactor*roll_thrust + motorsParameters[i].pitchFactor*pitch_thrust;
		if(!is_zero(motorsParameters[i].yawFactor))
		{
			if(motorsParameters[i].yawFactor*yaw_thrust > 0.0f)
			{
				unused_range = fabsf(MAX(1.0f - (throttle_thrust + thrustOutput[i]), 0.0f)/motorsParameters[i].yawFactor);
				if(yaw_allowed > unused_range)
				{
					yaw_allowed = unused_range;
				}
			}
			else
			{
				unused_range = fabsf(MAX(throttle_thrust + thrustOutput[i], 0.0f)/motorsParameters[i].yawFactor);
				if (yaw_allowed > unused_range)
				{
					yaw_allowed = unused_range;
				}
			}
		}
	}
	
	if(fabsf(yaw_thrust) > yaw_allowed)
	{
		 yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
	}
	
	// add yaw to intermediate numbers for each motor
	rpy_low = 0.0f;
	rpy_high = 0.0f;
	for(i=0; i<AP_MOTORS_NUM; i++) 
	{
		thrustOutput[i] = thrustOutput[i] + motorsParameters[i].yawFactor*yaw_thrust;

		// record lowest roll+pitch+yaw command
		if(thrustOutput[i] < rpy_low)
		{
			rpy_low = thrustOutput[i];
		}
		// record highest roll+pitch+yaw command
		if(thrustOutput[i] > rpy_high)
		{
			rpy_high = thrustOutput[i];
		}
	}

	if(is_zero(rpy_low) && is_zero(rpy_high))
	{
		rpy_scale = 1.0f;
	}
	else if(is_zero(rpy_low))
	{
		rpy_scale = constrain_float((1.0f-throttle_thrust)/rpy_high, 0.0f, 1.0f);
	}
	else if(is_zero(rpy_high))
	{
		rpy_scale = constrain_float(-throttle_thrust/rpy_low, 0.0f, 1.0f);
	}
	else
	{
		rpy_scale = constrain_float(MIN(-throttle_thrust/rpy_low, (1.0f-throttle_thrust)/rpy_high), 0.0f, 1.0f);
	}
	
	// distribute power
    for (i=0; i<AP_MOTORS_NUM; i++)
	  {
			thrustOutput[i] = throttle_thrust + rpy_scale*thrustOutput[i];
    }
	
	// output
	switch(motorMode)
	{
		case SHUT_DOWN:
			pwm1.set_duty(1100, AP_MOTORS_ESC_1);
			pwm1.set_duty(1100, AP_MOTORS_ESC_2);
			pwm1.set_duty(1100, AP_MOTORS_ESC_3);
			pwm1.set_duty(1100, AP_MOTORS_ESC_4);
			break;
		case SPIN_WHEN_ARMED:
			pwm1.set_duty(1100, AP_MOTORS_ESC_1);
			pwm1.set_duty(1100, AP_MOTORS_ESC_2);
			pwm1.set_duty(1100, AP_MOTORS_ESC_3);
			pwm1.set_duty(1100, AP_MOTORS_ESC_4);
			break;
		case THROTTLE_UNLIMITED:
			pwm1.set_duty(thrustOutput[0], AP_MOTORS_ESC_1);
			pwm1.set_duty(thrustOutput[1], AP_MOTORS_ESC_2);
			pwm1.set_duty(thrustOutput[2], AP_MOTORS_ESC_3);
			pwm1.set_duty(thrustOutput[3], AP_MOTORS_ESC_4);
			break;
		 default:
		 	pwm1.set_duty(1100, AP_MOTORS_ESC_1);
			pwm1.set_duty(1100, AP_MOTORS_ESC_2);
			pwm1.set_duty(1100, AP_MOTORS_ESC_3);
			pwm1.set_duty(1100, AP_MOTORS_ESC_4);
			break;		
	}
}

float AP_Motors::Update_Voltage_Compensation_Gain()
{
	float lift_max;
	float thrust_curve_expo;
	float ret;
	
	if(voltage_compensation_enable == true)
	{
		battery_voltage_estimate = MAX(battery_voltage,battery_voltage_estimate);
		
		if((battery_voltage_max <= 0) || (battery_voltage_min >= battery_voltage_max) || (battery_voltage_estimate < 0.25f*battery_voltage_min))
		{
				lift_max = 1.0f;
		}
		
		battery_voltage_min	= MAX(battery_voltage_min,battery_voltage_max * 0.6f);
		battery_voltage_estimate = constrain_float(battery_voltage_estimate,battery_voltage_min,battery_voltage_max);

		float battery_voltage_filt = batt_voltage_filt.apply(battery_voltage_estimate/battery_voltage_max, 1.0f/LOOPRATE);
		
		thrust_curve_expo = constrain_float(THRUST_CURVE_EXPO, -1.0f, 1.0f);
		
		lift_max = battery_voltage_filt*(1-thrust_curve_expo) + thrust_curve_expo*battery_voltage_filt*battery_voltage_filt;
		
		if(lift_max <= 0.0f)
		{
			return 1.0f;
		}
		
		ret = 1.0f /  lift_max;
		
		return ret;
	}
	
	else
	{
		ret = 1.0f;
		return ret;
	}
}