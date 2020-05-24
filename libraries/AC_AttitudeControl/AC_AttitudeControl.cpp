#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>


AC_AttitudeControl::AC_AttitudeControl( const AP_AHRS_View& _ahrs, AP_Motors& _motors ):
	ahrs(_ahrs),
	motors(_motors),
	rollAnglePID(AC_ANGLE_ROLL_P, AC_ANGLE_ROLL_I, AC_ANGLE_ROLL_D, AC_ANGLE_ROLL_IMAX, AC_ANGLE_ROLL_FILT_HZ),
	pitchAnglePID(AC_ANGLE_PITCH_P, AC_ANGLE_PITCH_I, AC_ANGLE_PITCH_D, AC_ANGLE_PITCH_IMAX, AC_ANGLE_PITCH_FILT_HZ),
	yawAnglePID(AC_ANGLE_YAW_P, AC_ANGLE_YAW_I, AC_ANGLE_YAW_D, AC_ANGLE_YAW_IMAX, AC_ANGLE_YAW_FILT_HZ),
	rollRatePID(AC_RATE_ROLL_P, AC_RATE_ROLL_I, AC_RATE_ROLL_D, AC_RATE_ROLL_IMAX, AC_RATE_ROLL_FILT_HZ),
	pitchRatePID(AC_RATE_PITCH_P, AC_RATE_PITCH_I, AC_RATE_PITCH_D, AC_RATE_PITCH_IMAX, AC_RATE_PITCH_FILT_HZ),
	yawRatePID(AC_RATE_YAW_P, AC_RATE_YAW_I, AC_RATE_YAW_D, AC_RATE_YAW_IMAX, AC_RATE_YAW_FILT_HZ)
{

}

void AC_AttitudeControl::RunAngleController()
{
	Vector3f actualAngle_deg = ahrs.eulerAngle_deg;
	Vector3f angleError_deg = desireAngleTarget_deg - actualAngle_deg;
}

void AC_AttitudeControl::RunRateController()
{
	Vector3f actualRate_rads = ahrs.gyro_rads;
	Vector3f rateError_rads = desireRateTarget_rads - actualRate_rads;
	
	rollRatePID.set_dt(rateLoopDt);
	pitchRatePID.set_dt(rateLoopDt);
	yawRatePID.set_dt(rateLoopDt);

    // pass error to PID controller
    rollRatePID.set_input_filter_d(rateError_rads.x);
    pitchRatePID.set_desired_rate(rateError_rads.y);
	yawRatePID.set_desired_rate(rateError_rads.z);

    Vector3f integrator;
	integrator.x = rollRatePID.get_integrator();
	integrator.y = pitchRatePID.get_integrator();
	integrator.z = yawRatePID.get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
	if ( ((is_positive(integrator.x) && is_negative(rateError_rads.x)) || (is_negative(integrator.x) && is_positive(rateError_rads.x)))) 
	{
		integrator.x = rollRatePID.get_i();
    }
	if ( ((is_positive(integrator.y) && is_negative(rateError_rads.y)) || (is_negative(integrator.y) && is_positive(rateError_rads.y)))) 
	{
		integrator.y = pitchRatePID.get_i();
    }
	if ( ((is_positive(integrator.z) && is_negative(rateError_rads.z)) || (is_negative(integrator.z) && is_positive(rateError_rads.z)))) 
	{
		integrator.z = yawRatePID.get_i();
    }

	Vector3f output;
    output.x = rollRatePID.get_p() + integrator.x + rollRatePID.get_d() + rollRatePID.get_ff(desireRateTarget_rads.x);
	output.y = pitchRatePID.get_p() + integrator.y + pitchRatePID.get_d() + pitchRatePID.get_ff(desireRateTarget_rads.y);
	output.z = yawRatePID.get_p() + integrator.z + yawRatePID.get_d() + yawRatePID.get_ff(desireRateTarget_rads.z);
    
	// Constrain output in range -1 ~ +1
	output.x = constrain_float(output.x, -1.0f, 1.0f);
	output.y = constrain_float(output.y, -1.0f, 1.0f);
	output.z = constrain_float(output.z, -1.0f, 1.0f);
	
	// output to motors
	motors.SetRoll( output.x );
	motors.SetPitch( output.y );
	motors.SetYaw( output.z );
	
}


// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl::RelaxAttitudeControllers()
{
    // Initialize the attitude variables to the current attitude

    // Reset the PID filters
    rollAnglePID.reset_filter();
    pitchAnglePID.reset_filter();
    yawAnglePID.reset_filter();
	rollRatePID.reset_filter();
    pitchRatePID.reset_filter();
    yawRatePID.reset_filter();

    // Reset the I terms
	rollAnglePID.reset_I();
    pitchAnglePID.reset_I();
    yawAnglePID.reset_I();
	rollRatePID.reset_I();
    pitchRatePID.reset_I();
    yawRatePID.reset_I();
}
