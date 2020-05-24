#ifndef _AC_ATTITUDE_CONTROL_H
#define _AC_ATTITUDE_CONTROL_H

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>
#include <AP_AHRS/AP_AHRS_View.h>

#define AC_ATTITUDE__ANGLE_MAX                         4500          //deg*100  default max lean angle for roll, pitch

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS       radians(40.0f)   // minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS       radians(720.0f)  // maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS        radians(10.0f)   // minimum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        radians(120.0f)  // maximum body-frame acceleration limit for the stability controller (for yaw axis)
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS        6000      // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sec * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS   110000.0f // default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS    27000.0f  // default maximum acceleration for yaw axis in centidegrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          1.0f    // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         1.0f    // body-frame rate controller maximum output (for yaw axis)

#define AC_ATTITUDE_THRUST_ERROR_ANGLE                  radians(30.0f) // Thrust angle error above which yaw corrections are limited

#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          1       // body-frame rate feedforward enabled by default

#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT      1.0f    // Time constant used to limit lean angle so that vehicle does not lose altitude
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX    0.8f    // Max throttle used to limit lean angle so that vehicle does not lose altitude
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN             10.0f   // Min lean angle so that vehicle can maintain limited control

#define AC_ATTITUDE_CONTROL_MIN_DEFAULT                 0.1f    // minimum throttle mix default
#define AC_ATTITUDE_CONTROL_MAN_DEFAULT                 0.5f    // manual throttle mix default
#define AC_ATTITUDE_CONTROL_MAX_DEFAULT                 0.5f    // maximum throttle mix default
#define AC_ATTITUDE_CONTROL_MAX                         5.0f    // maximum throttle mix default

#define AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT             0.5f  // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input



#define AC_ANGLE_ROLL_P			4.5f
#define AC_ANGLE_ROLL_I			0.0f
#define AC_ANGLE_ROLL_D			0.0f
#define AC_ANGLE_ROLL_IMAX		0.5f
#define AC_ANGLE_ROLL_FILT_HZ	20.0f

#define AC_ANGLE_PITCH_P		4.5f
#define AC_ANGLE_PITCH_I		0.0f
#define AC_ANGLE_PITCH_D		0.0f
#define AC_ANGLE_PITCH_IMAX		0.5f
#define AC_ANGLE_PITCH_FILT_HZ	20.0f

#define AC_ANGLE_YAW_P			4.5f
#define AC_ANGLE_YAW_I			0.0f
#define AC_ANGLE_YAW_D			0.0f
#define AC_ANGLE_YAW_IMAX		0.5f
#define AC_ANGLE_YAW_FILT_HZ	20.0f


#define AC_RATE_ROLL_P           0.135f
#define AC_RATE_ROLL_I           0.090f
#define AC_RATE_ROLL_D           0.0036f
#define AC_RATE_ROLL_IMAX         0.5f
#define AC_RATE_ROLL_FILT_HZ      20.0f

#define AC_RATE_PITCH_P           0.135f
#define AC_RATE_PITCH_I           0.090f
#define AC_RATE_PITCH_D           0.0036f
#define AC_RATE_PITCH_IMAX         0.5f
#define AC_RATE_PITCH_FILT_HZ      20.0f

#define AC_RATE_YAW_P           0.180f
#define AC_RATE_YAW_I           0.018f
#define AC_RATE_YAW_D           0.0f
#define AC_RATE_YAW_IMAX        0.5f
#define AC_RATE_YAW_FILT_HZ     2.5f


class AC_AttitudeControl
{
public:
	AC_AttitudeControl( const AP_AHRS_View& _ahrs, AP_Motors& _motors );
    ~AC_AttitudeControl() {}
	
	void SetDesireAngleTarget( const Vector3f& _desireAngleTarget );
	void SetDesireRateTartget( const Vector3f& _desireRateTarget );
	
	Vector3f desireAngleTarget_deg;
	Vector3f desireRateTarget_rads;

	// angle controller PID objects
    AC_PID rollAnglePID;
	AC_PID pitchAnglePID;
	AC_PID yawAnglePID;
	
	// rate controller PID objects
	AC_PID rollRatePID;
	AC_PID pitchRatePID;
	AC_PID yawRatePID;
	
    // pid accessors
    AC_PID& GetRollAnglePID() { return rollAnglePID; }
    AC_PID& GetPitchAnglePID() { return pitchAnglePID; }
    AC_PID& GetYawAnglePID() { return yawAnglePID; }
	AC_PID& GetRollRatePID() { return rollRatePID; }
    AC_PID& GetPitchRatePID() { return pitchRatePID; }
    AC_PID& GetYawRatePID() { return yawRatePID; }
	
	
	// Run angular controller
	void RunAngleController();
	
	// Run rate controller and send outputs to the motors
    void RunRateController();
	
	// Intersampling period in seconds
	float angleLoopDt;
	float rateLoopDt;
	
	// Ensure attitude controller have zero errors to relax rate controller output
	void RelaxAttitudeControllers();
    
    //const AP_Vehicle::MultiCopter &_aparm;
	const AP_AHRS_View&  ahrs;
    AP_Motors& motors;
private:
	

};

#endif
