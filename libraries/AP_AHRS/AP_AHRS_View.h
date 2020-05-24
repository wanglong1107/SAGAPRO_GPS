#ifndef __AP_AHRS_VIEW_H
#define __AP_AHRS_VIEW_H

#include <AP_Math/AP_Math.h>

class  AP_AHRS_View
{
	public:
		AP_AHRS_View();
		~AP_AHRS_View(){}
			
		static AP_AHRS_View *get_instance();
		static AP_AHRS_View *_s_instance;

	
		float Accx_cmss;  //accNED_N
		float Accy_cmss;  //accNED_E
		float Accz_cmss;  //accNED_D
		Vector3f accel_cmss;
		
		float roll_deg;
		float pitch_deg;
		float yaw_deg;
		Vector3f eulerAngle_deg;
		Vector3f eulerAngle_rad;
		
		float gyro_x;   //  ap_gyro
		float gyro_y;   //
		float gyro_z;   //
		Vector3f gyro_degs;
		Vector3f gyro_rads;
		
		float cosyaw;
		float sinyaw;
		
		Vector3f Pos_cm;
		Vector3f Vel_cm;
		Quaternion  quat;
		
		
		// returns the inertial navigation origin in lat*10e7/lon*10e7/alt_cm
		Vector3l  origin;
		Vector3l  home;
		bool home_is_set;
		
		
};


AP_AHRS_View &ahrs();



#endif  //__AP_AHRS_VIEW_H
