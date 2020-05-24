#pragma once

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS        (15.5f*GRAVITY_MSS) // accelerometer values over 15.5G are recorded as a clipping error
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz
#define AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS 500     // peak-hold detector timeout

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#define INS_MAX_INSTANCES 3
#define INS_MAX_BACKENDS  6
#define INS_VIBRATION_CHECK_INSTANCES 2

#define DEFAULT_IMU_LOG_BAT_MASK 0

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>
#include <Filter/NotchFilter.h>

#include "AP_Ins_Backend.h"

typedef enum MAV_RESULT
{
   MAV_RESULT_ACCEPTED=0,				/* Command ACCEPTED and EXECUTED | */
   MAV_RESULT_TEMPORARILY_REJECTED=1,	/* Command TEMPORARY REJECTED/DENIED | */
   MAV_RESULT_DENIED=2,					/* Command PERMANENTLY DENIED | */
   MAV_RESULT_UNSUPPORTED=3,			/* Command UNKNOWN/UNSUPPORTED | */
   MAV_RESULT_FAILED=4,					/* Command executed, but failed | */
   MAV_RESULT_ENUM_END=5,				/*  | */
} MAV_RESULT;

/* AP_Ins is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_Ins
{
public:
    AP_Ins();

    /* Do not allow copies */
    AP_Ins(const AP_Ins &other) = delete;
    AP_Ins &operator = (const AP_Ins&) = delete;

    static AP_Ins *get_instance();

    enum Gyro_Calibration_Timing {
        GYRO_CAL_NEVER = 0,
        GYRO_CAL_STARTUP_ONLY = 1
    };

    void init(uint16_t sample_rate_hz);

    const Vector3f &get_gyro() const { return gyro; }
    const Vector3f &get_gyro_offsets() const { return gyro_offset; }

    //get delta angle if available
    bool get_delta_angle(Vector3f &delta_angle) const;
    float get_delta_angle_dt() const;

    //get delta velocity if available
    bool get_delta_velocity(Vector3f &delta_velocity) const;
    float get_delta_velocity_dt() const;


	const Vector3f &get_accel() const { return accel; }
    const Vector3f &get_accel_scale() const { return accel_scale; }
    const Vector3f &get_accel_offsets() const { return accel_offset; }


    uint32_t get_gyro_error_count() const { return gyro_error_count; }

    // multi-device interface
    bool get_gyro_health() const { return gyro_healthy; }
    bool get_accel_health() const { return accel_healthy; }
	
    // get observed sensor rates, including any internal sampling multiplier
    uint16_t get_gyro_rate_hz() const { return uint16_t(gyro_raw_sample_rates * gyro_over_sampling); }
    uint16_t get_accel_rate_hz() const { return uint16_t(accel_raw_sample_rates * accel_over_sampling); }


	
    // return a 3D vector defining the position offset of the IMU accelerometer in metres relative to the body frame origin
    const Vector3f &get_imu_pos_offset() const { return accel_pos; }
	

    // return the temperature if supported. Zero is returned if no
    // temperature is available
    float get_temperature() const { return temperature; }

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    float get_delta_time() const { return MIN(delta_time, loop_delta_t_max); }

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    float get_gyro_drift_rate(void) const { return ToRad(0.5f/60); }

    // update gyro and accel values from accumulated samples
    void update(void);

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation, Matrix3f* custom_rotation = nullptr)
	{
        board_orientation = orientation;
        custom_rotation = custom_rotation;
    }

    // return the selected sample rate
    uint16_t get_sample_rate(void) const { return sample_rate; }

    // return the main loop delta_t in seconds
    float get_loop_delta_t(void) const { return loop_delta_t; }

    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_gyro_filter_hz(void) const { return gyro_filter_cutoff; }
    uint8_t get_accel_filter_hz(void) const { return accel_filter_cutoff; }

    // calculate vibration levels and check for accelerometer clipping (called by a backends)
    void calc_vibration_and_clipping(const Vector3f &accel, float dt);

    // retrieve latest calculated vibration levels
    Vector3f get_vibration_levels() const;

    // retrieve and clear accelerometer clipping count
    uint32_t get_accel_clip_count() const;

    // check for vibration movement. True when all axis show nearly zero movement
    bool is_still();

    // accel peak hold detector
    void set_accel_peak_hold(const Vector3f &accel);
    float get_accel_peak_hold_neg_x() const { return peak_hold_state.accel_peak_hold_neg_x; }

    // Returns newly calculated trim values if calculated
    bool get_new_trim(float& trim_roll, float &trim_pitch);
	
    // return time in microseconds of last update() call
    uint32_t get_last_update_usec(void) const { return last_update_usec; }

    enum IMU_SENSOR_TYPE {
        IMU_SENSOR_TYPE_ACCEL = 0,
        IMU_SENSOR_TYPE_GYRO = 1,
    };

private:
	
    AP_Ins_Backend *backend;

    bool calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch);
	
    // the selected sample rate
    uint16_t sample_rate;
    float loop_delta_t;
    float loop_delta_t_max;

    // Most recent accelerometer reading
    Vector3f accel;
    Vector3f delta_velocity;
    float delta_velocity_dt;
    bool delta_velocity_valid;
    // delta velocity accumulator
    Vector3f delta_velocity_acc;
    // time accumulator for delta velocity accumulator
    float delta_velocity_acc_dt;

    // Low Pass filters for gyro and accel
    LowPassFilter2pVector3f accel_filter;
    LowPassFilter2pVector3f gyro_filter;
	// filtering frequency (0 means default)
    int8_t accel_filter_cutoff;
    int8_t gyro_filter_cutoff;
	
	
    Vector3f accel_filtered;
    Vector3f gyro_filtered;
    bool new_accel_data;
    bool new_gyro_data;

    // optional notch filter on gyro
    NotchFilterVector3fParam notch_filter;

    // Most recent gyro reading
    Vector3f gyro;
    Vector3f delta_angle;
    float delta_angle_dt;
    bool delta_angle_valid;
    // time accumulator for delta angle accumulator
    float delta_angle_acc_dt;
    Vector3f delta_angle_acc;
    Vector3f last_delta_angle;
    Vector3f last_raw_gyro;


    // accelerometer scaling and offsets
    Vector3f accel_scale;
    Vector3f accel_offset;
    Vector3f gyro_offset;

    // accelerometer position offset in body frame
    Vector3f accel_pos;

    // accelerometer and gyro raw sample rate in units of Hz
    float accel_raw_sample_rates;
    float gyro_raw_sample_rates;

    // last sample time in microseconds. Use for deltaT calculations
    // on non-FIFO sensors
    uint64_t accel_last_sample_us;
    uint64_t gyro_last_sample_us;

    // sample times for checking real sensor rate for FIFO sensors
    uint16_t sample_accel_count;
    uint32_t sample_accel_start_us;
    uint16_t sample_gyro_count;
    uint32_t sample_gyro_start_us;
    
    // temperatures for an instance if available
    float temperature;
    
    // board orientation from AHRS
    enum Rotation board_orientation;
    Matrix3f *custom_rotation;

    // per-sensor orientation to allow for board type defaults at runtime
    enum Rotation gyro_orientation;
    enum Rotation accel_orientation;

    // calibrated_ok/id_ok flags
    bool gyro_cal_ok;
    bool accel_id_ok;


    // health of gyros and accels
    bool gyro_healthy;
    bool accel_healthy;

    uint32_t accel_error_count;
    uint32_t gyro_error_count;

    // vibration and clipping
    uint32_t accel_clip_count;
    LowPassFilterVector3f accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES];
    LowPassFilterVector3f accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES];

    // peak hold detector state for primary accel
    struct PeakHoldState
	{
        float accel_peak_hold_neg_x;
        uint32_t accel_peak_hold_neg_x_age;
    } peak_hold_state;

    // threshold for detecting stillness
    float still_threshold;

    // Trim options
    int8_t acc_body_aligned;
    int8_t trim_option;

    static AP_Ins *s_instance;

    float trim_pitch;
    float trim_roll;
    bool new_trim;
};

namespace AP {
    AP_Ins &ins();
};
