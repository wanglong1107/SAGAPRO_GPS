#include <assert.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Ins.h"

/* Define INS_TIMING_DEBUG to track down scheduling issues with the main loop.
 * Output is on the debug console. */
#ifdef INS_TIMING_DEBUG
#include <stdio.h>
#define timing_printf(fmt, args...)      //do { printf("[timing] " fmt, ##args); } while(0)
#else
#define timing_printf(fmt, args...)
#endif

extern const AP_HAL::HAL& hal;


#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#define DEFAULT_STILL_THRESH 0.1f

#define SAMPLE_UNIT 1

#define GYRO_INIT_MAX_DIFF_DPS 0.1f


AP_Ins *AP_Ins::_s_instance = nullptr;

AP_Ins::AP_Ins() :
    _board_orientation(ROTATION_NONE),
    _log_raw_bit(-1)
{
    if (_s_instance) {
//        AP_HAL::panic("Too many inertial sensors");
    }
    _s_instance = this;
//    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _gyro_cal_ok[i] = true;
        _accel_max_abs_offsets[i] = 3.5f;
    }
    for (uint8_t i=0; i<INS_VIBRATION_CHECK_INSTANCES; i++) {
        _accel_vibe_floor_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ);
        _accel_vibe_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ);
    }

//    AP_AccelCal::register_client(this);
}

/*
 * Get the AP_Ins singleton
 */
AP_Ins *AP_Ins::get_instance()
{
    if (!_s_instance) {
        _s_instance = new AP_Ins();
    }
    return _s_instance;
}

void AP_Ins::init(uint16_t sample_rate)
{
    // remember the sample rate
    _sample_rate = sample_rate;
    _loop_delta_t = 1.0f / sample_rate;

    // we don't allow deltat values greater than 10x the normal loop
    // time to be exposed outside of INS. Large deltat values can
    // cause divergence of state estimators
    _loop_delta_t_max = 10 * _loop_delta_t;

    if (_gyro_count == 0 && _accel_count == 0) {
        _start_backends();
    }

    // initialise accel scale if need be. This is needed as we can't
    // give non-zero default values for vectors in AP_Param
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (_accel_scale[i].is_zero()) {
            _accel_scale[i] = Vector3f(1,1,1);
        }
    }

    // calibrate gyros unless gyro calibration has been disabled
    if (gyro_calibration_timing() != GYRO_CAL_NEVER) {
        init_gyro();
    }

    _sample_period_usec = 1000*1000UL / _sample_rate;

    _notch_filter.init(sample_rate);
    
    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;

    // initialise IMU batch logging
    batchsampler.init();
}


// Armed, Copter, PixHawk:
// ins_periodic: 57500 events, 0 overruns, 208754us elapsed, 3us avg, min 1us max 218us 40.662us rms
void AP_Ins::periodic()
{
    batchsampler.periodic();
}


/*
  _calculate_trim - calculates the x and y trim angles. The
  accel_sample must be correctly scaled, offset and oriented for the
  board
*/
bool AP_Ins::_calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch)
{
    trim_pitch = atan2f(accel_sample.x, norm(accel_sample.y, accel_sample.z));
    trim_roll = atan2f(-accel_sample.y, -accel_sample.z);
    if (fabsf(trim_roll) > radians(10) ||
        fabsf(trim_pitch) > radians(10)) {
//        hal.console->printf("trim over maximum of 10 degrees\n");
        return false;
    }
//    hal.console->printf("Trim OK: roll=%.2f pitch=%.2f\n",
//                          (double)degrees(trim_roll),
//                          (double)degrees(trim_pitch));
    return true;
}

void
AP_Ins::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_gyro_calibration();
}

// accelerometer clipping reporting
uint32_t AP_Ins::get_accel_clip_count(uint8_t instance) const
{
    if (instance >= get_accel_count()) {
        return 0;
    }
    return _accel_clip_count[instance];
}

// get_gyro_health_all - return true if all gyros are healthy
bool AP_Ins::get_gyro_health_all(void) const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!get_gyro_health(i)) {
            return false;
        }
    }
    // return true if we have at least one gyro
    return (get_gyro_count() > 0);
}

// gyro_calibration_ok_all - returns true if all gyros were calibrated successfully
bool AP_Ins::gyro_calibrated_ok_all() const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!gyro_calibrated_ok(i)) {
            return false;
        }
    }
    for (uint8_t i=get_gyro_count(); i<INS_MAX_INSTANCES; i++) {
        if (_gyro_id[i] != 0) {
            // missing gyro
            return false;
        }
    }
    return (get_gyro_count() > 0);
}

// return true if gyro instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_Ins::use_gyro(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_gyro_health(instance) && _use[instance]);
}

// get_accel_health_all - return true if all accels are healthy
bool AP_Ins::get_accel_health_all(void) const
{
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!get_accel_health(i)) {
            return false;
        }
    }
    // return true if we have at least one accel
    return (get_accel_count() > 0);
}


/*
  calculate the trim_roll and trim_pitch. This is used for redoing the
  trim without needing a full accel cal
 */
bool AP_Ins::calibrate_trim(float &trim_roll, float &trim_pitch)
{
    Vector3f level_sample;

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return false;
    }

    _calibrating = true;

    const uint8_t update_dt_milliseconds = (uint8_t)(1000.0f/get_sample_rate()+0.5f);

    // wait 100ms for ins filter to rise
    for (uint8_t k=0; k<100/update_dt_milliseconds; k++) {
        wait_for_sample();
        update();
//        hal.scheduler->delay(update_dt_milliseconds);
    }

    uint32_t num_samples = 0;
    while (num_samples < 400/update_dt_milliseconds) {
        wait_for_sample();
        // read samples from ins
        update();
        // capture sample
        Vector3f samp;
        samp = get_accel(0);
        level_sample += samp;
        if (!get_accel_health(0)) {
            goto failed;
        }
//        hal.scheduler->delay(update_dt_milliseconds);
        num_samples++;
    }
    level_sample /= num_samples;

    if (!_calculate_trim(level_sample, trim_roll, trim_pitch)) {
        goto failed;
    }

    _calibrating = false;
    return true;

failed:
    _calibrating = false;
    return false;
}

/*
  check if the accelerometers are calibrated in 3D and that current number of accels matched number when calibrated
 */
bool AP_Ins::accel_calibrated_ok_all() const
{
    // calibration is not applicable for HIL mode
    if (_hil_mode) {
        return true;
    }

    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!_accel_id_ok[i]) {
            return false;
        }
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset[i].is_zero()) {
            return false;
        }
        // zero scaling also indicates not calibrated
        if (_accel_scale[i].is_zero()) {
            return false;
        }
    }
    for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
        if (_accel_id[i] != 0) {
            // missing accel
            return false;
        }
    }
    
    // check calibrated accels matches number of accels (no unused accels should have offsets or scaling)
    if (get_accel_count() < INS_MAX_INSTANCES) {
        for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
            const Vector3f &scaling = _accel_scale[i];
            bool have_scaling = (!is_zero(scaling.x) && !is_equal(scaling.x,1.0f)) || (!is_zero(scaling.y) && !is_equal(scaling.y,1.0f)) || (!is_zero(scaling.z) && !is_equal(scaling.z,1.0f));
            bool have_offsets = !_accel_offset[i].is_zero();
            if (have_scaling || have_offsets) {
                return false;
            }
        }
    }

    // if we got this far the accelerometers must have been calibrated
    return true;
}

// return true if accel instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_Ins::use_accel(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_accel_health(instance) && _use[instance]);
}

void
AP_Ins::_init_gyro()
{
    uint8_t num_gyros = MIN(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average, best_avg;
    Vector3f new_gyro_offset;
    float best_diff;
    bool converged;

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return;
    }

    // record we are calibrating
    _calibrating = true;

    // flash leds to tell user to keep the IMU still
//    AP_Notify::flags.initialising = true;

    // cold start
//    hal.console->printf("Init Gyro");

    /*
      we do the gyro calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset[k] = Vector3f();
        new_gyro_offset[k].zero();
        best_diff[k] = -1.f;
        last_average[k].zero();
        converged[k] = false;
    }

    for(int8_t c = 0; c < 5; c++) {
//        hal.scheduler->delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 30 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 30*4 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum, gyro_avg, gyro_diff;
        Vector3f accel_start;
        float diff_norm;
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

//        hal.console->printf("*");

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        accel_start = get_accel(0);
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += get_gyro(k);
            }
//            hal.scheduler->delay(5);
        }

        Vector3f accel_diff = get_accel(0) - accel_start;
        if (accel_diff.length() > 0.2f) {
            // the accelerometers changed during the gyro sum. Skip
            // this sample. This copes with doing gyro cal on a
            // steadily moving platform. The value 0.2 corresponds
            // with around 5 degrees/second of rotation.
            continue;
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (best_diff[k] < 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(GYRO_INIT_MAX_DIFF_DPS)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                if (!converged[k] || last_average[k].length() < new_gyro_offset[k].length()) {
                    new_gyro_offset[k] = last_average[k];
                }
                if (!converged[k]) {
                    converged[k] = true;
                    num_converged++;
                }
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
//    hal.console->printf("\n");
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
//            hal.console->printf("gyro[%u] did not converge: diff=%f dps (expected < %f)\n",
//                                (unsigned)k,
//                                (double)ToDeg(best_diff[k]),
//                                (double)GYRO_INIT_MAX_DIFF_DPS);
            _gyro_offset[k] = best_avg[k];
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        } else {
            _gyro_cal_ok[k] = true;
            _gyro_offset[k] = new_gyro_offset[k];
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    // record calibration complete
    _calibrating = false;

    // stop flashing leds
//    AP_Notify::flags.initialising = false;
}

/*
  update gyro and accel values from backends
 */
void AP_Ins::update(void)
{
	
}

/*
  get delta angles
 */
bool AP_Ins::get_delta_angle(uint8_t i, Vector3f &delta_angle) const
{
    if (_delta_angle_valid[i]) {
        delta_angle = _delta_angle[i];
        return true;
    } else if (get_gyro_health(i)) {
        // provide delta angle from raw gyro, so we use the same code
        // at higher level
        delta_angle = get_gyro(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
  get delta velocity if available
*/
bool AP_Ins::get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const
{
    if (_delta_velocity_valid[i]) {
        delta_velocity = _delta_velocity[i];
        return true;
    } else if (get_accel_health(i)) {
        delta_velocity = get_accel(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
  return delta_time for the delta_velocity
 */
float AP_Ins::get_delta_velocity_dt(uint8_t i) const
{
    float ret;
    if (_delta_velocity_valid[i]) {
        ret = _delta_velocity_dt[i];
    } else {
        ret = get_delta_time();
    }
    ret = MIN(ret, _loop_delta_t_max);
    return ret;
}

/*
  return delta_time for the delta_angle
 */
float AP_Ins::get_delta_angle_dt(uint8_t i) const
{
    float ret;
    if (_delta_angle_valid[i] && _delta_angle_dt[i] > 0) {
        ret = _delta_angle_dt[i];
    } else {
        ret = get_delta_time();
    }
    ret = MIN(ret, _loop_delta_t_max);
    return ret;
}


// calculate vibration levels and check for accelerometer clipping (called by a backends)
void AP_Ins::calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt)
{
    // check for clipping
    if (fabsf(accel.x) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.y) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.z) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS) {
        _accel_clip_count[instance]++;
    }

    // calculate vibration levels
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        // filter accel at 5hz
        Vector3f accel_filt = _accel_vibe_floor_filter[instance].apply(accel, dt);

        // calc difference from this sample and 5hz filtered value, square and filter at 2hz
        Vector3f accel_diff = (accel - accel_filt);
        accel_diff.x *= accel_diff.x;
        accel_diff.y *= accel_diff.y;
        accel_diff.z *= accel_diff.z;
        _accel_vibe_filter[instance].apply(accel_diff, dt);
    }
}

// peak hold detector for slower mechanisms to detect spikes
void AP_Ins::set_accel_peak_hold(uint8_t instance, const Vector3f &accel)
{
    if (instance != _primary_accel) {
        // we only record for primary accel
        return;
    }
    uint32_t now = AP_HAL::millis();

    // negative x peak(min) hold detector
    if (accel.x < _peak_hold_state.accel_peak_hold_neg_x ||
        _peak_hold_state.accel_peak_hold_neg_x_age <= now) {
        _peak_hold_state.accel_peak_hold_neg_x = accel.x;
        _peak_hold_state.accel_peak_hold_neg_x_age = now + AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS;
    }
}

// retrieve latest calculated vibration levels
Vector3f AP_Ins::get_vibration_levels(uint8_t instance) const
{
    Vector3f vibe;
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        vibe = _accel_vibe_filter[instance].get();
        vibe.x = safe_sqrt(vibe.x);
        vibe.y = safe_sqrt(vibe.y);
        vibe.z = safe_sqrt(vibe.z);
    }
    return vibe;
}

// check for vibration movement. Return true if all axis show nearly zero movement
bool AP_Ins::is_still()
{
    Vector3f vibe = get_vibration_levels();
    return (vibe.x < _still_threshold) &&
           (vibe.y < _still_threshold) &&
           (vibe.z < _still_threshold);
}

// initialise and register accel calibrator
// called during the startup of accel cal
void AP_Ins::acal_init()
{
    // NOTE: these objects are never deallocated because the pre-arm checks force a reboot
    if (_acal == nullptr) {
        _acal = new AP_AccelCal;
    }
    if (_accel_calibrator == nullptr) {
        _accel_calibrator = new AccelCalibrator;
    }
}

// update accel calibrator
void AP_Ins::acal_update()
{
    if(_acal == nullptr) {
        return;
    }

    _acal->update();

    if (_acal->get_status() != ACCEL_CAL_NOT_STARTED) {
        _acal->cancel();
    }
}

/*
    Returns true if new valid trim values are available and passes them to reference vars
*/
bool AP_Ins::get_new_trim(float& trim_roll, float &trim_pitch)
{
    if (_new_trim) {
        trim_roll = _trim_roll;
        trim_pitch = _trim_pitch;
        _new_trim = false;
        return true;
    }
    return false;
}

/*
    Returns body fixed accelerometer level data averaged during accel calibration's first step
*/
bool AP_Ins::get_fixed_mount_accel_cal_sample(uint8_t sample_num, Vector3f& ret) const
{
    if (_accel_count <= (_acc_body_aligned-1) || _accel_calibrator[2].get_status() != ACCEL_CAL_SUCCESS || sample_num>=_accel_calibrator[2].get_num_samples_collected()) {
        return false;
    }
    _accel_calibrator[_acc_body_aligned-1].get_sample_corrected(sample_num, ret);
    if (_board_orientation == ROTATION_CUSTOM && _custom_rotation) {
        ret = *_custom_rotation * ret;
    } else {
        ret.rotate(_board_orientation);
    }
    return true;
}

/*
    Returns Primary accelerometer level data averaged during accel calibration's first step
*/
bool AP_Ins::get_primary_accel_cal_sample_avg(uint8_t sample_num, Vector3f& ret) const
{
    uint8_t count = 0;
    Vector3f avg = Vector3f(0,0,0);
    for (uint8_t i=0; i<MIN(_accel_count,2); i++) {
        if (_accel_calibrator[i].get_status() != ACCEL_CAL_SUCCESS || sample_num>=_accel_calibrator[i].get_num_samples_collected()) {
            continue;
        }
        Vector3f sample;
        _accel_calibrator[i].get_sample_corrected(sample_num, sample);
        avg += sample;
        count++;
    }
    if (count == 0) {
        return false;
    }
    avg /= count;
    ret = avg;
    if (_board_orientation == ROTATION_CUSTOM && _custom_rotation) {
        ret = *_custom_rotation * ret;
    } else {
        ret.rotate(_board_orientation);
    }
    return true;
}

/*
  perform a simple 1D accel calibration, returning mavlink result code
 */
MAV_RESULT AP_Ins::simple_accel_cal()
{
    uint8_t num_accels = MIN(get_accel_count(), INS_MAX_INSTANCES);
    Vector3f last_average;
    Vector3f new_accel_offset;
    Vector3f saved_offsets;
    Vector3f saved_scaling;
    bool converged;
    const float accel_convergence_limit = 0.05;
    Vector3f rotated_gravity(0, 0, -GRAVITY_MSS);
    
    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // record we are calibrating
    _calibrating = true;

    // flash leds to tell user to keep the IMU still
//    AP_Notify::flags.initialising = true;

//    hal.console->printf("Simple accel cal");

    /*
      we do the accel calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // get the rotated gravity vector which will need to be applied to the offsets
    rotated_gravity.rotate_inverse(saved_orientation);
    
    // save existing accel offsets
    for (uint8_t k=0; k<num_accels; k++) {
        saved_offsets[k] = _accel_offset[k];
        saved_scaling[k] = _accel_scale[k];
    }
    
    // remove existing accel offsets and scaling
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k] = Vector3f();
        _accel_scale[k] = Vector3f(1,1,1);
        new_accel_offset[k].zero();
        last_average[k].zero();
        converged[k] = false;
    }

    for (uint8_t c = 0; c < 5; c++) {
//        hal.scheduler->delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 10 seconds
    // if the accels are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 10*4 && num_converged < num_accels; j++) {
        Vector3f accel_sum, accel_avg, accel_diff;
        float diff_norm;
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

//        hal.console->printf("*");

        for (uint8_t k=0; k<num_accels; k++) {
            accel_sum[k].zero();
        }
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_accels; k++) {
                accel_sum[k] += get_accel(k);
            }
//            hal.scheduler->delay(5);
        }

        for (uint8_t k=0; k<num_accels; k++) {
            accel_avg[k] = accel_sum[k] / i;
            accel_diff[k] = last_average[k] - accel_avg[k];
            diff_norm[k] = accel_diff[k].length();
        }

        for (uint8_t k=0; k<num_accels; k++) {
            if (j > 0 && diff_norm[k] < accel_convergence_limit) {
                last_average[k] = (accel_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                if (!converged[k] || last_average[k].length() < new_accel_offset[k].length()) {
                    new_accel_offset[k] = last_average[k];
                }
                if (!converged[k]) {
                    converged[k] = true;
                    num_converged++;
                }
            } else {
                last_average[k] = accel_avg[k];
            }
        }
    }

    MAV_RESULT result = MAV_RESULT_ACCEPTED;

    // see if we've passed
    for (uint8_t k=0; k<num_accels; k++) {
        if (!converged[k]) {
            result = MAV_RESULT_FAILED;
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    if (result == MAV_RESULT_ACCEPTED) {
//        hal.console->printf("\nPASSED\n");
        for (uint8_t k=0; k<num_accels; k++) {
            // remove rotated gravity
            new_accel_offset[k] -= rotated_gravity;
            _accel_offset[k] = new_accel_offset[k];
//            _accel_scale[k].save();
//            _accel_id[k].save();
            _accel_id_ok[k] = true;
        }

        // force trim to zero
//        AP::ahrs().set_trim(Vector3f(0, 0, 0));
    } else {
//        hal.console->printf("\nFAILED\n");
        // restore old values
        for (uint8_t k=0; k<num_accels; k++) {
            _accel_offset[k] = saved_offsets[k];
            _accel_scale[k] = saved_scaling[k];
        }
    }

    // record calibration complete
    _calibrating = false;

    // throw away any existing samples that may have the wrong
    // orientation. We do this by throwing samples away for 0.5s,
    // which is enough time for the filters to settle
    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 500) {
        update();
    }

    // and reset state estimators
//    AP::ahrs().reset();

    // stop flashing leds
//    AP_Notify::flags.initialising = false;

    return result;
}


namespace AP {

AP_Ins &ins()
{
    return *AP_Ins::get_instance();
}

};
