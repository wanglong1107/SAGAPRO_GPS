#include <AP_HAL/AP_HAL.h>

#include "AP_Compass.h"
#include "AP_Compass_QMC5883L.h"
#include "AP_Compass_Backend.h"

#if 0
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/I2CDevice.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_Compass_SITL.h"
#include "AP_Compass_AK8963.h"
#include "AP_Compass_Backend.h"
#include "AP_Compass_BMM150.h"
#include "AP_Compass_HIL.h"
#include "AP_Compass_HMC5843.h"
#include "AP_Compass_IST8310.h"
#include "AP_Compass_LSM303D.h"
#include "AP_Compass_LSM9DS1.h"
#include "AP_Compass_LIS3MDL.h"
#include "AP_Compass_AK09916.h"
#include "AP_Compass_QMC5883L.h"
#if HAL_WITH_UAVCAN
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "AP_Compass_UAVCAN.h"
#endif
#include "AP_Compass_MMC3416.h"
#include "AP_Compass_MAG3110.h"
#endif



extern AP_HAL::HAL& hal;

//#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
//#define COMPASS_LEARN_DEFAULT Compass::LEARN_NONE
//#else
//#define COMPASS_LEARN_DEFAULT Compass::LEARN_INTERNAL
//#endif

#ifndef AP_COMPASS_OFFSETS_MAX_DEFAULT
#define AP_COMPASS_OFFSETS_MAX_DEFAULT 850
#endif

#ifndef HAL_COMPASS_FILTER_DEFAULT
 #define HAL_COMPASS_FILTER_DEFAULT 0 // turned off by default
#endif


// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    _compass_cal_autoreboot(false),
    _cal_complete_requires_reboot(false),
    _cal_has_run(false),
    _backend_count(0),
    _compass_count(0),
    _board_orientation(ROTATION_NONE),
    _custom_rotation(nullptr),
    _null_init_done(false),
    _hil_mode(false)
{
    if (_singleton != nullptr) {
        return;
    }
    _singleton = this;
//    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        _backends[i] = nullptr;
        _state[i].last_update_usec = 0;
    }

    // default device ids to zero.  init() method will overwrite with the actual device ids
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        _state[i].dev_id = 0;
    }
}

// Default init method
//
bool
Compass::init()
{
    if (_compass_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }
    if (_compass_count != 0) {
        // get initial health status
//        hal.scheduler->delay(100);
        read();
    }
    return true;
}

//  Register a new compass instance
//
uint8_t Compass::register_compass(void)
{
    if (_compass_count == COMPASS_MAX_INSTANCES) {
//        AP_HAL::panic("Too many compass instances");
    }
    return _compass_count++;
}

bool Compass::_add_backend(AP_Compass_Backend *backend, const char *name, bool external)
{
    if (!backend) {
        return false;
    }

//    if (_backend_count == COMPASS_MAX_BACKEND) {
//        AP_HAL::panic("Too many compass backends");
//    }

    _backends[_backend_count++] = backend;

    return true;
}

/*
  return true if a driver type is enabled
 */
bool Compass::_driver_enabled(enum DriverType driver_type)
{
    uint32_t mask = (1U<<uint8_t(driver_type));
    return (mask & uint32_t(_driver_type_mask)) == 0;
}

/*
  see if we already have probed a driver by bus type
 */
//bool Compass::_have_driver(AP_HAL::Device::BusType bus_type, uint8_t bus_num, uint8_t address, uint8_t devtype) const
//{
//    uint32_t id = AP_HAL::Device::make_bus_id(bus_type, bus_num, address, devtype);
//    for (uint8_t i=0; i<_compass_count; i++) {
//        if (id == uint32_t(_state[i].dev_id.get())) {
//            return true;
//        }
//    }
//    return false;
//}

/*
  detect available backends for this board
 */
void Compass::_detect_backends(void)
{
//    if (_hil_mode) {
//        _add_backend(AP_Compass_HIL::detect(*this), nullptr, false);
//        return;
//    }

#if AP_FEATURE_BOARD_DETECT
    if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2) {
        // default to disabling LIS3MDL on pixhawk2 due to hardware issue
        _driver_type_mask.set_default(1U<<DRIVER_LIS3MDL);
    }
#endif
    
/*
  macro to add a backend with check for too many backends or compass
  instances. We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(driver_type, backend, name, external)   \
    do { if (_driver_enabled(driver_type)) { _add_backend(backend, name, external); } \
       if (_backend_count == COMPASS_MAX_BACKEND || \
           _compass_count == COMPASS_MAX_INSTANCES) { \
          return; \
        } \
    } while (0)

    

	//external i2c bus
//	ADD_BACKEND(DRIVER_QMC5883, AP_Compass_QMC5883L::probe(*this, hal.i2c_mgr->get_device(1, HAL_COMPASS_QMC5883L_I2C_ADDR),
//											true,ROTATION_ROLL_180), AP_Compass_QMC5883L::name, true);


    if (_backend_count == 0 ||
        _compass_count == 0) {
//        hal.console->printf("No Compass backends available\n");
    }
}

void
Compass::accumulate(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call accumulate on each of the backend
        _backends[i]->accumulate();
    }
}

bool
Compass::read(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }
    uint32_t time = AP_HAL::millis();
    for (uint8_t i=0; i < COMPASS_MAX_INSTANCES; i++) {
        _state[i].healthy = (time - _state[i].last_update_ms < 500);
    }
    return healthy();
}

uint8_t
Compass::get_healthy_mask() const
{
    uint8_t healthy_mask = 0;
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if(healthy(i)) {
            healthy_mask |= 1 << i;
        }
    }
    return healthy_mask;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset = offsets;
    }
}

void Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset = offsets;
        save_offsets(i);
    }
}

void
Compass::set_and_save_diagonals(uint8_t i, const Vector3f &diagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].diagonals = diagonals;
    }
}

void
Compass::set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offdiagonals = offdiagonals;
    }
}

void
Compass::save_offsets(uint8_t i)
{
//    _state[i].offset.save();  // save offsets
//    _state[i].dev_id.save();  // save device id corresponding to these offsets
}

void
Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(i);
    }
}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    _state[i].motor_compensation = motor_comp_factor;
}

void
Compass::save_motor_compensation()
{
//    _motor_comp_type.save();
//    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
//        _state[k].motor_compensation.save();
//    }
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
    if (_auto_declination)
	{
        // Set the declination based on the lat/lng from GPS
		_declination = radians( AP_Declination::get_declination( (float)latitude / 10000000, (float)longitude / 10000000));
    }
}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
    uint8_t prim = get_primary();
    return healthy(prim) && use_for_yaw(prim);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    // when we are doing in-flight compass learning the state
    // estimator must not use the compass. The learning code turns off
    // inflight learning when it has converged
    return _state[i].use_for_yaw && _learn != LEARN_INFLIGHT;
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination = radians;
    }else{
        _declination = radians;
    }
}

float
Compass::get_declination() const
{
    return _declination;
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float
Compass::calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    const Vector3f &field = get_field(i);

    float headY = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * M_PI);
        else if (heading < -M_PI)
            heading += (2.0f * M_PI);
    }

    return heading;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (is_zero(get_offsets(i).length())) {
        return false;
    }

    // backup detected dev_id
    int32_t dev_id_orig = _state[i].dev_id;

    // load dev_id from eeprom
//    _state[i].dev_id.load();

    // if different then the device has not been configured
    if (_state[i].dev_id != dev_id_orig) {
        // restore device id
        _state[i].dev_id = dev_id_orig;
        // return failure
        return false;
    }

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && (!use_for_yaw(i) || configured(i));
    }
    return all_configured;
}

// Update raw magnetometer values from HIL data
//
//void Compass::setHIL(uint8_t instance, float roll, float pitch, float yaw)
//{
//    Matrix3f R;

//    // create a rotation matrix for the given attitude
//    R.from_euler(roll, pitch, yaw);

//    if (!is_equal(_hil.last_declination,get_declination())) {
//        _setup_earth_field();
//        _hil.last_declination = get_declination();
//    }

//    // convert the earth frame magnetic vector to body frame, and
//    // apply the offsets
//    _hil.field[instance] = R.mul_transpose(_hil.Bearth);

//    // apply default board orientation for this compass type. This is
//    // a noop on most boards
//    _hil.field[instance].rotate(MAG_BOARD_ORIENTATION);

//    // add user selectable orientation
//    _hil.field[instance].rotate((enum Rotation)_state[0].orientation.get());

//    if (!_state[0].external) {
//        // and add in AHRS_ORIENTATION setting if not an external compass
//        if (_board_orientation == ROTATION_CUSTOM && _custom_rotation) {
//            _hil.field[instance] = *_custom_rotation * _hil.field[instance];
//        } else {
//            _hil.field[instance].rotate(_board_orientation);
//        }
//    }
//    _hil.healthy[instance] = true;
//}

//// Update raw magnetometer values from HIL mag vector
////
//void Compass::setHIL(uint8_t instance, const Vector3f &mag, uint32_t update_usec)
//{
//    _hil.field[instance] = mag;
//    _hil.healthy[instance] = true;
//    _state[instance].last_update_usec = update_usec;
//}

//const Vector3f& Compass::getHIL(uint8_t instance) const
//{
//    return _hil.field[instance];
//}

//// setup _Bearth
//void Compass::_setup_earth_field(void)
//{
//    // assume a earth field strength of 400
//    _hil.Bearth(400, 0, 0);

//    // rotate _Bearth for inclination and declination. -66 degrees
//    // is the inclination in Canberra, Australia
//    Matrix3f R;
//    R.from_euler(0, ToRad(66), get_declination());
//    _hil.Bearth = R * _hil.Bearth;
//}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{
    if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
        _motor_comp_type = (int8_t)comp_type;
        _thr = 0; // set current  throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}

bool Compass::consistent() const
{
    Vector3f primary_mag_field = get_field();
    Vector3f primary_mag_field_norm;

    if (!primary_mag_field.is_zero()) {
        primary_mag_field_norm = primary_mag_field.normalized();
    } else {
        return false;
    }

    Vector2f primary_mag_field_xy = Vector2f(primary_mag_field.x,primary_mag_field.y);
    Vector2f primary_mag_field_xy_norm;

    if (!primary_mag_field_xy.is_zero()) {
        primary_mag_field_xy_norm = primary_mag_field_xy.normalized();
    } else {
        return false;
    }

    for (uint8_t i=0; i<get_count(); i++) {
        if (use_for_yaw(i)) {
            Vector3f mag_field = get_field(i);
            Vector3f mag_field_norm;

            if (!mag_field.is_zero()) {
                mag_field_norm = mag_field.normalized();
            } else {
                return false;
            }

            Vector2f mag_field_xy = Vector2f(mag_field.x,mag_field.y);
            Vector2f mag_field_xy_norm;

            if (!mag_field_xy.is_zero()) {
                mag_field_xy_norm = mag_field_xy.normalized();
            } else {
                return false;
            }

            float xyz_ang_diff = acosf(constrain_float(mag_field_norm * primary_mag_field_norm,-1.0f,1.0f));
            float xy_ang_diff  = acosf(constrain_float(mag_field_xy_norm*primary_mag_field_xy_norm,-1.0f,1.0f));
            float xy_len_diff  = (primary_mag_field_xy-mag_field_xy).length();

            // check for gross misalignment on all axes
            bool xyz_ang_diff_large = xyz_ang_diff > AP_COMPASS_MAX_XYZ_ANG_DIFF;

            // check for an unacceptable angle difference on the xy plane
            bool xy_ang_diff_large = xy_ang_diff > AP_COMPASS_MAX_XY_ANG_DIFF;

            // check for an unacceptable length difference on the xy plane
            bool xy_length_diff_large = xy_len_diff > AP_COMPASS_MAX_XY_LENGTH_DIFF;

            // check for inconsistency in the XY plane
            if (xyz_ang_diff_large || xy_ang_diff_large || xy_length_diff_large) {
                return false;
            }
        }
    }
    return true;
}


// singleton instance
Compass *Compass::_singleton;

namespace AP {

Compass &compass()
{
    return *Compass::get_singleton();
}

}
