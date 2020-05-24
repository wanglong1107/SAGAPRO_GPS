/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 */

//#include <AP_ADC/AP_ADC.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
//#include <GCS_MAVLink/GCS.h>
//#include <SRV_Channel/SRV_Channel.h>
#include <utility>
#include "AP_Airspeed.h"
//#include "AP_Airspeed_MS4525.h"
//#include "AP_Airspeed_MS5525.h"
//#include "AP_Airspeed_SDP3X.h"
#include "AP_Airspeed_analog.h"
#include "AP_Airspeed_Backend.h"

extern const AP_HAL::HAL &hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define ARSPD_DEFAULT_TYPE TYPE_ANALOG
 #define ARSPD_DEFAULT_PIN 1
#else
 #define ARSPD_DEFAULT_TYPE TYPE_I2C_MS4525
 #define ARSPD_DEFAULT_PIN 15
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define PSI_RANGE_DEFAULT 0.05
#endif

#ifndef PSI_RANGE_DEFAULT
#define PSI_RANGE_DEFAULT 1.0f
#endif



AP_Airspeed::AP_Airspeed()
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        state[i].EAS2TAS = 1;
    }
//    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
//        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
}


/*
  this scaling factor converts from the old system where we used a
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

void AP_Airspeed::init()
{
    // cope with upgrade from old system
    if (param[0].pin && param[0].pin != 65) {
        param[0].type = TYPE_ANALOG ;
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        state[i].calibration.init(param[i].ratio);
        state[i].last_saved_ratio = param[i].ratio;

        switch ((enum airspeed_type)param[i].type) {
        case TYPE_NONE:
            // nothing to do
            break;
        case TYPE_ANALOG:
            sensor[i] = new AP_Airspeed_Analog(*this, i);
            break;
        }
//        if (sensor[i] && !sensor[i]->init()) {
//            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] init failed", i);
//            delete sensor[i];
//            sensor[i] = nullptr;
//        }
    }
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(uint8_t i)
{
    if (!enabled(i)) {
        return 0;
    }
    if (state[i].hil_set) {
        state[i].healthy = true;
        return state[i].hil_pressure;
    }
    float pressure = 0;
    if (sensor[i]) {
        state[i].healthy = sensor[i]->get_differential_pressure(pressure);
    }
    return pressure;
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(uint8_t i, float &temperature)
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_temperature(temperature);
    }
    return false;
}

// calibrate the zero offset for the airspeed. This must be called at
// least once before the get_airspeed() interface can be used
void AP_Airspeed::calibrate(bool in_startup)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        if (state[i].use_zero_offset) {
            param[i].offset = 0;
            continue;
        }
        if (in_startup && param[i].skip_cal) {
            continue;
        }
        state[i].cal.start_ms = AP_HAL::millis();
        state[i].cal.count = 0;
        state[i].cal.sum = 0;
        state[i].cal.read_count = 0;
    }
//    gcs().send_text(MAV_SEVERITY_INFO,"Airspeed calibration started");
}

/*
  update async airspeed zero offset calibration
*/
void AP_Airspeed::update_calibration(uint8_t i, float raw_pressure)
{
    if (!enabled(i) || state[i].cal.start_ms == 0) {
        return;
    }
    
    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - state[i].cal.start_ms >= 1000 &&
        state[i].cal.read_count > 15) {
        if (state[i].cal.count == 0) {
//            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] sensor unhealthy", i);
        } else {
//            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] sensor calibrated", i);
            param[i].offset = state[i].cal.sum / state[i].cal.count;
        }
        state[i].cal.start_ms = 0;
        return;
    }
    // we discard the first 5 samples
    if (state[i].healthy && state[i].cal.read_count > 5) {
        state[i].cal.sum += raw_pressure;
        state[i].cal.count++;
    }
    state[i].cal.read_count++;
}

// read one airspeed sensor
void AP_Airspeed::read(uint8_t i)
{
    float airspeed_pressure;
    if (!enabled(i) || !sensor[i]) {
        return;
    }
    bool prev_healthy = state[i].healthy;
    float raw_pressure = get_pressure(i);
    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }
    
    airspeed_pressure = raw_pressure - param[i].offset;

    // remember raw pressure for logging
    state[i].corrected_pressure = airspeed_pressure;

    // filter before clamping positive
    if (!prev_healthy) {
        // if the previous state was not healthy then we should not
        // use an IIR filter, otherwise a bad reading will last for
        // some time after the sensor becomees healthy again
        state[i].filtered_pressure = airspeed_pressure;
    } else {
        state[i].filtered_pressure = 0.7f * state[i].filtered_pressure + 0.3f * airspeed_pressure;
    }

    /*
      we support different pitot tube setups so user can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)param[i].tube_order) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        state[i].last_pressure  = -airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(-airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(-state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_POSITIVE:
        state[i].last_pressure  = airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(state[i].filtered_pressure, 0) * param[i].ratio);
        if (airspeed_pressure < -32) {
            // we're reading more than about -8m/s. The user probably has
            // the ports the wrong way around
            state[i].healthy = false;
        }
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        state[i].last_pressure  = fabsf(airspeed_pressure);
        state[i].raw_airspeed   = sqrtf(fabsf(airspeed_pressure) * param[i].ratio);
        state[i].airspeed       = sqrtf(fabsf(state[i].filtered_pressure) * param[i].ratio);
        break;
    }

    state[i].last_update_ms = AP_HAL::millis();
}

// read all airspeed sensors
void AP_Airspeed::read(void)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

#if 1
    // debugging until we get MAVLink support for 2nd airspeed sensor
    if (enabled(1)) {
//        gcs().send_named_float("AS2", get_airspeed(1));
    }
#endif

    // setup primary
    if (healthy(primary_sensor)) {
        primary = primary_sensor;
        return;
    }
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (healthy(i)) {
            primary = i;
            break;
        }
    }
}

void AP_Airspeed::setHIL(float airspeed, float diff_pressure, float temperature)
{
    state[0].raw_airspeed = airspeed;
    state[0].airspeed = airspeed;
    state[0].last_pressure = diff_pressure;
    state[0].last_update_ms = AP_HAL::millis();
    state[0].hil_pressure = diff_pressure;
    state[0].hil_set = true;
    state[0].healthy = true;
}

bool AP_Airspeed::use(uint8_t i) const
{
    if (!enabled(i) || !param[i].use) {
        return false;
    }
//    if (param[i].use == 2 && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) != 0) {
//        // special case for gliders with airspeed sensors behind the
//        // propeller. Allow airspeed to be disabled when throttle is
//        // running
//        return false;
//    }
    return true;
}

/*
  return true if all enabled sensors are healthy
 */
bool AP_Airspeed::all_healthy(void) const
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (enabled(i) && !healthy(i)) {
            return false;
        }
    }
    return true;
}

// singleton instance
AP_Airspeed *AP_Airspeed::_singleton;

