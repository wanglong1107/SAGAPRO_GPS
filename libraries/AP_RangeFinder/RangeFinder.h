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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
//#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/UARTDriver.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2
#define RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT 10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50

class AP_RangeFinder_Backend;

class RangeFinder
{
    friend class AP_RangeFinder_Backend;

public:
    RangeFinder();

    /* Do not allow copies */
    RangeFinder(const RangeFinder &other) = delete;
    RangeFinder &operator=(const RangeFinder&) = delete;

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5,
        RangeFinder_TYPE_BBB_PRU= 6,
        RangeFinder_TYPE_LWI2C  = 7,
        RangeFinder_TYPE_LWSER  = 8,
        RangeFinder_TYPE_BEBOP  = 9,
        RangeFinder_TYPE_MAVLink = 10,
        RangeFinder_TYPE_ULANDING= 11,
        RangeFinder_TYPE_LEDDARONE = 12,
        RangeFinder_TYPE_MBSER  = 13,
        RangeFinder_TYPE_TRI2C  = 14,
        RangeFinder_TYPE_PLI2CV3= 15,
        RangeFinder_TYPE_VL53L0X = 16,
        RangeFinder_TYPE_NMEA = 17,
        RangeFinder_TYPE_WASP = 18,
        RangeFinder_TYPE_BenewakeTF02 = 19,
        RangeFinder_TYPE_BenewakeTFmini = 20
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        enum RangeFinder_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks

        int8_t  type;
        int8_t  pin;
        int8_t  ratiometric;
        int8_t  stop_pin;
        int16_t settle_time_ms;
        float scaling;
        float offset;
        int8_t  function;
        int16_t min_distance_cm;
        int16_t max_distance_cm;
        int8_t  ground_clearance_cm;
        int8_t  address;
        Vector3f pos_offset; // position offset in body frame
        int8_t  orientation;
//        const struct AP_Param::GroupInfo *var_info;
    };

//static const struct AP_Param::GroupInfo *backend_var_info[RANGEFINDER_MAX_INSTANCES];
    
    int16_t _powersave_range;

    // parameters for each instance
//    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available rangefinders
    //void init(void);
	void init(AP_HAL::UARTDriver *serial_port);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
//    void handle_msg(mavlink_message_t *msg);

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    uint16_t voltage_mv_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
//    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder_Status status_orient(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    /*
      returns true if pre-arm checks have passed for all range finders
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;

    static RangeFinder *get_singleton(void) { return _singleton; }


private:
    static RangeFinder *_singleton;

    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t num_instances:3;
    float estimated_terrain_height;

	AP_HAL::UARTDriver *serial_manager[RANGEFINDER_MAX_INSTANCES];

    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void detect_instance(uint8_t instance, uint8_t& serial_instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(AP_RangeFinder_Backend *driver);
};