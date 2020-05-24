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

#include "RangeFinder.h"
#include "AP_RangeFinder_NMEA.h"

//#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;


RangeFinder::RangeFinder() :
    num_instances(0),
    estimated_terrain_height(0)
{
//    AP_Param::setup_object_defaults(this, var_info);

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        state[i].orientation = ROTATION_NONE;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
//        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
//void RangeFinder::init(void)
void RangeFinder::init(AP_HAL::UARTDriver *serial_port)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
	serial_manager[0] = &(serial_port[0]);
	
    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
//        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)state[instance].type;
    switch (_type) {

    case RangeFinder_TYPE_NMEA:
		drivers[instance] = new AP_RangeFinder_NMEA(state[instance], serial_instance++);
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
//    if (drivers[instance] && state[instance].var_info) {
//        backend_var_info[instance] = state[instance].var_info;
//        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
//    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

//void RangeFinder::handle_msg(mavlink_message_t *msg)
//{
//    uint8_t i;
//    for (i=0; i<num_instances; i++) {
//        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE)) {
//          drivers[i]->handle_msg(msg);
//        }
//    }
//}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

//MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
//{
//    AP_RangeFinder_Backend *backend = find_instance(orientation);
//    if (backend == nullptr) {
//        return MAV_DISTANCE_SENSOR_UNKNOWN;
//    }
//    return backend->get_mav_distance_sensor_type();
//}

RangeFinder *RangeFinder::_singleton;
