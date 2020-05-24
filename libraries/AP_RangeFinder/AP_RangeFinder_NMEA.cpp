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

#include <AP_HAL/AP_HAL.h>
//#include "AP_RangeFinder_LightWareSerial.h"
//#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NMEA.h"

extern const AP_HAL::HAL& hal;

// constructor initialises the rangefinder
// Note this is called after detect() returns true, so we
// already know that we should setup the rangefinder
AP_RangeFinder_NMEA::AP_RangeFinder_NMEA(RangeFinder::RangeFinder_State &_state,
                                         uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state),
    _distance_m(-1.0f)
{
//    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
//    if (uart != nullptr) {
//        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
//    }
}

// detect if a NMEA rangefinder by looking to see if the user has configured it
//bool AP_RangeFinder_NMEA::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
//{
//    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
//}

// update the state of the sensor
void AP_RangeFinder_NMEA::update(void)
{
    uint32_t now = AP_HAL::millis();

}

// return last value measured by sensor
bool AP_RangeFinder_NMEA::get_reading(uint16_t &reading_cm)
{

    return true;
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_RangeFinder_NMEA::decode(char c)
{

    return true;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_RangeFinder_NMEA::decode_latest_term()
{

    return true;
}
