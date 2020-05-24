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
  IMU driver backend class. Each supported gyro/accel sensor type
  needs to have an object derived from this class.

  Note that drivers can implement just gyros or just accels, and can
  also provide multiple gyro/accel instances.
 */
#pragma once


#include <inttypes.h>

#include <AP_Math/AP_Math.h>

const extern AP_HAL::HAL& hal;

class AP_Ins_Backend
{
public:
    AP_Ins_Backend();
    AP_Ins_Backend(const AP_Ins_Backend &that) = delete;

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Ins_Backend(void);

	virtual void start() = 0;
    virtual bool update() = 0;
    
    
    /*
      device driver IDs. These are used to fill in the devtype field
      of the device ID, which shows up as INS*ID* parameters to
      users. The values are chosen for compatibility with existing PX4
      drivers.
      If a change is made to a driver that would make existing
      calibration values invalid then this number must be changed.
     */
    enum DevTypes {
        DEVTYPE_BMI160       = 0x09,
        DEVTYPE_L3G4200D     = 0x10,
        DEVTYPE_ACC_LSM303D  = 0x11,
        DEVTYPE_ACC_BMA180   = 0x12,
        DEVTYPE_ACC_MPU6000  = 0x13,
        DEVTYPE_ACC_MPU9250  = 0x16,
        DEVTYPE_ACC_IIS328DQ = 0x17,
        DEVTYPE_ACC_LSM9DS1  = 0x18,
        DEVTYPE_GYR_MPU6000  = 0x21,
        DEVTYPE_GYR_L3GD20   = 0x22,
        DEVTYPE_GYR_MPU9250  = 0x24,
        DEVTYPE_GYR_I3G4250D = 0x25,
        DEVTYPE_GYR_LSM9DS1  = 0x26,
        DEVTYPE_INS_ICM20789 = 0x27,
        DEVTYPE_INS_ICM20689 = 0x28,
        DEVTYPE_INS_BMI055   = 0x29,
    };

protected:


private:

};
