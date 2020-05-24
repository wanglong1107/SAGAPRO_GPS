#include <AP_HAL/AP_HAL.h>
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2_core.h"
//#include <AP_Vehicle/AP_Vehicle.h>
//#include <GCS_MAVLink/GCS.h>
//#include <DataFlash/DataFlash.h>
#include <new>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built.
 */
//#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Replay)
// copter defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    100

//#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
//// rover defaults
//#define VELNE_M_NSE_DEFAULT     0.5f
//#define VELD_M_NSE_DEFAULT      0.7f
//#define POSNE_M_NSE_DEFAULT     1.0f
//#define ALT_M_NSE_DEFAULT       3.0f
//#define MAG_M_NSE_DEFAULT       0.05f
//#define GYRO_P_NSE_DEFAULT      3.0E-02f
//#define ACC_P_NSE_DEFAULT       6.0E-01f
//#define GBIAS_P_NSE_DEFAULT     1.0E-04f
//#define GSCALE_P_NSE_DEFAULT    5.0E-04f
//#define ABIAS_P_NSE_DEFAULT     5.0E-03f
//#define MAGB_P_NSE_DEFAULT      1.0E-04f
//#define MAGE_P_NSE_DEFAULT      1.0E-03f
//#define VEL_I_GATE_DEFAULT      500
//#define POS_I_GATE_DEFAULT      500
//#define HGT_I_GATE_DEFAULT      500
//#define MAG_I_GATE_DEFAULT      300
//#define MAG_CAL_DEFAULT         2
//#define GLITCH_RADIUS_DEFAULT   25
//#define FLOW_MEAS_DELAY         10
//#define FLOW_M_NSE_DEFAULT      0.25f
//#define FLOW_I_GATE_DEFAULT     300
//#define CHECK_SCALER_DEFAULT    100

//#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
//// plane defaults
//#define VELNE_M_NSE_DEFAULT     0.5f
//#define VELD_M_NSE_DEFAULT      0.7f
//#define POSNE_M_NSE_DEFAULT     1.0f
//#define ALT_M_NSE_DEFAULT       3.0f
//#define MAG_M_NSE_DEFAULT       0.05f
//#define GYRO_P_NSE_DEFAULT      3.0E-02f
//#define ACC_P_NSE_DEFAULT       6.0E-01f
//#define GBIAS_P_NSE_DEFAULT     1.0E-04f
//#define GSCALE_P_NSE_DEFAULT    5.0E-04f
//#define ABIAS_P_NSE_DEFAULT     5.0E-03f
//#define MAGB_P_NSE_DEFAULT      1.0E-04f
//#define MAGE_P_NSE_DEFAULT      1.0E-03f
//#define VEL_I_GATE_DEFAULT      500
//#define POS_I_GATE_DEFAULT      500
//#define HGT_I_GATE_DEFAULT      500
//#define MAG_I_GATE_DEFAULT      300
//#define MAG_CAL_DEFAULT         0
//#define GLITCH_RADIUS_DEFAULT   25
//#define FLOW_MEAS_DELAY         10
//#define FLOW_M_NSE_DEFAULT      0.25f
//#define FLOW_I_GATE_DEFAULT     300
//#define CHECK_SCALER_DEFAULT    150

//#else
//// build type not specified, use copter defaults
//#define VELNE_M_NSE_DEFAULT     0.5f
//#define VELD_M_NSE_DEFAULT      0.7f
//#define POSNE_M_NSE_DEFAULT     1.0f
//#define ALT_M_NSE_DEFAULT       3.0f
//#define MAG_M_NSE_DEFAULT       0.05f
//#define GYRO_P_NSE_DEFAULT      3.0E-02f
//#define ACC_P_NSE_DEFAULT       6.0E-01f
//#define GBIAS_P_NSE_DEFAULT     1.0E-04f
//#define GSCALE_P_NSE_DEFAULT    5.0E-04f
//#define ABIAS_P_NSE_DEFAULT     5.0E-03f
//#define MAGB_P_NSE_DEFAULT      1.0E-04f
//#define MAGE_P_NSE_DEFAULT      1.0E-03f
//#define VEL_I_GATE_DEFAULT      500
//#define POS_I_GATE_DEFAULT      500
//#define HGT_I_GATE_DEFAULT      500
//#define MAG_I_GATE_DEFAULT      300
//#define MAG_CAL_DEFAULT         3
//#define GLITCH_RADIUS_DEFAULT   25
//#define FLOW_MEAS_DELAY         10
//#define FLOW_M_NSE_DEFAULT      0.25f
//#define FLOW_I_GATE_DEFAULT     300
//#define CHECK_SCALER_DEFAULT    100

//#endif // APM_BUILD_DIRECTORY

extern const AP_HAL::HAL& hal;



NavEKF2::NavEKF2(const AP_AHRS *ahrs, const RangeFinder &rng) :
    _ahrs(ahrs),
    _rng(rng)
{
//    AP_Param::setup_object_defaults(this, var_info);
}

/*
  see if we should log some sensor data
 */
void NavEKF2::check_log_write(void)
{
    if (!have_ekf_logging()) {
        return;
    }
    if (logging.log_compass) {
//        DataFlash_Class::instance()->Log_Write_Compass(*_ahrs->get_compass(), imuSampleTime_us);
        logging.log_compass = false;
    }
    if (logging.log_gps) {
//        DataFlash_Class::instance()->Log_Write_GPS(AP::gps().primary_sensor(), imuSampleTime_us);
        logging.log_gps = false;
    }
    if (logging.log_baro) {
//        DataFlash_Class::instance()->Log_Write_Baro(imuSampleTime_us);
        logging.log_baro = false;
    }
    if (logging.log_imu) {
//        DataFlash_Class::instance()->Log_Write_IMUDT(imuSampleTime_us, _logging_mask.get());
        logging.log_imu = false;
    }

    // this is an example of an ad-hoc log in EKF
    // DataFlash_Class::instance()->Log_Write("NKA", "TimeUS,X", "Qf", AP_HAL::micros64(), (double)2.4f);
}


// Initialise the filter
bool NavEKF2::InitialiseFilter(void)
{
    if (_enable == 0) {
        return false;
    }
    const AP_InertialSensor &ins = AP::ins();

    imuSampleTime_us = AP_HAL::micros64();

    // remember expected frame time
    _frameTimeUsec = 1e6 / ins.get_sample_rate();

    // expected number of IMU frames per prediction
    _framesPerPrediction = uint8_t((EKF_TARGET_DT / (_frameTimeUsec * 1.0e-6) + 0.5));

    // see if we will be doing logging
//    DataFlash_Class *dataflash = DataFlash_Class::instance();
//    if (dataflash != nullptr) {
//        logging.enabled = dataflash->log_replay();
//    }
    
    if (core == nullptr) {

        // don't run multiple filters for 1 IMU
        uint8_t mask = (1U<<ins.get_accel_count())-1;
        _imuMask = _imuMask & mask;
        
        // count IMUs from mask
        num_cores = 0;
        for (uint8_t i=0; i<7; i++) {
            if (_imuMask & (1U<<i)) {
                num_cores++;
            }
        }

        // check if there is enough memory to create the EKF cores
//        if (hal.util->available_memory() < sizeof(NavEKF2_core)*num_cores + 4096) {
////            gcs().send_text(MAV_SEVERITY_CRITICAL, "NavEKF2: not enough memory");
//            _enable.set(0);
//            return false;
//        }

        // try to allocate from CCM RAM, fallback to Normal RAM if not available or full
//        core = (NavEKF2_core*)hal.util->malloc_type(sizeof(NavEKF2_core)*num_cores, AP_HAL::Util::MEM_FAST);
//        if (core == nullptr) {
//            _enable.set(0);
////            gcs().send_text(MAV_SEVERITY_CRITICAL, "NavEKF2: allocation failed");
//            return false;
//        }
        for (uint8_t i = 0; i < num_cores; i++) {
            //Call Constructors
            new (&core[i]) NavEKF2_core();
        }

        // set the IMU index for the cores
        num_cores = 0;
        for (uint8_t i=0; i<7; i++) {
            if (_imuMask & (1U<<i)) {
                if(!core[num_cores].setup_core(this, i, num_cores)) {
                    return false;
                }
                num_cores++;
            }
        }

        // Set the primary initially to be the lowest index
        primary = 0;
    }

    // initialise the cores. We return success only if all cores
    // initialise successfully
    bool ret = true;
    for (uint8_t i=0; i<num_cores; i++) {
        ret &= core[i].InitialiseFilterBootstrap();
    }

    // zero the structs used capture reset events
    memset(&yaw_reset_data, 0, sizeof(yaw_reset_data));
    memset(&pos_reset_data, 0, sizeof(pos_reset_data));
    memset(&pos_down_reset_data, 0, sizeof(pos_down_reset_data));

    check_log_write();
    return ret;
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2::UpdateFilter(void)
{
    if (!core) {
        return;
    }

    imuSampleTime_us = AP_HAL::micros64();
    
    const AP_InertialSensor &ins = AP::ins();

    bool statePredictEnabled[num_cores];
    for (uint8_t i=0; i<num_cores; i++) {
        // if we have not overrun by more than 3 IMU frames, and we
        // have already used more than 1/3 of the CPU budget for this
        // loop then suppress the prediction step. This allows
        // multiple EKF instances to cooperate on scheduling
        if (core[i].getFramesSincePredict() < (_framesPerPrediction+3) &&
            (AP_HAL::micros() - ins.get_last_update_usec()) > _frameTimeUsec/3) {
            statePredictEnabled[i] = false;
        } else {
            statePredictEnabled[i] = true;
        }
        core[i].UpdateFilter(statePredictEnabled[i]);
    }

    // If the current core selected has a bad error score or is unhealthy, switch to a healthy core with the lowest fault score
    // Don't start running the check until the primary core has started returned healthy for at least 10 seconds to avoid switching
    // due to initial alignment fluctuations and race conditions
    if (!runCoreSelection) {
        static uint64_t lastUnhealthyTime_us = 0;
        if (!core[primary].healthy() || lastUnhealthyTime_us == 0) {
            lastUnhealthyTime_us = imuSampleTime_us;
        }
        runCoreSelection = (imuSampleTime_us - lastUnhealthyTime_us) > 1E7;
    }
    float primaryErrorScore = core[primary].errorScore();
    if ((primaryErrorScore > 1.0f || !core[primary].healthy()) && runCoreSelection) {
        float lowestErrorScore = 0.67f * primaryErrorScore;
        uint8_t newPrimaryIndex = primary; // index for new primary
        for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {

            if (coreIndex != primary) {
                // an alternative core is available for selection only if healthy and if states have been updated on this time step
                bool altCoreAvailable = core[coreIndex].healthy() && statePredictEnabled[coreIndex];

                // If the primary core is unhealthy and another core is available, then switch now
                // If the primary core is still healthy,then switching is optional and will only be done if
                // a core with a significantly lower error score can be found
                float altErrorScore = core[coreIndex].errorScore();
                if (altCoreAvailable && (!core[primary].healthy() || altErrorScore < lowestErrorScore)) {
                    newPrimaryIndex = coreIndex;
                    lowestErrorScore = altErrorScore;
                }
            }
        }
        // update the yaw and position reset data to capture changes due to the lane switch
        if (newPrimaryIndex != primary) {
            updateLaneSwitchYawResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosDownResetData(newPrimaryIndex, primary);
            primary = newPrimaryIndex;
        }
    }

    check_log_write();
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF2::healthy(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].healthy();
}

// returns the index of the primary core
// return -1 if no primary core selected
int8_t NavEKF2::getPrimaryCoreIndex(void) const
{
    if (!core) {
        return -1;
    }
    return primary;
}

// returns the index of the IMU of the primary core
// return -1 if no primary core selected
int8_t NavEKF2::getPrimaryCoreIMUIndex(void) const
{
    if (!core) {
        return -1;
    }
    return core[primary].getIMUIndex();
}

// Write the last calculated NE position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosNE(int8_t instance, Vector2f &posNE) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getPosNE(posNE);
}

// Write the last calculated D position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosD(int8_t instance, float &posD) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getPosD(posD);
}

// return NED velocity in m/s
void NavEKF2::getVelNED(int8_t instance, Vector3f &vel) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getVelNED(vel);
    }
}

// Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
float NavEKF2::getPosDownDerivative(int8_t instance) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    // return the value calculated from a complementary filer applied to the EKF height and vertical acceleration
    if (core) {
        return core[instance].getPosDownDerivative();
    }
    return 0.0f;
}

// This returns the specific forces in the NED frame
void NavEKF2::getAccelNED(Vector3f &accelNED) const
{
    if (core) {
        core[primary].getAccelNED(accelNED);
    }
}

// return body axis gyro bias estimates in rad/sec
void NavEKF2::getGyroBias(int8_t instance, Vector3f &gyroBias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroBias(gyroBias);
    }
}

// return body axis gyro scale factor error as a percentage
void NavEKF2::getGyroScaleErrorPercentage(int8_t instance, Vector3f &gyroScale) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroScaleErrorPercentage(gyroScale);
    }
}

// return tilt error convergence metric for the specified instance
void NavEKF2::getTiltError(int8_t instance, float &ang) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getTiltError(ang);
    }
}

// reset body axis gyro bias estimates
void NavEKF2::resetGyroBias(void)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].resetGyroBias();
        }
    }
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool NavEKF2::resetHeightDatum(void)
{
    bool status = true;
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            if (!core[i].resetHeightDatum()) {
                status = false;
            }
        }
    } else {
        status = false;
    }
    return status;
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming as it will only be actioned when the filter is in static mode
// This command is forgotten by the EKF each time it goes back into static mode (eg the vehicle disarms)
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF2::setInhibitGPS(void)
{
    if (!core) {
        return 0;
    }
    return core[primary].setInhibitGPS();
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF2::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (core) {
        core[primary].getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    }
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF2::getAccelZBias(int8_t instance, float &zbias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getAccelZBias(zbias);
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF2::getWind(int8_t instance, Vector3f &wind) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getWind(wind);
    }
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF2::getMagNED(int8_t instance, Vector3f &magNED) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF2::getMagXYZ(int8_t instance, Vector3f &magXYZ) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagXYZ(magXYZ);
    }
}

// return the magnetometer in use for the specified instance
uint8_t NavEKF2::getActiveMag(int8_t instance) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        return core[instance].getActiveMag();
    } else {
        return 255;
    }
}

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool NavEKF2::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    if (!core) {
        return false;
    }
    // try the primary first, else loop through all of the cores and return when one has offsets for this mag instance
    if (core[primary].getMagOffsets(mag_idx, magOffsets)) {
        return true;
    }
    for (uint8_t i=0; i<num_cores; i++) {
        if(core[i].getMagOffsets(mag_idx, magOffsets)) {
            return true;
        }
    }
    return false;
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF2::getLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core[primary].getLLH(loc);
}

// Return the latitude and longitude and height used to set the NED origin for the specified instance
// An out of range instance (eg -1) returns data for the the primary instance
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF2::getOriginLLH(int8_t instance, struct Location &loc) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getOriginLLH(loc);
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool NavEKF2::setOriginLLH(const Location &loc)
{
    if (!core) {
        return false;
    }
    return core[primary].setOriginLLH(loc);
}

// return estimated height above ground level
// return false if ground height is not being estimated.
bool NavEKF2::getHAGL(float &HAGL) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHAGL(HAGL);
}

// return the Euler roll, pitch and yaw angle in radians for the specified instance
void NavEKF2::getEulerAngles(int8_t instance, Vector3f &eulers) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getEulerAngles(eulers);
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF2::getRotationBodyToNED(Matrix3f &mat) const
{
    if (core) {
        core[primary].getRotationBodyToNED(mat);
    }
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF2::getQuaternion(int8_t instance, Quaternion &quat) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getQuaternion(quat);
    }
}

// return the innovations for the specified instance
void NavEKF2::getInnovations(int8_t instance, Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }
}

// publish output observer angular, velocity and position tracking error
void NavEKF2::getOutputTrackingError(int8_t instance, Vector3f &error) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getOutputTrackingError(error);
    }
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void NavEKF2::getVariances(int8_t instance, float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }
}

// should we use the compass? This is public so it can be used for
// reporting via ahrs.use_compass()
bool NavEKF2::use_compass(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].use_compass();
}

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
// msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
// posOffset is the XYZ flow sensor position in the body frame in m
void NavEKF2::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, const Vector3f &posOffset)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
        }
    }
}

// return data for debugging optical flow fusion
void NavEKF2::getFlowDebug(int8_t instance, float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov,
                           float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFlowDebug(varFlow, gndOffset, flowInnovX, flowInnovY, auxInnov, HAGL, rngInnov, range, gndOffsetErr);
    }
}

// return data for debugging range beacon fusion
bool NavEKF2::getRangeBeaconDebug(int8_t instance, uint8_t &ID, float &rng, float &innov, float &innovVar, float &testRatio, Vector3f &beaconPosNED, float &offsetHigh, float &offsetLow) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        return core[instance].getRangeBeaconDebug(ID, rng, innov, innovVar, testRatio, beaconPosNED, offsetHigh, offsetLow);
    } else {
        return false;
    }
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTakeoffExpected(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTakeoffExpected(val);
        }
    }
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTouchdownExpected(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTouchdownExpected(val);
        }
    }
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference
void NavEKF2::setTerrainHgtStable(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTerrainHgtStable(val);
        }
    }

}

/*
  return the filter fault status as a bitmasked integer
  0 = quaternions are NaN
  1 = velocities are NaN
  2 = badly conditioned X magnetometer fusion
  3 = badly conditioned Y magnetometer fusion
  5 = badly conditioned Z magnetometer fusion
  6 = badly conditioned airspeed fusion
  7 = badly conditioned synthetic sideslip fusion
  7 = filter is not initialised
*/
void NavEKF2::getFilterFaults(int8_t instance, uint16_t &faults) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterFaults(faults);
    } else {
        faults = 0;
    }
}

/*
  return filter timeout status as a bitmasked integer
  0 = position measurement timeout
  1 = velocity measurement timeout
  2 = height measurement timeout
  3 = magnetometer measurement timeout
  5 = unassigned
  6 = unassigned
  7 = unassigned
  7 = unassigned
*/
void NavEKF2::getFilterTimeouts(int8_t instance, uint8_t &timeouts) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterTimeouts(timeouts);
    } else {
        timeouts = 0;
    }
}

/*
  return filter status flags
*/
void NavEKF2::getFilterStatus(int8_t instance, nav_filter_status &status) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

/*
return filter gps quality check status
*/
void  NavEKF2::getFilterGpsStatus(int8_t instance, nav_gps_status &status) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterGpsStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

// send an EKF_STATUS_REPORT message to GCS
//void NavEKF2::send_status_report(mavlink_channel_t chan)
//{
//    if (core) {
//        core[primary].send_status_report(chan);
//    }
//}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF2::getHeightControlLimit(float &height) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHeightControlLimit(height);
}

// Returns the amount of yaw angle change (in radians) due to the last yaw angle reset or core selection switch
// Returns the time of the last yaw angle reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastYawResetAngle(float &yawAngDelta)
{
    if (!core) {
        return 0;
    }

    yawAngDelta = 0.0f;

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastYawReset_ms = yaw_reset_data.last_primary_change;

    // There has been a change notification in the primary core that the controller has not consumed
    // or this is a repeated acce
    if (yaw_reset_data.core_changed || yaw_reset_data.last_function_call == now_time_ms) {
        yawAngDelta = yaw_reset_data.core_delta;
        yaw_reset_data.core_changed = false;
    }

    // Record last time controller got the yaw reset
    yaw_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    float temp_yawAng;
    uint32_t lastCoreYawReset_ms = core[primary].getLastYawResetAngle(temp_yawAng);
    if (lastCoreYawReset_ms > lastYawReset_ms) {
        yawAngDelta = wrap_PI(yawAngDelta + temp_yawAng);
        lastYawReset_ms = lastCoreYawReset_ms;
    }

    return lastYawReset_ms;
}

// Returns the amount of NE position change due to the last position reset or core switch in metres
// Returns the time of the last reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastPosNorthEastReset(Vector2f &posDelta)
{
    if (!core) {
        return 0;
    }

    posDelta.zero();

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastPosReset_ms = pos_reset_data.last_primary_change;

    // There has been a change in the primary core that the controller has not consumed
    // allow for multiple consumers on the same frame
    if (pos_reset_data.core_changed || pos_reset_data.last_function_call == now_time_ms) {
        posDelta = pos_reset_data.core_delta;
        pos_reset_data.core_changed = false;
    }

    // Record last time controller got the position reset
    pos_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    Vector2f tempPosDelta;
    uint32_t lastCorePosReset_ms = core[primary].getLastPosNorthEastReset(tempPosDelta);
    if (lastCorePosReset_ms > lastPosReset_ms) {
        posDelta = posDelta + tempPosDelta;
        lastPosReset_ms = lastCorePosReset_ms;
    }

    return lastPosReset_ms;
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF2::getLastVelNorthEastReset(Vector2f &vel) const
{
    if (!core) {
        return 0;
    }
    return core[primary].getLastVelNorthEastReset(vel);
}

// report the reason for why the backend is refusing to initialise
const char *NavEKF2::prearm_failure_reason(void) const
{
    if (!core) {
        return nullptr;
    }
    return core[primary].prearm_failure_reason();
}

// Returns the amount of vertical position change due to the last reset or core switch in metres
// Returns the time of the last reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastPosDownReset(float &posDelta)
{
    if (!core) {
        return 0;
    }

    posDelta = 0.0f;

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastPosReset_ms = pos_down_reset_data.last_primary_change;

    // There has been a change in the primary core that the controller has not consumed
    // allow for multiple consumers on the same frame
    if (pos_down_reset_data.core_changed || pos_down_reset_data.last_function_call == now_time_ms) {
        posDelta = pos_down_reset_data.core_delta;
        pos_down_reset_data.core_changed = false;
    }

    // Record last time controller got the position reset
    pos_down_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    float tempPosDelta;
    uint32_t lastCorePosReset_ms = core[primary].getLastPosDownReset(tempPosDelta);
    if (lastCorePosReset_ms > lastPosReset_ms) {
        posDelta += tempPosDelta;
        lastPosReset_ms = lastCorePosReset_ms;
    }

    return lastPosReset_ms;
}

// update the yaw reset data to capture changes due to a lane switch
void NavEKF2::updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary)
{
    Vector3f eulers_old_primary, eulers_new_primary;
    float old_yaw_delta;

    // If core yaw reset data has been consumed reset delta to zero
    if (!yaw_reset_data.core_changed) {
        yaw_reset_data.core_delta = 0;
    }

    // If current primary has reset yaw after controller got it, add it to the delta
    if (core[old_primary].getLastYawResetAngle(old_yaw_delta) > yaw_reset_data.last_function_call) {
        yaw_reset_data.core_delta += old_yaw_delta;
    }

    // Record the yaw delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getEulerAngles(eulers_old_primary);
    core[new_primary].getEulerAngles(eulers_new_primary);
    yaw_reset_data.core_delta = wrap_PI(eulers_new_primary.z - eulers_old_primary.z + yaw_reset_data.core_delta);
    yaw_reset_data.last_primary_change = imuSampleTime_us / 1000;
    yaw_reset_data.core_changed = true;

}

// update the position reset data to capture changes due to a lane switch
void NavEKF2::updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary)
{
    Vector2f pos_old_primary, pos_new_primary, old_pos_delta;

    // If core position reset data has been consumed reset delta to zero
    if (!pos_reset_data.core_changed) {
        pos_reset_data.core_delta.zero();
    }

    // If current primary has reset position after controller got it, add it to the delta
    if (core[old_primary].getLastPosNorthEastReset(old_pos_delta) > pos_reset_data.last_function_call) {
        pos_reset_data.core_delta += old_pos_delta;
    }

    // Record the position delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getPosNE(pos_old_primary);
    core[new_primary].getPosNE(pos_new_primary);
    pos_reset_data.core_delta = pos_new_primary - pos_old_primary + pos_reset_data.core_delta;
    pos_reset_data.last_primary_change = imuSampleTime_us / 1000;
    pos_reset_data.core_changed = true;

}

// Update the vertical position reset data to capture changes due to a core switch
// This should be called after the decision to switch cores has been made, but before the
// new primary EKF update has been run
void NavEKF2::updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary)
{
    float posDownOldPrimary, posDownNewPrimary, oldPosDownDelta;

    // If core position reset data has been consumed reset delta to zero
    if (!pos_down_reset_data.core_changed) {
        pos_down_reset_data.core_delta = 0.0f;
    }

    // If current primary has reset position after controller got it, add it to the delta
    if (core[old_primary].getLastPosDownReset(oldPosDownDelta) > pos_down_reset_data.last_function_call) {
        pos_down_reset_data.core_delta += oldPosDownDelta;
    }

    // Record the position delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getPosD(posDownOldPrimary);
    core[new_primary].getPosD(posDownNewPrimary);
    pos_down_reset_data.core_delta = posDownNewPrimary - posDownOldPrimary + pos_down_reset_data.core_delta;
    pos_down_reset_data.last_primary_change = imuSampleTime_us / 1000;
    pos_down_reset_data.core_changed = true;

}

/*
  get timing statistics structure
*/
void NavEKF2::getTimingStatistics(int8_t instance, struct ekf_timing &timing) const
{
    if (instance < 0 || instance >= num_cores) {
        instance = primary;
    }
    if (core) {
        core[instance].getTimingStatistics(timing);
    } else {
        memset(&timing, 0, sizeof(timing));
    }
}

/*
 * Write position and quaternion data from an external navigation system
 *
 * pos        : XYZ position (m) in a RH navigation frame with the Z axis pointing down and XY axes horizontal. Frame must be aligned with NED if the magnetomer is being used for yaw.
 * quat       : quaternion describing the the rotation from navigation frame to body frame
 * posErr     : 1-sigma spherical position error (m)
 * angErr     : 1-sigma spherical angle error (rad)
 * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
 * resetTime_ms : system time of the last position reset request (mSec)
 *
*/
void NavEKF2::writeExtNavData(const Vector3f &sensOffset, const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeExtNavData(sensOffset, pos, quat, posErr, angErr, timeStamp_ms, resetTime_ms);
        }
    }
}

#endif //HAL_CPU_CLASS
