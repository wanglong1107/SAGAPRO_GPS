#pragma once
/*
  driver for the invensense range of IMUs, including:

  MPU6000
  MPU9250
  ICM-20608
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>


// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS        (15.5f*GRAVITY_MSS) // accelerometer values over 15.5G are recorded as a clipping error
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz
#define AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS 500     // peak-hold detector timeout




extern const AP_HAL::HAL& hal;


class AP_Ins_Invensense
{
public:

    enum Invensense_Type {
        Invensense_MPU6000=0,
        Invensense_MPU6500,
        Invensense_MPU9250,
        Invensense_ICM20608,
        Invensense_ICM20602,
        Invensense_ICM20789,
        Invensense_ICM20689,
    };
	
	/*
	device driver IDs. These are used to fill in the devtype field
	of the device ID, which shows up as INS*ID* parameters to
	users. The values are chosen for compatibility with existing PX4
	drivers.
	If a change is made to a driver that would make existing
	calibration values invalid then this number must be changed.
	*/
    enum DevTypes {

		DEVTYPE_ACC_MPU6000  = 0x13,
		DEVTYPE_ACC_MPU9250  = 0x16,
		DEVTYPE_GYR_MPU6000  = 0x21,
		DEVTYPE_GYR_MPU9250  = 0x24,
		DEVTYPE_INS_ICM20789 = 0x27,
		DEVTYPE_INS_ICM20689 = 0x28,
	};
    // acclerometers on Invensense sensors will return values up to
    // 24G, but they are not guaranteed to be remotely linear past
    // 16G
	
	
	AP_Ins_Invensense();
    virtual ~AP_Ins_Invensense();
	
	bool initialize();
    bool hardware_init();
    bool check_whoami();
	
	void start();
	bool update();
	
	

    const uint16_t multiplier_accel = INT16_MAX/(26*GRAVITY_MSS);
	
	void fifo_reset();
    /* Read samples from FIFO (FIFO enabled) */
    void read_fifo();
	
	
	
private:
	
	AP_HAL::Device *dev;

    void set_filter_register(void);
   

    bool accumulate(uint8_t *samples, uint8_t n_samples);
    bool accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples);

    bool check_raw_temp(int16_t t2);

	bool data_ready();
	
    int16_t raw_temp;
    
    // instance numbers of accel and gyro data
    uint8_t gyro_instance;
    uint8_t accel_instance;

    float temp_sensitivity = 1.0/340; // degC/LSB
    float temp_zero = 36.53; // degC
    
    float temp_filtered;
    float accel_scale;
	float gyro_scale;

    float fifo_accel_scale;
    float fifo_gyro_scale;
    LowPassFilter2pFloat temp_filter;

    enum Rotation rotation;

    // which sensor type this is
    enum Invensense_Type mpu_type;

    // are we doing more than 1kHz sampling?
    bool fast_sampling;

    // what downsampling rate are we using from the FIFO?
    uint8_t fifo_downsample_rate;

    // what rate are we generating samples into the backend?
    uint16_t backend_rate_hz;
	
    // Last status from register user control
    uint8_t last_stat_user_ctrl;    

    // buffer for fifo read
    uint8_t *fifo_buffer;

    /*
      accumulators for sensor_rate sampling
      See description in _accumulate_sensor_rate_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t count;
        LowPassFilterVector3f accel_filter{4000, 188};
        LowPassFilterVector3f gyro_filter{8000, 188};
    } accum;
};


#ifndef INS_INVENSENSE_20789_I2C_ADDR
#define INS_INVENSENSE_20789_I2C_ADDR 0x68
#endif
