#include "AP_Ins_Invensense.h"
#include "AP_Ins_Invensense_regs.h"

#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_BUFFER_LEN 16

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

AP_Ins_Invensense::AP_Ins_Invensense()
{
	dev = hal.spi_dev1;
}

AP_Ins_Invensense::~AP_Ins_Invensense()
{
	  if (fifo_buffer != nullptr) {
        hal.util->free_type(fifo_buffer, MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

bool AP_Ins_Invensense::initialize()
{
	dev->set_read_flag(0x80);
	bool success = hardware_init();
	return success;
}

bool AP_Ins_Invensense::hardware_init(void)
{
    // setup for register checking. We check much less often on I2C
    // where the cost of the checks is higher
    dev->setup_checked_registers(7, 20);
    
    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!check_whoami())
	{
        return false;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++)
	{
        last_stat_user_ctrl = dev->read_register(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN)
		{
            last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            dev->write_register(MPUREG_USER_CTRL, last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        dev->write_register(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
		
		/* Disable I2C bus if SPI selected (Recommended in Datasheet to be
		 * done just after the device is reset) */
		last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
		dev->write_register(MPUREG_USER_CTRL, last_stat_user_ctrl);


        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        dev->write_register(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (dev->read_register(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (tries == 5)
	{
        return false;
    }

	if (mpu_type == Invensense_ICM20608 || mpu_type == Invensense_ICM20602)
	{
        // this avoids a sensor bug, see description above
		dev->write_register(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    return true;
}

/*
  check whoami for sensor type
 */
bool AP_Ins_Invensense::check_whoami(void)
{
    uint8_t whoami = dev->read_register(MPUREG_WHOAMI);
    switch (whoami) {
    case MPU_WHOAMI_6000:
        mpu_type = Invensense_MPU6000;
        return true;
    case MPU_WHOAMI_6500:
        mpu_type = Invensense_MPU6500;
        return true;
    case MPU_WHOAMI_MPU9250:
    case MPU_WHOAMI_MPU9255:
        mpu_type = Invensense_MPU9250;
        return true;
    case MPU_WHOAMI_20608:
        mpu_type = Invensense_ICM20608;
        return true;
    case MPU_WHOAMI_20602:
        mpu_type = Invensense_ICM20602;
        return true;
    case MPU_WHOAMI_ICM20789:
    case MPU_WHOAMI_ICM20789_R1:
        mpu_type = Invensense_ICM20789;
        return true;
    case MPU_WHOAMI_ICM20689:
        mpu_type = Invensense_ICM20689;
        return true;
    }
    // not a value WHOAMI result
    return false;
}


void AP_Ins_Invensense::fifo_reset()
{
    uint8_t user_ctrl = last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);

    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->write_register(MPUREG_FIFO_EN, 0);
    dev->write_register(MPUREG_USER_CTRL, user_ctrl);
	dev->write_register(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    dev->write_register(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    dev->write_register(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN | BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN, true);
    hal.scheduler->delay(1);
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;
	
}

uint16_t bytes_read;
uint8_t n_samples;

bool need_reset = false;

void AP_Ins_Invensense::read_fifo()
{
    
    uint8_t *rx = fifo_buffer;

    if (!dev->read_registers(MPUREG_FIFO_COUNTH, rx, 2))
	{
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;

    if (n_samples == 0) 
	{
        /* Not enough data in FIFO */
        goto check_registers;
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz

     */
	if (n_samples > 32)
	{
		need_reset = true;
		n_samples = 24;
	}
    
    while (n_samples > 0)
	{
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);

		if (!dev->read_registers(MPUREG_FIFO_R_W, rx, n * MPU_SAMPLE_SIZE)) {
			goto check_registers;
		}
		
		if (!accumulate(rx, n))
		{
			break;
		}
        n_samples -= n;
    }

    if (need_reset)
	{
		need_reset = false;
        //debug("fifo reset n_samples %u", bytes_read/MPU_SAMPLE_SIZE);
        fifo_reset();
    }
    
check_registers:
    // check next register value for correctness
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!dev->check_next_register())
	{
//        inc_gyro_error_count(_gyro_instance);
//        inc_accel_error_count(_accel_instance);
    }
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_Ins_Invensense::start()
{

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    dev->write_register(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    // always use FIFO
    fifo_reset();

    // grab the used instances
    enum DevTypes gdev, adev;
	
    switch (mpu_type) {
    case Invensense_MPU9250:
        gdev = DEVTYPE_GYR_MPU9250;
        adev = DEVTYPE_ACC_MPU9250;
        break;
    case Invensense_MPU6000:
    case Invensense_MPU6500:
    case Invensense_ICM20608:
    case Invensense_ICM20602:
    default:
        gdev = DEVTYPE_GYR_MPU6000;
        adev = DEVTYPE_ACC_MPU6000;
        break;
    case Invensense_ICM20789:
        gdev = DEVTYPE_INS_ICM20789;
        adev = DEVTYPE_INS_ICM20789;
        break;
    case Invensense_ICM20689:
        gdev = DEVTYPE_INS_ICM20689;
        adev = DEVTYPE_INS_ICM20689;
        break;
    }

    /*
      setup temperature sensitivity and offset. This varies
      considerably between parts
     */
    switch (mpu_type) {
    case Invensense_MPU9250:
        temp_zero = 21;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_MPU6000:
    case Invensense_MPU6500:
        temp_zero = 36.53;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_ICM20608:
    case Invensense_ICM20602:
        temp_zero = 25;
        temp_sensitivity = 1.0/326.8; 
        break;

    case Invensense_ICM20789:
        temp_zero = 25;
        temp_sensitivity = 0.003;
        break;
    case Invensense_ICM20689:
        temp_zero = 25;
        temp_sensitivity = 0.003;
        break;
    }

//    gyro_instance = _imu.register_gyro(1000, _dev->get_bus_id_devtype(gdev));
//    accel_instance = _imu.register_accel(1000, _dev->get_bus_id_devtype(adev));

    // setup ODR and on-sensor filtering
    set_filter_register();

    // update backend sample rate
//    set_accel_raw_sample_rate(_accel_instance, _backend_rate_hz);
//    set_gyro_raw_sample_rate(_gyro_instance, _backend_rate_hz);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
//    set_raw_sample_accel_multiplier(_accel_instance, multiplier_accel);
    
    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    dev->write_register(MPUREG_SMPLRT_DIV, 0, true);
    hal.scheduler->delay(1);

    // Gyro scale 2000º/s
    dev->write_register(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS, true);
	gyro_scale = M_PI/(180*GYRO_FS_2000DPS_LSB);
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t product_id = dev->read_register(MPUREG_PRODUCT_ID);

    if (mpu_type == Invensense_MPU6000 &&
        ((product_id == MPU6000ES_REV_C4) ||
         (product_id == MPU6000ES_REV_C5) ||
         (product_id == MPU6000_REV_C4)   ||
         (product_id == MPU6000_REV_C5))) 
	{
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        dev->write_register(MPUREG_ACCEL_CONFIG,1<<3, true);
        accel_scale = GRAVITY_MSS / 4096.f;
    }
	else
	{
        // Accel scale 16g (2048 LSB/g)
        dev->write_register(MPUREG_ACCEL_CONFIG,3<<3, true);
        accel_scale = GRAVITY_MSS / 2048.f;
    }
    hal.scheduler->delay(1);

	if (mpu_type == Invensense_ICM20608 ||
        mpu_type == Invensense_ICM20602) {
        // this avoids a sensor bug, see description above
		dev->write_register(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    // configure interrupt to fire when new data arrives
    dev->write_register(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt. We don't do this for the 20789 as
    // that sensor has already setup the appropriate config inside the
    // baro driver.
    if (mpu_type != Invensense_ICM20789) {    
        uint8_t v = dev->read_register(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN;
        v &= BIT_BYPASS_EN;
        dev->write_register(MPUREG_INT_PIN_CFG, v);
    }

    // now that we have initialised, we set the bus speed to high
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // setup sensor rotations from probe()
//    set_gyro_orientation(_gyro_instance, _rotation);
//    set_accel_orientation(_accel_instance, _rotation);

    // setup scale factors for fifo data after downsampling
    fifo_accel_scale = accel_scale / (MAX(fifo_downsample_rate,2)/2);
    fifo_gyro_scale = gyro_scale / fifo_downsample_rate;
    
    // allocate fifo buffer
    fifo_buffer = (uint8_t *)hal.util->malloc_type(MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (fifo_buffer == nullptr)
	{
        AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }
}


/*
  publish any pending data
 */
bool AP_Ins_Invensense::update()
{
	read_fifo();
	
    return true;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_Ins_Invensense::data_ready()
{
    uint8_t status = dev->read_register(MPUREG_INT_STATUS);
    return (status & BIT_RAW_RDY_INT) != 0;
}

/*
  when doing fast sampling the sensor gives us 8k samples/second. Every 2nd accel sample is a duplicate.

  To filter this we first apply a 1p low pass filter at 188Hz, then we
  average over 8 samples to bring the data rate down to 1kHz. This
  gives very good aliasing rejection at frequencies well above what
  can be handled with 1kHz sample rates.
 */
bool AP_Ins_Invensense::accumulate(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    const int32_t clip_limit = AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS / accel_scale;
    bool clipped = false;
    bool ret = true;
    
    for (uint8_t i = 0; i < n_samples; i++)
	{
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;

        // use temperatue to detect FIFO corruption
        int16_t t2 = int16_val(data, 3);
        if (!check_raw_temp(t2)) 
		{
            fifo_reset();
            ret = false;
            break;
        }
        tsum += t2;

        if ((accum.count & 1) == 0)
		{
            // accel data is at 4kHz
            Vector3f a( -int16_val(data, 0), int16_val(data, 1), -int16_val(data, 2) );
			
            if (fabsf(a.x) > clip_limit || fabsf(a.y) > clip_limit || fabsf(a.z) > clip_limit)
			{
                clipped = true;
            }
            accum.accel += accum.accel_filter.apply(a);
            Vector3f a2 = a * accel_scale;
//            _notify_new_accel_sensor_rate_sample(_accel_instance, a2);
        }

        Vector3f g( -int16_val(data, 4), int16_val(data, 5), -int16_val(data, 6) );
		
		accum.gyro += accum.gyro_filter.apply(g);
        Vector3f g2 = g * gyro_scale;
		
//        notify_new_gyro_sensor_rate_sample(_gyro_instance, g2);
		
        accum.count++;

        if (accum.count == fifo_downsample_rate)
		{

            accum.accel *= fifo_accel_scale;
            accum.gyro *= fifo_gyro_scale;
            
//            rotate_and_correct_accel(_accel_instance, _accum.accel);
//            rotate_and_correct_gyro(_gyro_instance, _accum.gyro);
//            
//            notify_new_accel_raw_sample(_accel_instance, _accum.accel, 0, false);
//            notify_new_gyro_raw_sample(_gyro_instance, _accum.gyro);
            
            accum.accel.zero();
            accum.gyro.zero();
            accum.count = 0;
        }
    }

    if (ret) 
	{
        float temp = (static_cast<float>(tsum)/n_samples)*temp_sensitivity + temp_zero;
        temp_filtered = temp_filter.apply(temp);
    }
    
    return ret;
}


/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_Ins_Invensense::check_raw_temp(int16_t t2)
{
    if (abs(t2 - raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (dev->read_registers(MPUREG_TEMP_OUT_H, trx, 2)) {
        raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - raw_temp) < 400);
}

void AP_Ins_Invensense::set_filter_register(void)
{
	uint8_t config;

#if INVENSENSE_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    config = (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#else
    config = 0;
#endif

    // assume 1kHz sampling to start
    fifo_downsample_rate = 1;
    backend_rate_hz = 1000;
   
    fifo_downsample_rate = 8;
	// calculate rate we will be giving samples to the backend
	backend_rate_hz *= (8 / fifo_downsample_rate);

	/* set divider for internal sample rate to 0x1F when fast
	 sampling enabled. This reduces the impact of the slave
	 sensor on the sample rate. It ends up with around 75Hz
	 slave rate, and reduces the impact on the gyro and accel
	 sample rate, ending up with around 7760Hz gyro rate and
	 3880Hz accel rate
	 */
	dev->write_register(MPUREG_I2C_SLV4_CTRL, 0x1F);
    
	// this gives us 8kHz sampling on gyros and 4kHz on accels
	config |= BITS_DLPF_CFG_256HZ_NOLPF2;

    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    dev->write_register(MPUREG_CONFIG, config, true);

	if (mpu_type != Invensense_MPU6000) 
	{
		// setup for 4kHz accels
		dev->write_register(ICMREG_ACCEL_CONFIG2, ICM_ACC_FCHOICE_B, true);
    }
}


