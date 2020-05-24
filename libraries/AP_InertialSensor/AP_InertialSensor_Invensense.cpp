#include "AP_InertialSensor_Invensense.h"
#include "AP_InertialSensor_Invensense_registers.h"

#include <stdint.h>
#include <stdlib.h>

#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_BUFFER_LEN 16

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_Invensense::AP_InertialSensor_Invensense( void *_dev )
    : AP_InertialSensor_Backend()
{
	dev = (HAL_BUS *)_dev;
}

AP_InertialSensor_Invensense::~AP_InertialSensor_Invensense()
{
    if (fifo_buffer != nullptr) 
	{
        free(fifo_buffer);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensense::probe( void *_dev )
{
	int ret = -1;

	if(_dev == nullptr)
	{
		return nullptr;
	}
	
	AP_InertialSensor_Invensense *sensor = new AP_InertialSensor_Invensense( _dev );
	if(sensor == nullptr)
	{
		return nullptr;
	}
	
	ret = sensor->Initialize();
	if(ret<0)
	{
		delete sensor;
		return nullptr;
	}
	
	sensor->CalibrationGyroOffset();
	
	
	return sensor;
}




/*
  publish any pending data
 */
int AP_InertialSensor_Invensense::GetRawData(void )
{
	int ret = -1;
	uint8_t temp[14];
	uint8_t reg = MPUREG_ACCEL_XOUT_H;
	
	ret = dev->read_registers( reg, temp, 14 );
	if(ret<0)
	{
		return -1;
	}
	memcpy( imuRawDataBuffer, temp, 14 );
	
	accelRawData.x = (float)( (int16_t)(uint16_t(temp[0])<<8 | temp[1]) );
	accelRawData.y = (float)( (int16_t)(uint16_t(temp[2])<<8 | temp[3]) );
	accelRawData.z = (float)( (int16_t)(uint16_t(temp[4])<<8 | temp[5]) );
	
	tempRawData = (float)( (int16_t)((uint16_t)temp[6]<<8 | temp[7]) );
	
	gyroRawData.x = (float)( (int16_t)(uint16_t(temp[8]) <<8 | temp[9]) ); 
	gyroRawData.y = (float)( (int16_t)(uint16_t(temp[10])<<8 | temp[11]) );
	gyroRawData.z = (float)( (int16_t)(uint16_t(temp[12])<<8 | temp[13]) );
	
    return 0;
}


int AP_InertialSensor_Invensense::Update(void )
{
	int ret = -1;
	
	ret = GetRawData();
	if(ret<0)
	{
		return -1;
	}
	
	accelData = accelRawData*accel_scale;
	tempData = tempRawData/340 + 36.53f;
	gyroData = (gyroRawData-gyroOffset)*gyro_scale;
	
	accelFilterData = accel_filter.apply(accelData);
	gyroFilterData = gyro_filter.apply(gyroData);
	
	return 0;
}

int AP_InertialSensor_Invensense::CalibrationGyroOffset(void)
{
	int ret = -1;
	
	bool gyroCalibateOK = false;
	uint16_t cnt = 0;
	
	Vector3f gyroCalibrateSataSum(0.0f, 0.0f, 0.0f);
	Vector3f gyroCalibrateData(0.0f, 0.0f, 0.0f);
	Vector3f gyroCalibrateDataLast(0.0f, 0.0f, 0.0f);
	Vector3f goryDataError(0.0f, 0.0f, 0.0f);

	while(!gyroCalibateOK)
	{
		for(cnt=0; cnt<500; cnt++)
		{
			ret = GetRawData();
			gyroCalibrateSataSum += gyroRawData;
			HAL_Delay_ms(2);
		}
		gyroCalibrateData = gyroCalibrateSataSum/cnt;
		gyroCalibrateSataSum.zero();
		
		goryDataError = gyroCalibrateDataLast - gyroCalibrateData;
		if( goryDataError.length()<0.7f )
		{
			gyroCalibateOK = true;
		}
		else
		{
			gyroCalibateOK = false;
		}
		
		gyroCalibrateDataLast = gyroCalibrateData;
	}
	gyroOffset = gyroCalibrateDataLast;
	
	return 0;
}


/*
  check whoami for sensor type
 */
int AP_InertialSensor_Invensense::CheckWhoami(void)
{
    uint8_t whoami = dev->read_register(MPUREG_WHOAMI);
    switch (whoami) 
	{
    case MPU_WHOAMI_6000:
        mpu_type = Invensense_MPU6000;
        return 0;
    case MPU_WHOAMI_6500:
        mpu_type = Invensense_MPU6500;
        return 0;
    case MPU_WHOAMI_MPU9250:
    case MPU_WHOAMI_MPU9255:
        mpu_type = Invensense_MPU9250;
        return 0;
    case MPU_WHOAMI_20608:
        mpu_type = Invensense_ICM20608;
        return 0;
    case MPU_WHOAMI_20602:
        mpu_type = Invensense_ICM20602;
        return 0;
    case MPU_WHOAMI_ICM20789:
    case MPU_WHOAMI_ICM20789_R1:
        mpu_type = Invensense_ICM20789;
        return 0;
    case MPU_WHOAMI_ICM20689:
        mpu_type = Invensense_ICM20689;
        return 0;
    }
    // not a value WHOAMI result
    return -1;
}


int AP_InertialSensor_Invensense::Initialize(void)
{
	int ret = -1;
	
	dev->set_read_flag(0x80);
	
	ret = CheckWhoami();
    if( ret<0 ) 
	{
        return -1;
    }

    // Chip reset
    uint8_t tries = 0;
    for(tries = 0; tries < 5; tries++)
	{
        last_stat_user_ctrl = dev->read_register(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) 
		{
            last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            dev->write_register(MPUREG_USER_CTRL, last_stat_user_ctrl);
            HAL_Delay_ms(10);
        }

        /* reset device */
        dev->write_register(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        HAL_Delay_ms(100);

        /* bus-dependent initialization */
        if(dev->get_bus_type() == HAL_BUS::BUS_TYPE_SPI)
		{
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
            dev->write_register(MPUREG_USER_CTRL, last_stat_user_ctrl);
        }

        /* bus-dependent initialization */
        if ((dev->get_bus_type() == HAL_BUS::BUS_TYPE_I2C) && (mpu_type == Invensense_MPU9250 || mpu_type == Invensense_ICM20789)) 
		{
            /* Enable I2C bypass to access internal device */
            dev->write_register(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN);
        }


        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        dev->write_register(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        HAL_Delay_ms(5);

        // check it has woken up
        if (dev->read_register(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) 
		{
            break;
        }

        HAL_Delay_ms(10);
        if (data_ready()) 
		{
            break;
        }
    }

    if (tries == 5) 
	{
        return -1;
    }

	if (mpu_type == Invensense_ICM20608 || mpu_type == Invensense_ICM20602)
	{
        // this avoids a sensor bug, see description above
		dev->write_register(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
	}
	
	Start();
    
    return 0;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
int AP_InertialSensor_Invensense::data_ready()
{
    uint8_t status = dev->read_register(MPUREG_INT_STATUS);
    if( (status & BIT_RAW_RDY_INT) != 0 )
	{
		return 0;
	}
	return -1;
}


int AP_InertialSensor_Invensense::Start()
{


    // initially run the bus at low speed
    dev->set_speed(HAL_BUS::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    dev->write_register(MPUREG_PWR_MGMT_2, 0x00);
    HAL_Delay_ms(1);

    // always use FIFO
    //_fifo_reset();

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

	// _gyro_instance = _imu.register_gyro(1000, _dev->get_bus_id_devtype(gdev));
	// _accel_instance = _imu.register_accel(1000, _dev->get_bus_id_devtype(adev));

    // setup ODR and on-sensor filtering
	// _set_filter_register();

    // update backend sample rate
    //_set_accel_raw_sample_rate(_accel_instance, _backend_rate_hz);
    //_set_gyro_raw_sample_rate(_gyro_instance, _backend_rate_hz);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    //_set_raw_sample_accel_multiplier(_accel_instance, multiplier_accel);

    
    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    dev->write_register(MPUREG_SMPLRT_DIV, 0);
    HAL_Delay_ms(1);

    // Gyro scale 2000º/s
    dev->write_register(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);
	gyro_scale = GYRO_SCALE;
    HAL_Delay_ms(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t product_id = dev->read_register(MPUREG_PRODUCT_ID);

    if (mpu_type == Invensense_MPU6000 &&
		((product_id == MPU6000ES_REV_C4) ||
		( product_id == MPU6000ES_REV_C5) ||
		( product_id == MPU6000_REV_C4)   ||
		( product_id == MPU6000_REV_C5))) 
	{
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        dev->write_register(MPUREG_ACCEL_CONFIG,1<<3);
        accel_scale = GRAVITY_MSS / 4096.f;
    }
	else 
	{
        // Accel scale 16g (2048 LSB/g)
        dev->write_register(MPUREG_ACCEL_CONFIG,3<<3);
        accel_scale = GRAVITY_MSS / 2048.f;
    }
    HAL_Delay_ms(1);

	if (mpu_type == Invensense_ICM20608 || mpu_type == Invensense_ICM20602)
	{
        // this avoids a sensor bug, see description above
		dev->write_register(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
	}
    
    // configure interrupt to fire when new data arrives
    dev->write_register(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    HAL_Delay_ms(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt. We don't do this for the 20789 as
    // that sensor has already setup the appropriate config inside the
    // baro driver.
    if (mpu_type != Invensense_ICM20789)
	{    
        uint8_t v = dev->read_register(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN;
        v &= BIT_BYPASS_EN;
        dev->write_register(MPUREG_INT_PIN_CFG, v);
    }

    // now that we have initialised, we set the bus speed to high
    dev->set_speed(HAL_BUS::SPEED_HIGH);

    // setup sensor rotations from probe()
    //set_gyro_orientation(_gyro_instance, _rotation);
    //set_accel_orientation(_accel_instance, _rotation);
    
    // allocate fifo buffer
    fifo_buffer = (uint8_t *)calloc(MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, 1);
    if (fifo_buffer == nullptr)
	{
        //AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples
    //dev->register_periodic_callback(1000000UL / _backend_rate_hz, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensense::_poll_data, void));
	
	return 0;
}
