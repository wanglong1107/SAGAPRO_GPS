#include "ms5611.h"

//********************************************************
//! @brief calculate the CRC code for details look into CRC CODE NOTES
//!
//! @return crc code
//********************************************************
uint16_t static crc4(uint16_t n_prom[])
{
  int cnt; // simple counter
  uint16_t n_rem; // crc reminder
  uint16_t crc_read; // original value of the crc
  uint8_t n_bit;
  
  n_rem = 0x00;
  crc_read = n_prom[7]; //save read CRC
  n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
  
  for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt%2==1) 
    {
      n_rem ^= (uint16_t) ((n_prom[cnt>>1]) & 0x00FF);
    }
    else 
    {
      n_rem ^= (uint16_t) (n_prom[cnt>>1]>>8);
    }
    
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }
  n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
  n_prom[7]=crc_read; // restore the crc_read to its original place
  return (n_rem ^ 0x00);
}


MS5611::MS5611()
{
	dev = hal.i2c_dev2;
	
	initialFlag = false;
	updateState = 0;
	updateFlag = false;
	
	cnt = 0;
	sum_temp = 0.0f;
	sum_pres = 0.0f;
}


void MS5611::reset(void)
{
	uint8_t data = 0x1E;
	
	dev->write_register( CMD_MS5611_RESET, data);
	
	hal.scheduler->delay(10);
	
	
}

void MS5611::readProm( void )
{	
	uint8_t buffer[2] = {0x00, 0x00};

	dev->read_registers( CMD_MS5611_PROM_Setup, buffer, 2 );
	receve = ( buffer[0]<<8 | buffer[1] );

	dev->read_registers( CMD_MS5611_PROM_C1, buffer, 2 );
	c1 = ( buffer[0]<<8 | buffer[1] );	


	dev->read_registers( CMD_MS5611_PROM_C2, buffer, 2 );
	c2 = ( buffer[0]<<8 | buffer[1] );	

	dev->read_registers( CMD_MS5611_PROM_C3, buffer, 2 );
	c3 = ( buffer[0]<<8 | buffer[1] );	

	dev->read_registers( CMD_MS5611_PROM_C4, buffer, 2 );
	c4 = ( buffer[0]<<8 | buffer[1] );	

	dev->read_registers( CMD_MS5611_PROM_C5, buffer, 2 );
	c5 = ( buffer[0]<<8 | buffer[1] );	

	dev->read_registers( CMD_MS5611_PROM_C6, buffer, 2 );
	c6 = ( buffer[0]<<8 | buffer[1] );	

	dev->read_registers( CMD_MS5611_PROM_CRC, buffer, 2 );
	crc = ( buffer[0]<<8 | buffer[1] );
}

int MS5611::initialize(void)
{
	
	uint16_t pArray[8] = {0x0000};
	uint16_t CRC_Value = 0x00;
	
	dev->initialize();
	
	reset();
	readProm();
	
	pArray[0] = receve;
	pArray[1] = c1;
	pArray[2] = c2;
	pArray[3] = c3;
	pArray[4] = c4;
	pArray[5] = c5;
	pArray[6] = c6;
	pArray[7] = crc;
	
	CRC_Value = crc4(pArray);
	
	if((crc&0x000F) != CRC_Value)
	{
		crc_ok = false;
		return -1;

	}
	crc_ok = true;
	
	writeTemperatureCMD();
	
	return 0;
	
}


uint32_t MS5611::readADC(void)
{
	uint8_t buffer[3] = {0, 0, 0};
	uint32_t return_value;

	dev->read_registers( MS5611_ADC, buffer, 3);
	return_value = (((uint32_t)buffer[0])<<16) | (((uint32_t)buffer[1])<<8) | (buffer[2]);

	return(return_value);
}


void MS5611::writePressureCMD()
{
	uint8_t data = 0x00;

	dev->write_register( CMD_CONVERT_D1_OSR4096, data);
}
void MS5611::writeTemperatureCMD()
{
	uint8_t data = 0x00;

	dev->write_register( CMD_CONVERT_D2_OSR4096, data);
}

int32_t MS5611::getTemperature()
{
	uint32_t d2;//digital temperature value
	int32_t ret;

	d2 = readADC();

	dt = (int32_t)d2 - ((int32_t)c5<<8);

	ret = 2000 + (int32_t)(((int64_t)dt * c6)>> 23);
	temperature = ret;

	return (ret);
}

	
float MS5611::getPressure()
{
	uint32_t d1;// digital pressure value

	//温度校验值
	int64_t off2;
	int32_t temp2;
	int64_t sens2;
	int64_t aux;

	d1 = readADC();


	sens = ((int64_t)c1 << 15) + (((int64_t)c3 * dt)>>8);
	off = ((int64_t)c2 << 16) + (((int64_t)c4 * dt)>>7);


	if(temperature < 2000)
	{
		// second order temperature compensation when under 20 degrees C
		temp2 = (int32_t)(((int64_t)dt * dt)>>31);
		
		aux = ((int64_t)temperature-2000)*((int64_t)temperature-2000);
		off2 = 5*aux >> 1;
		sens2 = 5*aux >> 2;
		if( temperature < -1500)
		{
			aux = ((int64_t)temperature + 1500)*((int64_t)temperature + 1500);
			off2 = off2 + 7*aux;
			sens2 = sens2 + (11*aux >> 1);
		}
		temperature = temperature - temp2;
		off = off - off2;
		sens = sens - sens2; 
	}
	pressure = (((d1*sens) >> 21) - off ) >> 15;

	return(pressure);
	
}



// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float MS5611::calculateAltitudeDifference()
{
    float ret;
	
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = 101.325;

	/* measured pressure in kPa */
	double p = pressure / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */

	ret = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
	altitude = ret;
	return ret;
}

float MS5611::getAltitudeDifference(float base_alt, float alt)
{
    float ret;
    ret = alt - base_alt;
    return ret;
}


int MS5611::update(void)
{
	if (updateState == 0) 
	{
		// On state 0 we read temp
		temperature = getTemperature();
		writePressureCMD();
		updateState++;
	} 		
	else if( updateState >4 )
	{
		updateState = 0;
	}
	else
	{
		pressureRawData = getPressure();
		pressure = baro_filter.apply(pressureRawData);
		if (updateState == 4)
		{
			writeTemperatureCMD();
			updateState = 0;
		} 
		else
		{
			writePressureCMD();
			updateState++;
		}
	}
	
	if( !initialFlag )
	{
		cnt ++;
		if(cnt >200 )
		{
			sum_temp += (float)temperature;
			sum_pres += (float)pressureRawData;
		}
		
		if(cnt > 300)
		{
			initialFlag = true;
			
			groundTemperture = sum_temp /101.0f;		
			groundPressure = sum_pres /101.0f;
		}
	}

//	last_update_ms = hrt_absolute_time();
	updateFlag = true;
	return 0;
}

