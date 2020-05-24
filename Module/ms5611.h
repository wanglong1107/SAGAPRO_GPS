#ifndef _MS5611_H
#define _MS5611_H

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
	 
	 
	 
#define MS5611_ADC            0x00
#define CMD_MS5611_RESET      0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1    0xA2
#define CMD_MS5611_PROM_C2    0xA4
#define CMD_MS5611_PROM_C3    0xA6
#define CMD_MS5611_PROM_C4    0xA8
#define CMD_MS5611_PROM_C5    0xAA
#define CMD_MS5611_PROM_C6    0xAC
#define CMD_MS5611_PROM_CRC   0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximun resolution

extern const AP_HAL::HAL& hal;

class MS5611
{
public:
	MS5611();
	
	
	
	bool flag;
	bool baroUpdate;
	
	uint16_t receve;
	uint16_t c1;//Pressure sensitivity
	uint16_t c2;//Pressure offset
	uint16_t c3;//Temperature coefficient of pressure sensitivity
	uint16_t c4;//Temperature coefficient of pressure offset
	uint16_t c5;//Reference temperature
	uint16_t c6;//temperature coefficient of the temperature
	uint16_t crc;
	
	bool crc_ok;
	
	int64_t off; //offset at actual temperature
	int64_t sens;//sensitivity at actual temperature

	int32_t dt;
	
	bool initialFlag;
	
	uint32_t cnt;
	float sum_temp;
	float sum_pres;
	
	float groundTemperture;
	float groundPressure;
	float groundAltitude;
	
	
	
	int32_t temperature;
	int32_t pressureRawData;
	float altitude;
	float relativeAltitude;
	
	float pressure;
	
	uint8_t updateState;
	bool updateFlag;
	
	LowPassFilter2pFloat baro_filter{100, 10};
	
	
	
	void reset(void);
	void readProm(void);
	int initialize(void);
	
	uint32_t readADC(void);

	
	void writePressureCMD();
	void writeTemperatureCMD();
	

	int update(void);
	float getAltitudeDifference(float base_alt, float alt);
	
	float getPressure();
	int32_t getTemperature();
	
	float getBarometerGroundTemperature();
	float getBarometerGroundPressure();	
	float getBarometerGroundAltitude();	
	
	float calculateAltitudeDifference();
	
private:
	AP_HAL::Device *dev;
	
	
	
};

#endif //_MS5611_H
