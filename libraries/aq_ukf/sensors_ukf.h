#ifndef _sensors_ukf_h
#define _sensors_ukf_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <hal_drivers/drv_hrt.h>

typedef struct{
	uint64_t timestamp;
	uint64_t integral_dt;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float x_integral;
	float y_integral;
	float z_integral;
	float temperature;
	float range_rad_s;
	float scaling;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
	int16_t temperature_raw;
	uint32_t device_id;
} sensor_gyro_s;

typedef struct{
	uint64_t timestamp;
	uint64_t integral_dt;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float x_integral;
	float y_integral;
	float z_integral;
	float temperature;
	float range_m_s2;
	float scaling;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
	int16_t temperature_raw;
	uint32_t device_id;
} sensor_accel_s;

typedef struct{
	float pressure;
	float altitude;
	float temperature;
	uint64_t timestamp;
	uint64_t error_count;
	uint8_t updateFlag;
} sensor_baro_s;


typedef struct{
	uint64_t timestamp;
	uint64_t error_count;
	float x;
	float y;
	float z;
	float range_ga;
	float scaling;
	float temperature;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
	uint32_t device_id;
	
	float inclination;
	float declination;
	uint8_t updateFlag;
	uint8_t enabled;
} sensor_mag_s;


typedef struct {
	unsigned long iTOW;
	int32_t lat_int;
	int32_t lon_int;
	int32_t height_int;
	int32_t delta_lat;
	int32_t delta_lon;
	double lat;
	double lon;
	float height;   // above mean sea level (m)
	float hAcc;     // horizontal accuracy est (m)
	float vAcc;     // vertical accuracy est (m)

	float delta_velN;
	float delta_velE;
	float delta_velD;
	int32_t velN_int;
	int32_t velE_int;
	int32_t velD_int;
	float velN;     // north velocity (m/s)
	float velE;     // east velocity (m/s)
	float velD;     // down velocity (m/s)
	float speed;    // ground speed (m/s)
	int32_t heading_int;  // course over ground (deg)
	float heading;  // course over ground (deg)
	float sAcc;     // speed accuracy est (m/s)
	float cAcc;     // course accuracy est (deg)
	float pDOP;     // Position Dilution of Precision
	float hDOP;     // Horizontal DOP
	float vDOP;     // Vertical DOP
	float tDOP;     // Time DOP
	float nDOP;     // Northing DOP
	float eDOP;     // Easting DOP
	float gDOP;     // Geometric DOP

	unsigned long TPtowMS;    // timepulse time of week (ms)
	unsigned long lastReceivedTPtowMS;
	unsigned long lastPosUpdate;
	unsigned long lastVelUpdate;
	unsigned long lastMessage;

	uint8_t fix_type;
	uint8_t satellites_used;
	
	uint8_t carrSoln;

	uint8_t vel_updateFlag;
	uint8_t pos_updateFlag;
	
} gpsStruct_t;




typedef struct{
	uint64_t timestamp;
	uint8_t sensor_id;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t quality;
} sensor_opticalFlow_s;

extern sensor_gyro_s ap_gyro;
extern sensor_accel_s ap_accel;
extern sensor_baro_s ap_baro;
extern sensor_mag_s ap_mag;
extern gpsStruct_t gnssData;
extern sensor_opticalFlow_s ap_opticalFlow;


#ifdef __cplusplus
}
#endif

#endif //_sensors_ukf_h
