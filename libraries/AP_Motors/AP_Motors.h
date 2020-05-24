#ifndef _AP_MOTORS_H
#define _AP_MOTORS_H
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>

#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f
#define THRUST_CURVE_EXPO AP_MOTORS_THST_EXPO_DEFAULT

#define SCHEDULER_DEFAULT_LOOP_RATE 400
#define LOOPRATE SCHEDULER_DEFAULT_LOOP_RATE

#define AP_MOTORS_NUM 4

#define AP_MOTORS_MOT_1 0U
#define AP_MOTORS_MOT_2 1U
#define AP_MOTORS_MOT_3 2U
#define AP_MOTORS_MOT_4 3U
#define AP_MOTORS_MOT_5 4U
#define AP_MOTORS_MOT_6 5U
#define AP_MOTORS_MOT_7 6U
#define AP_MOTORS_MOT_8 7U
#define AP_MOTORS_MOT_9 8U
#define AP_MOTORS_MOT_10 9U
#define AP_MOTORS_MOT_11 10U
#define AP_MOTORS_MOT_12 11U

#define AP_MOTORS_ESC_1 2U
#define AP_MOTORS_ESC_2 0U
#define AP_MOTORS_ESC_3 3U
#define AP_MOTORS_ESC_4 1U

#define AP_MOTORS_ESC_5 4U
#define AP_MOTORS_ESC_6 5U
#define AP_MOTORS_ESC_7 6U
#define AP_MOTORS_ESC_8 7U
#define AP_MOTORS_ESC_9 8U
#define AP_MOTORS_ESC_10 9U
#define AP_MOTORS_ESC_11 10U
#define AP_MOTORS_ESC_12 11U


#define AP_MOTORS_MATRIX_YAW_FACTOR_CW    -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

typedef struct
{
	uint8_t motor_num;
	float angleDegrees;
	float rollFactor;
	float pitchFactor;
	float yawFactor;
	float throttleFactor;
} AP_MotorsParameters;

enum AP_MotorFrameClass
{
	MOTOR_FRAME_QUAD = 0,
	MOTOR_FRAME_HEXA = 1,
};

enum AP_MotorFrameType
{
	MOTOR_FRAME_TYPE_PLUS = 0,
	MOTOR_FRAME_TYPE_X = 1,
};

enum AP_MotorMode
{
	SHUT_DOWN = 0,			// all motors stop
	SPIN_WHEN_ARMED = 1,	// all motors at spin when armed
	THROTTLE_UNLIMITED = 2,	// throttle is no longer constrained by start up procedure
};

class AP_Motors
{
	
public:
	AP_Motors(){}
	
	void SetRoll(float _rollInput) { rollInput = _rollInput; };        // range -1 ~ +1
	void SetPitch(float _pitchInput) { pitchInput = _pitchInput; };    // range -1 ~ +1
	void SetYaw(float _yawInput) { yawInput = _yawInput; };            // range -1 ~ +1
	void SetThrottle(float _throttleInput) { throttleInput = _throttleInput; };   // range 0 ~ 1
	void SetBattVoltage(float _battery_voltage) {battery_voltage = _battery_voltage; };
	void SetBattVoltage_estimate(float _battery_voltage_estimate) {battery_voltage_estimate = _battery_voltage_estimate; };
	void SetVoltageMax(float _battery_voltage_max) {battery_voltage_max = _battery_voltage_max; };
	void SetVoltageMin(float _battery_voltage_min) {battery_voltage_min = _battery_voltage_min; };	
	void Enable_Voltage_Compensation() {voltage_compensation_enable = true;};
	void Disable_Voltage_Compensation() {voltage_compensation_enable = false;};
		
	float GetRoll() const { return rollInput; }
	float GetPitch() const { return pitchInput; }
	float GetYaw() const { return yawInput; }
	float GetThrottle() const { return throttleInput;}
	
	void SetFrameClassAndType(AP_MotorFrameClass _frameClass, AP_MotorFrameType _frameType);
	void AddMotor( uint8_t _num, float _degrees, float _yaw_factor);
	void SetMotorsParameters(void);
	
	void SetMotorMode(AP_MotorMode _motorMode);
	float Update_Voltage_Compensation_Gain();
	void OutPut();
	
	
private:
	float rollInput;					// range -1 ~ +1
	float pitchInput;					// range -1 ~ +1
	float yawInput;						// range -1 ~ +1
	float throttleInput;				// range 0 ~ 1
	float battery_voltage;
	float battery_voltage_max;
	float battery_voltage_min;
	float battery_voltage_estimate;
	bool  voltage_compensation_enable = true;
	float throttle_hover;

	float rollFactor[AP_MOTORS_NUM]; // each motors contribution to roll
	float pitchFactor[AP_MOTORS_NUM]; // each motors contribution to pitch
	float yawFactor[AP_MOTORS_NUM];  // each motors contribution to yaw (normally 1 or -1)
	
	float thrustOutput[AP_MOTORS_NUM]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range

	bool armed;
	AP_MotorFrameClass motorFrameclass;
	AP_MotorFrameType motorFrametype;
	AP_MotorsParameters motorsParameters[AP_MOTORS_NUM];
	AP_MotorMode motorMode;
	
	LowPassFilterFloat  batt_voltage_filt;
};

#endif //_AP_MOTORS_H