/*
  per-motor compass compensation
 */

#include "AP_Compass.h"
//#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;


// constructor
Compass_PerMotor::Compass_PerMotor(Compass &_compass) :
    compass(_compass)
{
//    AP_Param::setup_object_defaults(this, var_info);
}

// return current scaled motor output
float Compass_PerMotor::scaled_output(uint8_t motor)
{
    if (!have_motor_map) {
//        if (SRV_Channels::find_channel(SRV_Channel::k_motor1, motor_map[0]) &&
//            SRV_Channels::find_channel(SRV_Channel::k_motor2, motor_map[1]) &&
//            SRV_Channels::find_channel(SRV_Channel::k_motor3, motor_map[2]) &&
//            SRV_Channels::find_channel(SRV_Channel::k_motor4, motor_map[3])) {
//            have_motor_map = true;
//        }
    }
    if (!have_motor_map) {
        return 0;
    }
    
    // this currently assumes first 4 channels. 
    uint16_t pwm = 0; //hal.rcout->read_last_sent(motor_map[motor]);

    // get 0 to 1 motor demand
    float output = 0; //(hal.rcout->scale_esc_to_unity(pwm)+1) * 0.5;

    if (output <= 0) {
        return 0;
    }
    
    // scale for voltage
    output *= voltage;

    // apply expo correction
    output = powf(output, expo);
    return output;
}

// per-motor calibration update
void Compass_PerMotor::calibration_start(void)
{
    for (uint8_t i=0; i<4; i++) {
        field_sum[i].zero();
        output_sum[i] = 0;
        count[i] = 0;
        start_ms[i] = 0;
    }

    // we need to ensure we get current data by throwing away several
    // samples. The offsets may have just changed from an offset
    // calibration
    for (uint8_t i=0; i<4; i++) {
        compass.read();
//        hal.scheduler->delay(50);
    }
    
    base_field = compass.get_field(0);
    running = true;
}

// per-motor calibration update
void Compass_PerMotor::calibration_update(void)
{
    uint32_t now = AP_HAL::millis();
    
    // accumulate per-motor sums
    for (uint8_t i=0; i<4; i++) {
        float output = scaled_output(i);

        if (output <= 0) {
            // motor is off
            start_ms[i] = 0;
            continue;
        }
        if (start_ms[i] == 0) {
            start_ms[i] = now;
        }
        if (now - start_ms[i] < 500) {
            // motor must run for 0.5s to settle
            continue;
        }

        // accumulate a sample
        field_sum[i] += compass.get_field(0);
        output_sum[i] += output;
        count[i]++;
    }
}

// calculate per-motor calibration values
void Compass_PerMotor::calibration_end(void)
{
    for (uint8_t i=0; i<4; i++) {
        if (count[i] == 0) {
            continue;
        }

        // calculate effective output
        float output = output_sum[i] / count[i];

        // calculate amount that field changed from base field
        Vector3f field_change = base_field - (field_sum[i] / count[i]);
        if (output <= 0) {
            continue;
        }
        
        Vector3f c = field_change / output;
        compensation[i] = c;
    }

    // enable per-motor compensation
    enable = 1;
    
    running = false;
}

/*
  calculate total offset for per-motor compensation
 */
void Compass_PerMotor::compensate(Vector3f &offset)
{
    offset.zero();

    if (running) {
        // don't compensate while calibrating
        return;
    }

    for (uint8_t i=0; i<4; i++) {
        float output = scaled_output(i);

        const Vector3f &c = compensation[i];

        offset += c * output;
    }
}
