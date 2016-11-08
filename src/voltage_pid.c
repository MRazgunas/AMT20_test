#include "ch.h"
#include "hal.h"

#include "voltage_pid.h"
#include "parameters_d.h"

static float volt_p_term, volt_i_term, volt_d_term;
static uint16_t rpm_out = 0;
static float i_temp_volt = 0.0f;
static float d_temp_volt = 0.0f;


uint16_t apply_voltage_pid(float target_voltage, float voltage) {

    float Kp_volt = volt_pid_p;
    float Ki_volt = volt_pid_i;
    float Kd_volt = volt_pid_d;

    //static uint32_t last_time = 0;
    // uint32_t now = ST2MS(chVTGetSystemTime());
    // uint32_t dt = now - last_time;
    // last_time = now;

    float err = target_voltage - voltage;

    //Calculate P term
    volt_p_term = Kp_volt * err;

    //Don't change integral if output is saturated
    if(rpm_out > 3000  && rpm_out < 8000)
        i_temp_volt += (err);
    //Calculate I term
    volt_i_term = Ki_volt * i_temp_volt;

    volt_d_term = Kd_volt * (err - d_temp_volt);
    d_temp_volt = err;

    rpm_out = volt_p_term + volt_i_term + volt_d_term;
    if(rpm_out > 8000)
        rpm_out = 8000;
    else if(rpm_out < 3000)
        rpm_out = 3000;

    return rpm_out;
}
