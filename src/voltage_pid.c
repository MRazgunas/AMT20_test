#include "ch.h"
#include "hal.h"
#include "stdlib.h"

#include "voltage_pid.h"
#include "parameters_d.h"

float volt_p_term = 0.0f;
float volt_i_term = 0.0f;
float volt_d_term = 0.0f;
static uint16_t rpm_out = 0;
static float rpm_out_lpf = 0;
static float i_temp_volt = 0.0f;
static float d_temp_volt = 0.0f;
static systime_t last_ex = 0;

void reset_volt_integrator(void) {
    i_temp_volt = 0.0f;
}


uint16_t apply_voltage_pid(float target_voltage, float voltage, float thr, uint16_t rpm) {

    float dt = ST2US(abs(chVTGetSystemTime() - last_ex))/ 1000000.0f;
    last_ex = chVTGetSystemTime();

    static float Kp_volt = 0.0f;
    static float Ki_volt = 0.0f;
    static float Kd_volt = 0.0f;

    if(volt_pid_i != Ki_volt) {
        reset_volt_integrator();
    }

    Kp_volt = volt_pid_p;
    Ki_volt = volt_pid_i;
    Kd_volt = volt_pid_d;

    float err = target_voltage - voltage;

    volt_p_term = Kp_volt * err;
    int16_t rpm_err = (int16_t)rpm_out - (int16_t)rpm;
    //Don't change integral if output or throttle is saturated or rpm controller hasn't stabilized
    if((rpm_out > 2000  && rpm_out < 8000) && thr < 0.999f ){//&& (err < -1.0f || err > 1.0f)){
        if((rpm_err < 500 && rpm_err > -500) || rpm_out < 3600) { //&&
            i_temp_volt += (err);
        }
    }
    //Calculate I term
    volt_i_term = Ki_volt * i_temp_volt * dt;

    volt_d_term = Kd_volt * (err - d_temp_volt) / dt;
    d_temp_volt = err;
//    if(err > 0.8f || err < -0.8f)
    rpm_out = 2000.0f + volt_p_term + volt_i_term + volt_d_term; //Offset by 2000RPM

    rpm_out_lpf = rpm_out_lpf - (rpm_out_lpf_beta * (rpm_out_lpf - rpm_out));

    rpm_out = rpm_out_lpf;

    if(rpm_out > 8000)
        rpm_out = 8000;
    else if(rpm_out < 2000)
        rpm_out = 2000;

    if(voltage < 4.0f)
        rpm_out = 2000;

    return rpm_out;
}

