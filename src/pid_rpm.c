#include "pid_rpm.h"
#include "parameters_d.h"
#include "stdlib.h"

static float thr_out = 0.0f;
static float i_temp = 0.0f;
static float d_temp = 0.0f;
float p_term = 0.0f;
float i_term = 0.0f;
float d_term = 0.0f;

float apply_rpm_pid(uint16_t target_rpm, uint16_t rpm) {

    float Kp_rpm = rpm_pid_p / 1000.0f;
    float Ki_rpm = rpm_pid_i / 1000.0f;
    float Kd_rpm = rpm_pid_d / 1000.0f;

    //static uint32_t last_time = 0;
    // uint32_t now = ST2MS(chVTGetSystemTime());
    // uint32_t dt = now - last_time;
    // last_time = now;

    float rpm_scaled = rpm / 10000.0f;
    float target_rpm_scaled = target_rpm / 10000.0f;

    float err = target_rpm_scaled - rpm_scaled;

    //Calculate P term
    p_term = Kp_rpm * err;

    //Don't change integral if output is saturated. 400RPM deadband
    if(thr_out < 1.0f && thr_out > 0.0f && (err > 0.01f || err < -0.01f))
        i_temp += (err);
    //Calculate I term
    i_term = Ki_rpm * i_temp;

    d_term = Kd_rpm * (err - d_temp);
    d_temp = err;

    thr_out = p_term + i_term + d_term;
    if(thr_out > 1.0f)
        thr_out = 1.0f;
    else if(thr_out < 0.0f)
        thr_out = 0.0f;

    return thr_out;
}

void reset_integrator(void) {
    i_temp = 0.0f;
}

