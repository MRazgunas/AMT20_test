#include "ch.h"
#include "hal.h"

#include "servo.h"
#include "parameters_d.h"

static bool init = false;
static uint16_t thr_pwm = 0;

static PWMConfig pwmcfg = { 1000000, /* 1MHz PWM clock frequency */
        20000, /* PWM period 20 milli  second */
        NULL, /* No callback */
        { { PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL },
          { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_DISABLED, NULL }, },
        0 , 0};


void init_servo(void) {
    pwmStart(&PWMD3, &pwmcfg);
    init = true;
}

void set_thr_pwm(uint16_t pulse_width) {
    if(!init)
        return;

    if(pulse_width > thr_max) pulse_width = thr_max;
    else if(pulse_width < thr_min) pulse_width = thr_min;

    thr_pwm = pulse_width;
    pwmEnableChannel(&PWMD3, 2, pulse_width);
}

uint16_t get_thr_pwm(void) {
    return thr_pwm;
}

void set_thr_position(float pos) { //range 0.0 to 1.0
    if(thr_rev == -1)
        pos = 1.0f - pos;

    uint16_t pwm = pos * (thr_max - thr_min) + thr_min;
    set_thr_pwm(pwm);
}


