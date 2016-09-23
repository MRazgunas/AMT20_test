#include "ch.h"
#include "hal.h"

#include "servo.h"

static bool init = false;

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

void set_servo_pwm(uint16_t pulse_width) {
    if(!init)
        return;

    pwmEnableChannel(&PWMD3, 2, pulse_width);
}


