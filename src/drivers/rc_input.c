#include "ch.h"
#include "hal.h"

#include "rc_input.h"
#include "parameters_d.h"

#define RC_MAX 2200
#define RC_MIN 900

static virtual_timer_t rc_timeout;

icucnt_t last_width, last_period;
static bool init = false;

/*
 * RC timeout timer callback.
 */
static void rc_timeout_cb(void *arg) {
    (void) arg;
    last_width = 0;
}

static void icuwidthcb(ICUDriver *icup) {
    uint16_t width = icuGetWidthX(icup);

    if(width < RC_MAX && width > RC_MIN)
        last_width = width;

    /* Set timeout virtual timer if we don't get more callback
    * int given time it will set rc to 0*/
    chSysLockFromISR();
    chVTSetI(&rc_timeout, MS2ST(200), rc_timeout_cb, NULL);
    chSysUnlockFromISR();
}

static void icuperiodcb(ICUDriver *icup) {

  last_period = icuGetPeriodX(icup);
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,               /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

void init_rc_input(void) {
    icuStart(&ICUD8, &icucfg);
    icuStartCapture(&ICUD8);
    icuEnableNotifications(&ICUD8);

    /* RPM timeout timer initialization.*/
    chVTObjectInit(&rc_timeout);

    init = true;

}

float get_norm_rc_input(void) {
    if(!init)
        return 0.0f;

    float ret = ((float)last_width - rc1_min) / (float)(rc1_max - rc1_min);
    if (rc1_rev == -1) {
        ret = 1.0f - ret;
    }

    if(ret > 1.0f) ret = 1.0f;
    else if (ret < 0.0f) ret = 0.0f;

    return ret;
}

uint16_t get_rc_pwm(void) {
    if(!init)
        return 0.0f;

    return last_width;
}

