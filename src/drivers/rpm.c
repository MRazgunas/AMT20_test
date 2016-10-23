#include "ch.h"
#include "hal.h"

#include "rpm.h"
#include "parameters_d.h"

#define ICU_FREQUENCY 100000
#define MIN_RPM 1000

static bool init = false;

icucnt_t last_width_rpm, last_period_rpm;
uint16_t last_rpm = 0;
uint16_t lpf_rpm = 0;

static virtual_timer_t rpm_timeout;

/*
 * RPM timeout timer callback.
 */
static void rpm_timeout_cb(void *arg) {
    (void) arg;
    last_rpm = 0;
}
static void icuwidthcb(ICUDriver *icup) {
  last_width_rpm = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) {
    last_period_rpm = icuGetPeriodX(icup);

    last_rpm = (60 * ICU_FREQUENCY) / last_period_rpm;


    /* Set timeout virtual timer if we don't get more callback
     * int given time it will set rpm to 0*/
    chSysLockFromISR();
    chVTSetI(&rpm_timeout, MS2ST(60000/(MIN_RPM)), rpm_timeout_cb, NULL);
    chSysUnlockFromISR();
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_LOW,
  ICU_FREQUENCY,           /* 100kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

void init_rpm(void) {
    /* RPM timeout timer initialization.*/
    chVTObjectInit(&rpm_timeout);

    icuStart(&ICUD1, &icucfg);
    icuStartCapture(&ICUD1);
    icuEnableNotifications(&ICUD1);

    init = true;
}

void apply_filter(void) {
    //Low pass filter rpm
    lpf_rpm = lpf_rpm - (rpm_lpf_beta * (lpf_rpm - last_rpm));
}

uint16_t get_rpm(void) {
    if(!init)
        return 0;
    return lpf_rpm;
}



