#include "ch.h"
#include "hal.h"

#include "rpm.h"
#include "filters.h"
#include "parameters_d.h"
#include "stdlib.h"

#define ICU_FREQUENCY 100000
#define MIN_RPM 1000
#define DEBOUNCE_TIME_MS 5

static bool init = false;

uint16_t last_rpm = 0;
uint16_t lpf_rpm = 0;

systime_t last_rising_edge = 0; //Last detected rising edge. This is not debounce
systime_t last_signal = 0; //Last detected rising edge time. This is debounced

static uint8_t debounce_count = 0;

static virtual_timer_t rpm_timeout;
static virtual_timer_t rpm_debounce;

/*
 * RPM timeout timer callback.
 */
static void rpm_timeout_cb(void *arg) {
    (void) arg;
    last_rpm = 0;
}

/*
 * RPM debounce timer callback.
 */
static void rpm_debounce_cb(void *arg) {
    (void) arg;

    uint8_t pad = palReadPad(GPIOA, GPIOA_RPM);
    if(pad == PAL_HIGH && debounce_count == 3) {
        /* We still have high signal so it was true signal
         * calculate RPM and assign last edge time */
        last_rpm =  60e6 / ST2US(abs(last_rising_edge - last_signal));
        last_signal = last_rising_edge;
        last_rpm = median_filter(last_rpm); // median filter(3) rejects only one bad measurement

        debounce_count = 0;

        //Set timeout timer
        chSysLockFromISR();
        chVTSetI(&rpm_timeout, MS2ST(60000/(MIN_RPM)), rpm_timeout_cb, NULL);
        chSysUnlockFromISR();
    }
    else if(pad == PAL_HIGH) {
        debounce_count++;
        chSysLockFromISR();
        chVTSetI(&rpm_debounce, US2ST(100), rpm_debounce_cb, NULL);
        chSysUnlockFromISR();
    }
    else
        debounce_count = 0;
}

/* Triggered when rising edge is detected. Starts debounce timer*/
static void rpm_rising_edge(EXTDriver *extp, expchannel_t channel) {
    (void) extp;
    (void) channel;

    last_rising_edge = chVTGetSystemTimeX();
    debounce_count = 0;

    /* Set debounce timer. When it expires we check port state and if it is
     * still high we assume that it got right signal and calculate rpm */
    chSysLockFromISR();
    chVTSetI(&rpm_debounce, US2ST(100), rpm_debounce_cb, NULL);
    chSysUnlockFromISR();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, rpm_rising_edge},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

void init_rpm(void) {
    /* Virtual timers initialization.*/
    chVTObjectInit(&rpm_timeout);
    chVTObjectInit(&rpm_debounce);

    extStart(&EXTD1, &extcfg);
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



