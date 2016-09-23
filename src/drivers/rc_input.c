#include "ch.h"
#include "hal.h"

#include "rc_input.h"

icucnt_t last_width, last_period;
static bool init = false;

static void icuwidthcb(ICUDriver *icup) {

  last_width = icuGetWidthX(icup);
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

    init = true;

}

uint16_t get_rc_input(void) {
    if(!init)
        return 0;
    return last_width;
}

