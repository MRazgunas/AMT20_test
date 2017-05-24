/*
 ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "math.h"

#include "eeprom.h"

#include "Storm32_telemetry.h"
#include "telemetry.h"
#include "parameters_d.h"
#include "km_math.h"

enum State_machine {
    WAITING_FOR_STORM_BOOTUP,
    CALIBRATING_ZERO_POSITION,
    WORKING,
}state;


/*
 * Blinker thread #1.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

    (void) arg;

    chRegSetThreadName("blinker");
    while (true) {
		palSetPad(GPIOC, GPIOC_LED1);
        chThdSleepMilliseconds(250);
        palClearPad(GPIOC, GPIOC_LED1);
        chThdSleepMilliseconds(250);
    }
}

static void zero_point(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  //chSysLockFromISR();
  //chSysUnlockFromISR();
  QEID1.tim->CNT = 0;
  if(state == CALIBRATING_ZERO_POSITION) {
      state = WORKING;
      set_target_angles(0.0f, 0.0f, 0.0f, true);
  }
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
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, zero_point},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static const SerialConfig sd3_config =
{
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

int main(void) {
    halInit();
    chSysInit();

    chThdSetPriority(NORMALPRIO+2);
    /*
     * Activates the serial driver 1 using the driver default configuration.
     * PA9(TX) and PA10(RX) are routed to USART1.
     */
    sdStart(&SD1, NULL);
    sdStart(&SD3, &sd3_config);

    init_eeprom();
    load_parameters();
    init_telemetry();

    static QEIConfig qeicfg = {
      QEI_MODE_QUADRATURE,
      QEI_SINGLE_EDGE,
      //QEI_BOTH_EDGES,
      QEI_DIRINV_FALSE,
      0,
      0,
      2047,
    };

    qeiStart(&QEID1, &qeicfg);
    qeiEnable(&QEID1);

    init_sotmr32_telemetry();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-2, Thread1,
    NULL);

   // uint32_t teset = QEID1.tim->ARR;

    float yaw_angle = 0.0f;
    //chThdSleep(S2ST(30));

    state = WAITING_FOR_STORM_BOOTUP;
    float target_yaw = 0.0f;
    float storm_last_reading = 0.0f;
    int32_t storm_rotation_count = 0;

    uint32_t count1 = 0;

    while (TRUE) {
        systime_t current_time = ST2MS(chVTGetSystemTime());

        switch(state) {
            case WAITING_FOR_STORM_BOOTUP:
                extStart(&EXTD1, &extcfg);
                if(get_storm_state()->state == 6) {
                    state = CALIBRATING_ZERO_POSITION;
                    storm_rotation_count = 0;
                }
                break;
            case CALIBRATING_ZERO_POSITION:
                set_target_angles(0.0f, 0.0f, 370.0f, true);
                break;
            case WORKING: {
                extStop(&EXTD1);
                if(get_storm_state()->state != 6) {
                    state = WAITING_FOR_STORM_BOOTUP;
                    break;
                }
                float storm_yaw = get_storm_state()->imu1_yaw / 100.0f;
                float yaw_diff = storm_yaw - storm_last_reading;

                if(yaw_diff < -200.0f) {
                    storm_rotation_count++;
                } else if(yaw_diff > 200.0f) {
                    storm_rotation_count--;
                }

                storm_last_reading = storm_yaw;

                uint32_t cycle = cycle_time * 50;
                //test
                switch(point_mode) {
                case 1:
                    if (count1 % cycle == 0) {
                        if (target_gimbal_yaw == 0.0f) {
                            target_gimbal_yaw = point_angle;
                        }
                        else if (target_gimbal_yaw == point_angle) {
                            target_gimbal_yaw = -point_angle;
                        }
                        else {
                            target_gimbal_yaw = 0.0f;
                        }
                    }
                    count1++;
                    target_gimbal_pitch = point_pitch;
                case 2:
                    if (count1 % cycle == 0) {
                        if (target_gimbal_yaw == 0.0f) {
                            target_gimbal_yaw = point_angle;
                        }
                        else {
                            target_gimbal_yaw = 0.0f;
                        }
                    }
                    count1++;
                    target_gimbal_pitch = point_pitch;
                }

             /*
*/                //======================================


                float encoder_pos = wrap180(QEID1.tim->CNT * 360.0f/2048.0f - encoder_offset, 1);
                float diff = wrap180(encoder_pos - target_gimbal_yaw, 1);
                //float diff = encoder_pos - target_gimbal_yaw;
                storm_yaw = wrap180(storm_yaw, 1) + 360.0f*storm_rotation_count;
                target_yaw = storm_yaw - diff;


                set_target_angles(target_gimbal_pitch, 0.0f, target_yaw, true);
               // chprintf((BaseSequentialStream *)&SD1, "%i %i %i\n", yaw, encd, diff);
                break;
            }
        }
        update_storm32();

        chThdSleepMilliseconds(20);
    }
}
