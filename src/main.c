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

#include "servo.h"
#include "rc_input.h"
#include "rpm.h"

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

/*
 * Blinker thread #2.
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

    (void) arg;

    chRegSetThreadName("blinker");
    while (true) {
        palSetPad(GPIOC, GPIOC_LED2);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOC, GPIOC_LED2);
        chThdSleepMilliseconds(500);
    }
}

int main(void) {
    halInit();
    chSysInit();

    init_servo();
    init_rc_input();
    init_rpm();

    uint16_t width = 0;
    uint16_t rpm = 0;

    /*
     * Activates the serial driver 1 using the driver default configuration.
     * PA9(TX) and PA10(RX) are routed to USART1.
     */
    //sdStart(&SD1, NULL);

    /*
     * Creates the example threads.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
    NULL);
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 1, Thread2,
    NULL);

    while (TRUE) {
        width = get_rc_input();
        if(width > 900 && width < 2000) {
            set_servo_pwm(width);
        }
        rpm = get_rpm();
        chThdSleepMilliseconds(5);
    }
}
