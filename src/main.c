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
#include "drivers/eeprom.h"

#include "telemetry.h"
#include "servo.h"
#include "rc_input.h"
#include "rpm.h"
#include "parameters_d.h"
#include "pid_rpm.h"

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

int main(void) {
    halInit();
    chSysInit();

    uint16_t width = 0;
    uint16_t rpm = 0;

    /*
     * Activates the serial driver 1 using the driver default configuration.
     * PA9(TX) and PA10(RX) are routed to USART1.
     */
    sdStart(&SD1, NULL);
    init_eeprom();

    /*
     * Creates the example threads.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
    NULL);

    load_parameters();

    //Dump first page of eeprom
    uint8_t buff[EEPROM_PAGE_SIZE];
    read_block(buff, 0x0, EEPROM_PAGE_SIZE);


    init_servo();
    init_rc_input();
    init_rpm();
    init_telemetry();

    bool running = false;
    float thr;
    uint16_t tmp = 0;

    while (TRUE) {
        width = get_rc_input();
        rpm = get_rpm();

        //PWM -> RPM
        target_rpm = 6.5359*width - 4614.4;
        if(target_rpm < 2000) target_rpm = 2000;
        else if(target_rpm > 8000) target_rpm = 8000;

        thr = apply_rpm_pid(target_rpm, rpm);

        if(thr > 0.5f) thr = 0.5f;

        //Servo min range 1012 max 1930
        throttle_servo = 918 * thr + 1012;

        if(!running) {
            throttle_servo = width;
        }

        if(throttle_servo < 1012) throttle_servo = 1012;
        else if(throttle_servo > 1930) throttle_servo = 1930;

        if(rpm > 6000 && !running) {
            tmp++;
        }
        else
            tmp = 0;

        if(tmp > 100)
            running = true;

        set_servo_pwm(throttle_servo);

        chThdSleepMilliseconds(20);
    }
}
