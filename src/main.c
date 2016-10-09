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

float rpm_pid(uint16_t target_rpm, uint16_t rpm) {
    const float Kp_rpm = 0.02f;
    const float Ki_rpm = 0.02f;
    const float Kd_rpm = 0.0f;

    static float out = 0.0f;
    static float i_temp = 0;
    static float d_temp = 0;
    static uint32_t last_time = 0;
    uint32_t now = ST2MS(chVTGetSystemTime());
    uint32_t dt = now - last_time;
    last_time = now;

    float rpm_scaled = rpm / 10000.0f;
    float target_rpm_scaled = target_rpm / 10000.0f;

    float err = target_rpm_scaled - rpm_scaled;

    //Calculate P term
    float p_term = Kp_rpm * err;

    //Don't change integral if output is saturated
    if(out < 1.0f && out > 0.0f)
        i_temp += (err);
    //Calculate I term
    float i_term = Ki_rpm * i_temp;

    float d_term = Kd_rpm * (err - d_temp);
    d_temp = err;

    out = p_term + i_term + d_term;
    if(out > 1.0f)
        out = 1.0f;
    else if(out < 0.0f)
        out = 0.0f;

    return out;
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
    uint16_t lpf_rpm = 0;
    const float lpf_beta = 0.15f;
    uint16_t tmp = 0;

    while (TRUE) {
        width = get_rc_input();
        rpm = get_rpm();

        //Low pass filter rpm
        lpf_rpm = lpf_rpm - (lpf_beta * (lpf_rpm - rpm));

        //PWM -> RPM
        uint16_t target_rpm = 6.5359*width - 4614.4;
        if(target_rpm < 2000) target_rpm = 2000;
        else if(target_rpm > 8000) target_rpm = 8000;

        thr = rpm_pid(target_rpm, lpf_rpm);

        if(thr > 0.5f) thr = 0.5f;

        //Servo min range 1012 max 1930
        uint16_t throttle_servo = 918 * thr + 1012;
        if(throttle_servo < 1012) throttle_servo = 1012;
        else if(throttle_servo > 1930) throttle_servo = 1930;

        if(lpf_rpm > 6000 && !running) {
            tmp++;
        }
        else
            tmp = 0;

        if(tmp > 100)
            running = true;

        if(running)
            set_servo_pwm(throttle_servo);
        else
            set_servo_pwm(width);

        chThdSleepMilliseconds(20);
    }
}
