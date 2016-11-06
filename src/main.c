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
#include "voltage_pid.h"

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
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN0.
 */
static const ADCConversionGroup adcVoltage = {
FALSE,
1,
NULL,
NULL,//adcerrorcallback,
0, 0, /* CR1, CR2 */
0, /* SMPR1 */
ADC_SMPR2_SMP_AN6(ADC_SAMPLE_41P5), /* SMPR2 */
ADC_SQR1_NUM_CH(1), /* SQR1 */
0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN6)  /* SQR3 */
};

static adcsample_t samples1[8];

float measure_voltage(void) {
    float voltage;

    adcConvert(&ADCD1, &adcVoltage, samples1, 8);
    uint16_t sum = 0;
    for(int i = 0; i < 8; i++) {
        sum += samples1[i];
    }
    float avg = sum / 8.0f;

    voltage = avg*0.0282307938;

    return voltage;
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
    adcStart(&ADCD1, NULL);
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
    uint16_t run_timeout = 0;

    while (TRUE) {
        width = get_rc_input();
        apply_filter();
        rpm = get_rpm();
        voltage = measure_voltage();

        uint16_t target_rrpm = apply_voltage_pid(target_voltage, voltage);

        //PWM -> RPM
        target_rpm = 6.5359*width - 4614.4;
        if(target_rpm < 2000) target_rpm = 2000;
        else if(target_rpm > 8000) target_rpm = 8000;

        thr = apply_rpm_pid(target_rpm, rpm);

        if(thr > 0.4f) thr = 0.4f;

        //Servo min range 1012 max 1930
        throttle_servo = 918 * thr + 1012;

        if(!running) {
            throttle_servo = width;
            reset_integrator();
        }

        if(rpm > 8000) {
            reset_integrator();
            throttle_servo = 1012;
        }

        if(throttle_servo < 1012) throttle_servo = 1012;
        else if(throttle_servo > 1930) throttle_servo = 1930;

        if(rpm > 5500 && !running) {
            tmp++;
        }
        else
            tmp = 0;

        if(tmp > 100)
            running = true;

        if(rpm == 0 && running) {
            run_timeout++;
        } else {
            run_timeout = 0;
        }
        if(run_timeout > 50) {
            running = false;
        }

        set_servo_pwm(throttle_servo);

        chThdSleepMilliseconds(20);
    }
}
