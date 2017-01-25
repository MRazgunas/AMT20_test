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
#include "dps6015a.h"

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

void apply_voltage_filter(float voltage_raw) {
    //Low pass filter rpm
    voltage = voltage - (volt_lpf_beta * (voltage - voltage_raw));
}

int main(void) {
    halInit();
    chSysInit();

    /*
     * Activates the serial driver 1 using the driver default configuration.
     * PA9(TX) and PA10(RX) are routed to USART1.
     */
    sdStart(&SD1, NULL);
    sdStart(&SD3, NULL);

    adcStart(&ADCD1, NULL);
    init_eeprom();

    /*
     * Creates the example threads.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
    NULL);

    //Dump first page of eeprom
//    uint8_t buff[EEPROM_PAGE_SIZE*3];
//    read_block(buff, 0x0, EEPROM_PAGE_SIZE*3);


    load_parameters();


    init_servo();
    init_rc_input();
    init_rpm();
    init_telemetry();


    float rc_in = 0;
    uint16_t rpm = 0;
    float thr = 0.0f;
    uint16_t run_timeout = 0;
    uint16_t over_rpm = 0;

   /* chThdSleepMilliseconds(2000);
    turn_on_output(1); */

    systime_t last_ex_time;

    while (TRUE) {
        last_ex_time = chVTGetSystemTime();
        rc_in = get_norm_rc_input();

        apply_filter();
        rpm = get_rpm();
        apply_voltage_filter(measure_voltage()); //This sets voltage

        //PWM -> RPM
       /* target_rpm = 6.5359*width - 4614.4;
        if(target_rpm < 2000) target_rpm = 2000;
        else if(target_rpm > 8000) target_rpm = 8000; */

        if(rpm > 8000) {
            over_rpm++;
        } else {
            over_rpm = 0;
        }
        if(over_rpm > 10) {
            over_rpm = 0;
            engine_state = ENGINE_EMERGENCY_SHUTDOWN;
            engine_control = false;
        }

        switch(engine_state) {
            case ENGINE_STOPED:
                thr = rc_in * max_man_thr;
                reset_integrator();
                reset_volt_integrator();
                if(rpm > 500) {
                    engine_state = ENGINE_WARMUP;
                }
                engine_control = false;
                //Set ignition to off
                break;
            case ENGINE_START_REQUESTED:
                break;
            case ENGINE_STARTING_WITH_CHOKE:
                break;
            case ENGINE_STARTING_WO_CHOKE:
                break;
            case ENGINE_WARMUP:
                thr = rc_in * max_man_thr;
                reset_integrator();
                reset_volt_integrator();
                if(engine_control == true) engine_state = ENGINE_RUNNING;
                break;
            case ENGINE_TRANSION_TO_RUNNING:
                break;
            case ENGINE_RUNNING:
                target_rrpm = 2000 + rc_in * 6000;//apply_voltage_pid(target_voltage, voltage, thr);
                thr = apply_rpm_pid(target_rrpm, rpm);
                if(rpm < 200) {
                    run_timeout++;
                } else {
                    run_timeout = 0;
                }
                if(run_timeout > 10) {
                    engine_state = ENGINE_STOPED;
                    engine_control = false;
                    break;
                }

                if(engine_control == false) engine_state = ENGINE_WARMUP;

                break;
            case ENGINE_EMERGENCY_SHUTDOWN:
                reset_integrator();
                reset_volt_integrator();
                thr = 0.0f;
                //Turn ignition off
                //Turn POWER off
                //
                break;
        }

        if(thr > 1.0f) thr = 1.0f;
        else if(thr < 0.0) thr = 0.0f;
        set_thr_position(thr);

        chThdSleepUntil(last_ex_time + MS2ST(20));
    }
}
