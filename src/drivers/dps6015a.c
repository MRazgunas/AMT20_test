#include "dps6015a.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "filters.h"
#include <stdlib.h>
#include <string.h>

#define PSU_PORT SD3

bool output_on = false;
uint16_t voltage_setpoint = 2500;
uint16_t current_setpoint = 100;

dps6015a_state psu_state = {0, 25.0f, 2.0f, 0.0f, 0.0f, 0, 0};
static virtual_timer_t psu_timeout;

static void psu_timeout_cb(void *arg) {
    (void) arg;
    psu_state.link_active = 0;
}

enum {
    WAITING_FOR_SEMI,
    RECEIVING_ADDRESS,
    RECEIVING_COMMAND,
    RECEIVING_VALUE,
};

void send_output_state(void);
void send_output_voltage(void);
void send_output_current(void);
void read_voltage_cmd(void);
void read_current_cmd(void);
void read_state_machine_cmd(void);
bool isCharNumber(char a);

static THD_WORKING_AREA(wadps6015a, 2000);
static THD_FUNCTION(dps6015a, arg) {
    (void) arg;

    msg_t charData;
    event_listener_t serialData;
    eventflags_t flags;
    (void) flags;
    uint8_t state = WAITING_FOR_SEMI;
    uint8_t counter = 0;
    chEvtRegisterMask((event_source_t *) chnGetEventSource(&PSU_PORT),
            &serialData, EVENT_MASK(1));

    char cmd_buf[2] = {0};
    uint8_t read_value_buff[10];
    uint32_t read_value = 0;

    set_output_state(true);

    chRegSetThreadName("dps6015a");
    while (true) {
        chEvtWaitOneTimeout(EVENT_MASK(1), TIME_INFINITE);
        flags = chEvtGetAndClearFlags(&serialData);
        if(flags != SD_BREAK_DETECTED) {
          (void) flags;
        }
        do {
            charData = chnGetTimeout(&PSU_PORT, TIME_IMMEDIATE);
            if(charData > 0xFF || charData == Q_TIMEOUT) {
                break;
            }

            chSysLock();
            chVTSetI(&psu_timeout, MS2ST(100), psu_timeout_cb, NULL);
            chSysUnlock();

            psu_state.link_active = 1;

            switch(state) {
                case WAITING_FOR_SEMI:
                    if(charData == ':') {
                        state = RECEIVING_ADDRESS;
                    }
                    break;
                case RECEIVING_ADDRESS:
                    if(++counter == 2) {
                        state = RECEIVING_COMMAND;
                        counter = 0;
                    }
                    break;
                case RECEIVING_COMMAND:
                    cmd_buf[counter++] = (uint8_t) charData;
                    if(counter == 2) {
                        counter = 0;
                        state = RECEIVING_VALUE;
                    }
                    break;
                case RECEIVING_VALUE:
                    read_value_buff[counter++] = charData;
                    if(counter == 10) {
                      counter = 0;
                      state = WAITING_FOR_SEMI;
                    }
                    if(charData == '\r' || charData == '\n') {
                        state = WAITING_FOR_SEMI;
                        counter = 0;
                        read_value = strtoul((char *)read_value_buff, NULL, 10);
                        if(strncmp(cmd_buf, "rv", 2) == 0) {
                            psu_state.voltage_out = read_value / 100.0f;
                        } else if(strncmp(cmd_buf, "rj", 2) == 0) {
                            //read_value = median_filter_psu(read_value);
                            psu_state.current_out = read_value / 100.0f;
                        } else if(strncmp(cmd_buf, "rc", 2) == 0) {
                            psu_state.switch_state = read_value;
                        }
                        break;
                    }
                    break;
            }
        } while(charData != Q_TIMEOUT);
    }
    /* This point may be reached if shut down is requested. */
}

static THD_WORKING_AREA(wadps6015atx, 600);
static THD_FUNCTION(dps6015atx, arg) {
    (void) arg;

    chRegSetThreadName("dps6015atx");
    uint8_t state = 0;

    while(true) {
        if(psu_state.link_active)
            palSetPad(GPIOC, GPIOC_LED1);
        else palClearPad(GPIOC, GPIOC_LED1);
        switch(state++) {
            case 0:
                send_output_state();
                break;
            case 1:
                read_voltage_cmd();
                break;
            case 2:
                read_current_cmd();
                break;
            case 3:
                read_state_machine_cmd();
                break;
            case 4:
                send_output_voltage();
                break;
            case 5:
                send_output_current();
                break;
        }
        if(state == 6) {
            state = 0;
        }
        chThdSleepMilliseconds(12);
    }

}

void set_output_state(bool on) {
    psu_state.output_on = on;
}

void send_output_state(void) {
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01so%d\n", psu_state.output_on);
}

void set_output_voltage(float voltage) {
    if(voltage > 25.0f) voltage = 25.0f;
    else if(voltage < 0.0f) voltage = 0.0f;
    voltage_setpoint = voltage * 100.0f;
}

void send_output_voltage(void) {
    psu_state.voltage_set = voltage_setpoint / 100.0f;
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01su%04u\n", voltage_setpoint);
}

void set_output_current(float current) {
    if(current > 15.0f) current = 15.0f;
    else if(current < 0.0f) current = 0.0f;
    current_setpoint = current * 100;
}

void send_output_current(void) {
    psu_state.current_set = current_setpoint / 100.0f;
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01si%04u\n", current_setpoint);
}

void read_voltage_cmd(void) {
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01rv\n");
}

void read_current_cmd(void) {
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01rj\n");
}

void read_state_machine_cmd(void) {
    chprintf((BaseSequentialStream *)&PSU_PORT, ":01rc\n");
}

bool isCharNumber(char a) {
    if(a >= '0' && a <= '9')
        return true;
    else return false;
}

dps6015a_state get_psu_state(void) {
    return psu_state;
}

uint8_t calculate_lrc(uint8_t cmd, uint8_t n) {
    uin16_t sum = 0;
    for(uint8_t i = 0; i < n; i++) {
        sum += cmd[i];
    }
    return sum % 27 + 64;
}

void init_dps6015a() {
    chVTObjectInit(&psu_timeout);

    chThdCreateStatic(wadps6015a, sizeof(wadps6015a), NORMALPRIO, dps6015a, NULL);
    chThdCreateStatic(wadps6015atx, sizeof(wadps6015atx), NORMALPRIO, dps6015atx, NULL);
}


