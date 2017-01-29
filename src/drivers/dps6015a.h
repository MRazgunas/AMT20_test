/*
 * dps6015a.h
 *
 *  Created on: 2016-11-25
 *      Author: matas
 */

#ifndef SRC_DRIVERS_DPS6015A_H_
#define SRC_DRIVERS_DPS6015A_H_

#include "ch.h"
#include "hal.h"

typedef struct _dps6015a_state {
    bool output_on;
    float voltage_set; //Voltage setpoint
    float current_set; //Current setpoint
    float voltage_out; //Voltage output
    float current_out; //Current output
    uint8_t switch_state;
    bool link_active;
} dps6015a_state;

enum switch_state {
    SWITCH_CLOSED = 0,
    SWITCH_CV = 1,
    SWITCH_CC = 2,
    SWITCH_OPEN = 3,
};

void set_output_state(bool on);
void set_output_voltage(float voltage);
void set_output_current(float current);
void init_dps6015a(void);
dps6015a_state get_psu_state(void);


#endif /* SRC_DRIVERS_DPS6015A_H_ */
