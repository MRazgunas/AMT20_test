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

bool turn_on_output(uint8_t address);
bool turn_off_output(uint8_t address);
bool set_output_voltage(uint8_t address, float voltage);
bool set_output_current(uint8_t address, float current);



#endif /* SRC_DRIVERS_DPS6015A_H_ */
