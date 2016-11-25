#include "dps6015a.h"

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define PSU_PORT SD2

bool turn_on_output(uint8_t address) {
    if(address < 1 || address > 99) {
        //Invalid address
        return false;
    }

    chprintf((BaseSequentialStream *)&PSU_PORT, ":%02uso1\n", address);

    return true;
}

bool turn_off_output(uint8_t address) {
    if(address < 1 || address > 99) {
        //Invalid address
        return false;
    }

    chprintf((BaseSequentialStream *)&PSU_PORT, ":%02uso0\n", address);

    return true;
}

bool set_output_voltage(uint8_t address, float voltage) {
    if(address < 1 || address > 99) {
        //Invalid address
        return false;
    }
    uint16_t voltage_setpoint = (uint16_t)(voltage * 100);
    chprintf((BaseSequentialStream *)&PSU_PORT, ":%02usu%04u\n", address,
            voltage_setpoint);
    return true;
}

bool set_output_current(uint8_t address, float current) {
    if(address < 1 || address > 99) {
        //Invalid address
        return false;
    }
    uint16_t current_setpoint = (uint16_t)(current * 100);
    chprintf((BaseSequentialStream *)&PSU_PORT, ":%02usi%04u\n", address,
            current_setpoint);
    return true;
}
