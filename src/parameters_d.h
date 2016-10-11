#ifndef SRC_PARAMETERS_D_H_
#define SRC_PARAMETERS_D_H_
#include "ch.h"
#include "hal.h"

#include "parameters.h"

//////////////////////////////////////////////////////////////////
// STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
// COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
static const uint16_t k_format_version = 2;
//////////////////////////////////////////////////////////////////


// EEPROM layout
enum {
    k_param_format_version = 0,
    k_param_rpm_pid_p,
    k_param_rpm_pid_i,
    k_param_rpm_pid_d,
};



extern float rpm_pid_p;
extern float rpm_pid_i;
extern float rpm_pid_d;

void load_parameters(void);


#endif /* SRC_PARAMETERS_D_H_ */
