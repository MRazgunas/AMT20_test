#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_

#define FIRMWARE_STRING "0.1_alfa"

#include "parameters.h"

void init_telemetry(void);
void send_parameter_value_all(const char *param_name, ap_var_type param_type,
        float param_value);

extern uint16_t target_rpm;
extern uint16_t lpf_rpm;

#endif /* SRC_TELEMETRY_H_ */
