#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_

#define FIRMWARE_STRING "0.1_alfa"

#include "parameters.h"

enum streams {STREAM_RAW_SENSORS,
              STREAM_RC_CHANNELS,
              STREAM_RAW_CONTROLLER,
              STREAM_PARAMS,
              NUM_STREAMS};


void init_telemetry(void);
void send_parameter_value_all(const char *param_name, ap_var_type param_type,
        float param_value);

extern uint16_t target_rpm;
extern uint16_t target_rrpm;
extern uint16_t throttle_servo;
extern float voltage;
extern bool engine_control;

#endif /* SRC_TELEMETRY_H_ */
