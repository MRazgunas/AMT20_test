#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_

#define FIRMWARE_STRING "0.1_alfa"

#include "parameters.h"

enum streams {STREAM_RAW_SENSORS,
              STREAM_RC_CHANNELS,
              STREAM_RAW_CONTROLLER,
              STREAM_PARAMS,
              NUM_STREAMS};

enum Control_State {
    ENGINE_STOPED = 0, //Always has to be 0
    ENGINE_START_REQUESTED,
    ENGINE_STARTING_WITH_CHOKE,
    ENGINE_STARTING_WO_CHOKE,
    ENGINE_WARMUP,
    ENGINE_TRANSION_TO_RUNNING,
    ENGINE_RUNNING,
    ENGINE_COOLDOWN,
    ENGINE_EMERGENCY_SHUTDOWN,
};


void init_telemetry(void);
void send_parameter_value_all(const char *param_name, ap_var_type param_type,
        float param_value);

extern uint16_t target_rpm;
extern uint16_t target_rrpm;
extern float voltage;
extern bool engine_control;
extern uint8_t engine_state;

#endif /* SRC_TELEMETRY_H_ */
