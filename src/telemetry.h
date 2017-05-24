#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_

#define FIRMWARE_STRING "0.1_alfa"

#include "parameters.h"

enum streams {STREAM_RAW_SENSORS,
              STREAM_RC_CHANNELS,
              STREAM_RAW_CONTROLLER,
              STREAM_PARAMS,
              STREAM_REQUEST_DATA_STREAM,
              NUM_STREAMS};

extern float target_gimbal_pitch;
extern float target_gimbal_yaw;


void init_telemetry(void);
void send_parameter_value_all(const char *param_name, ap_var_type param_type,
        float param_value);

#endif /* SRC_TELEMETRY_H_ */
