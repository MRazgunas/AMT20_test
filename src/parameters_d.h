#ifndef SRC_PARAMETERS_D_H_
#define SRC_PARAMETERS_D_H_
#include "ch.h"
#include "hal.h"

#include "parameters.h"
#include "parameters_d.h"
#include "telemetry.h"

//////////////////////////////////////////////////////////////////
// STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
// COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
static const uint16_t k_format_version = 1;
//////////////////////////////////////////////////////////////////


// EEPROM layout
enum {
    k_param_format_version = 0,
    k_param_stream_param,
    k_param_stream_sensor,
    k_param_stream_rc_chan,
    k_param_stream_controller,
    k_param_encoder_offset,
    k_param_stream_req_data,
    k_param_point_mode,
    k_param_point_angle,
    k_param_cycle_time,
    k_param_point_pitch,
};

extern int16_t stream_rates[NUM_STREAMS];
extern float encoder_offset;
extern int16_t point_mode;
extern float point_angle;
extern float cycle_time;
extern float point_pitch;

void load_parameters(void);


#endif /* SRC_PARAMETERS_D_H_ */
