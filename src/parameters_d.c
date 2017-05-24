#include "ch.h"
#include "hal.h"

#include "parameters_d.h"
#include "parameters.h"
#include "telemetry.h"

#define GSCALAR(t, v, name, def) { t, name, k_param_ ## v, &v, def , 0}
#define GSCALARA(t, v, arr, name, def) { t, name, k_param_ ## v, &arr, def , 0} //for array type

int16_t format_version;

int16_t stream_rates[NUM_STREAMS];
float encoder_offset;
int16_t point_mode;
float point_angle;
float cycle_time;
float point_pitch;

const struct Info var_info[] = {
        // @Param: FORMAT_VERSION
        // @DisplayName: Eeprom format version number
        // @Description: This value is incremented when changes are made to the eeprom format
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, format_version, "FORMAT_VERSION", 0),

        // @Param: SR_PARAM
        // @DisplayName: Parameter stream frequency
        // @Description: This is frequency of param stream
        // @User: Advanced
        GSCALARA(AP_PARAM_INT16, stream_param, stream_rates[STREAM_PARAMS], "SR_PARAM", 10),

        // @Param: SR_SENSOR
        // @DisplayName: Sensor stream frequency
        // @Description: This is frequency of sensor stream
        // @User: Advanced
        GSCALARA(AP_PARAM_INT16, stream_sensor, stream_rates[STREAM_RAW_SENSORS], "SR_SENSOR", 10),

        // @Param: SR_RC_CHAN
        // @DisplayName: RC Channels stream frequency
        // @Description: This is frequency of RC channel streams
        // @User: Advanced
        GSCALARA(AP_PARAM_INT16, stream_rc_chan, stream_rates[STREAM_RC_CHANNELS], "SR_RC_CHANN", 10),

        // @Param: SR_PID_CONT
        // @DisplayName: PID controller stream frequency
        // @Description: This is frequency of PID controller streams
        // @User: Advanced
        GSCALARA(AP_PARAM_INT16, stream_controller, stream_rates[STREAM_RAW_CONTROLLER], "SR_PID_CONT", 10),

        GSCALARA(AP_PARAM_INT16, stream_req_data, stream_rates[STREAM_REQUEST_DATA_STREAM], "SR_STR_REQ", 1),

        // @Param: SR_PID_CONT
        // @DisplayName: PID controller stream frequency
        // @Description: This is frequency of PID controller streams
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, encoder_offset, "ENCD_OFFSET", 0.0f),

        GSCALAR(AP_PARAM_INT16, point_mode, "POINT_MODE", 0),

        GSCALAR(AP_PARAM_FLOAT, point_angle, "POINT_ANGLE", 90.0f),

        GSCALAR(AP_PARAM_FLOAT, cycle_time, "CYCLE_TIME", 2.0f),

        GSCALAR(AP_PARAM_FLOAT, point_pitch, "POINT_PITCH", 0.0f),

        AP_VAREND,
};


void load_parameters(void) {
    init_param_lib(var_info);
    if(!check_var_info()) {
        chSysHalt("Bad var_info table");
    }

    if(!load_value_using_pointer(&format_version) ||
            format_version != k_format_version) {
        //erase all parameters
        //TODO: debug message for erasing eeprom
        erase_all();

        set_and_save_using_pointer(&format_version, (float)k_format_version, false);
        //save the current format version
    }
    load_all_parameters();

}

