#include "ch.h"
#include "hal.h"

#include "parameters_d.h"
#include "parameters.h"
#include "telemetry.h"

#define GSCALAR(t, v, name, def) { t, name, k_param_ ## v, &v, def , 0}
#define GSCALARA(t, v, arr, name, def) { t, name, k_param_ ## v, &arr, def , 0} //for array type

int16_t format_version;
float rpm_pid_p;
float rpm_pid_i;
float rpm_pid_d;
float rpm_lpf_beta;
int16_t stream_rates[NUM_STREAMS];
float volt_pid_p, volt_pid_i, volt_pid_d;
float target_voltage;
float volt_lpf_beta;
int16_t pid_report;
float max_man_thr;
float d_lpf_beta;
float rpm_out_lpf_beta;

float load_ramp_speed;

//RC input
int16_t rc1_min;
int16_t rc1_max;
int16_t rc1_rev;

//thr output
int16_t thr_min;
int16_t thr_max;
int16_t  thr_rev;

float max_charge_current;


const struct Info var_info[] = {
        // @Param: FORMAT_VERSION
        // @DisplayName: Eeprom format version number
        // @Description: This value is incremented when changes are made to the eeprom format
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, format_version, "FORMAT_VERSION", 0),

        // @Param: RPM_PID_P
        // @DisplayName: RPM pid contant P
        // @Description: This value represent P of rpm pid controller
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, rpm_pid_p, "RPM_PID_P", 0),

        // @Param: RPM_PID_I
        // @DisplayName: RPM pid contant I
        // @Description: This value represent I of rpm pid controller
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, rpm_pid_i, "RPM_PID_I", 0),

        // @Param: RPM_PID_D
        // @DisplayName: RPM pid contant D
        // @Description: This value represent D of rpm pid controller
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, rpm_pid_d, "RPM_PID_D", 0),

        // @Param: RPM_LPF_BETA
        // @DisplayName: RPM LPF filter beta
        // @Description: This value is const of lpf filter for RPM
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, rpm_lpf_beta, "RPM_LPF_BETA", 0.15f),

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

        // @Param: VOLT_PID_P
        // @DisplayName: RPM LPF filter beta
        // @Description: This value is const of lpf filter for RPM
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, volt_pid_p, "VOLT_PID_P", 5),

        // @Param: VOLT_PID_I
        // @DisplayName: RPM LPF filter beta
        // @Description: This value is const of lpf filter for RPM
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, volt_pid_i, "VOLT_PID_I", 5),

        // @Param: VOLT_PID_D
        // @DisplayName: RPM LPF filter beta
        // @Description: This value is const of lpf filter for RPM
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, volt_pid_d, "VOLT_PID_D", 0),

        // @Param: VOLT_PID_D
        // @DisplayName: This is target voltage to hold
        // @Description: This value sets target voltage that VOLTAGE PID uses
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, target_voltage, "VOLT_TARGET", 30.0f),

        // @Param: VOLT_LPF_BETA
        // @DisplayName: Voltage LPF filter beta
        // @Description: This value is const of lpf filter for voltage
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, volt_lpf_beta, "VOLT_LPF_BETA", 0.4f),

        // @Param: PID_REPORT
        // @DisplayName: Which PID controller is reported to GCS
        // @Description: 1 - RPM, 0 - Voltage
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, pid_report, "PID_REPORT", 0),

        // @Param: MAX_MAN_THR
        // @DisplayName: Max manual throttle when in manual mode (%)
        // @Description: Max throttle (0.0-1.0)
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, max_man_thr, "MAX_MAN_THR", 0.05f),

        // @Param: RC1_MIN
        // @DisplayName: RC1 min PWM
        // @Description: Minimum RC1 PWM input
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, rc1_min, "RC1_MIN", 1100),

        // @Param: RC1_MAX
        // @DisplayName: RC1 max PWM
        // @Description: Maximum RC1 PWM input
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, rc1_max, "RC1_MAX", 1900),

        // @Param: RC1_REV
        // @DisplayName: RC1 reverse
        // @Description: Reverse channel 1-normal -1 reverse
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, rc1_rev, "RC1_REV", 1),

        // @Param: THR_MIN
        // @DisplayName: THR min PWM
        // @Description: Minimum THR PWM output
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, thr_min, "THR_MIN", 1100),

        // @Param: THR_MAX
        // @DisplayName: THR max PWM
        // @Description: Maximum THR PWM output
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, thr_max, "THR_MAX", 1900),

        // @Param: THR_REV
        // @DisplayName: THR reverse
        // @Description: Reverse channel 1-normal -1 reverse
        // @User: Advanced
        GSCALAR(AP_PARAM_INT16, thr_rev, "THR_REV", 1),

        // @Param: PID_D_FILT
        // @DisplayName:
        // @Description:
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, d_lpf_beta, "PID_D_FILT", 0.4f),

        // @Param: PID_D_FILT
        // @DisplayName:
        // @Description:
        // @User: Advanced
        GSCALAR(AP_PARAM_FLOAT, max_charge_current, "MAX_CHRG_CURR", 4.0f),

        GSCALAR(AP_PARAM_FLOAT, rpm_out_lpf_beta, "RPM_OUT_BETA", 0.2f),

        // @Param: LOAD_RMP_SPD
        // @DisplayName:
        // @Description: Amps/s
        GSCALAR(AP_PARAM_FLOAT, load_ramp_speed, "LOAD_RMP_SPD", 1.0f),


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

