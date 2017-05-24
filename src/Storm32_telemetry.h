/*
 * Storm32_telemetry.h
 *
 *  Created on: 2017 kov. 11
 *      Author: matas
 */

#ifndef SRC_STORM32_TELEMETRY_H_
#define SRC_STORM32_TELEMETRY_H_

#define PACKED __attribute__((__packed__))

#define CMD_ACK             0x96
#define CMD_GETDATA         0x05
#define CMD_GETDATAFIELDS   0x06

struct PACKED cmd_get_data_fields {
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint16_t data_type;
    uint16_t crc;
};

struct PACKED cmd_get_data_fields_ack_imu {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
};

struct PACKED cmd_set_angles_struct {
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    float pitch;
    float roll;
    float yaw;
    uint8_t flags;
    uint8_t type;
    uint16_t crc;
};

struct PACKED cmd_get_data_struct {
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t type; //Currenlty 0
    uint16_t crc;
};

struct PACKED SToRM32_reply_data_struct {
    uint16_t state;
    uint16_t status;
    uint16_t status2;

    uint16_t i2c_errors;
    uint16_t lipo_voltage;
    uint16_t systicks;
    uint16_t cycle_time;

    int16_t imu1_gx;
    int16_t imu1_gy;
    int16_t imu1_gz;

    int16_t imu1_ax;
    int16_t imu1_ay;
    int16_t imu1_az;

    int16_t ahrs_x;
    int16_t ahrs_y;
    int16_t ahrs_z;

    int16_t imu1_pitch;
    int16_t imu1_roll;
    int16_t imu1_yaw;

    int16_t cpid_pitch;
    int16_t cpid_roll;
    int16_t cpid_yaw;

    uint16_t input_pitch;
    uint16_t input_roll;
    uint16_t input_yaw;

    int16_t imu2_pitch;
    int16_t imu2_roll;
    int16_t imu2_yaw;

    int16_t mag2_yaw;
    int16_t mag2_pitch;

    int16_t ahrs_imu_confidence;

    uint16_t function_input_values;

    uint16_t crc;
    uint8_t magic;
};

#define AP_MOUNT_STORM32_SERIAL_RESEND_MS   1000    // resend angle targets to gimbal once per second

void set_target_angles(float pitch, float roll, float yaw, bool force_update);
void init_sotmr32_telemetry(void);
struct SToRM32_reply_data_struct * get_storm_state(void);

void update_storm32(void);

#endif /* SRC_STORM32_TELEMETRY_H_ */
