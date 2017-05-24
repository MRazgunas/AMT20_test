#include "ch.h"
#include "hal.h"

#include "Storm32_telemetry.h"
#include "checksum.h"

struct storm32_message {
    uint8_t lenght;
    uint8_t id;
    uint8_t payload[80];
    uint16_t crc;
};

uint8_t get_reply_size(uint8_t reply_type);
void parse_reply(void);
void get_data(void);
void read_incoming(void);
bool parse_storm_message(uint8_t c, struct storm32_message * msg);
void handle_storm_message(struct storm32_message msg);

#define bkpt() __asm volatile("BKPT #0\n")

typedef struct _orient {
    float pitch;
    float roll;
    float yaw;
} Vector3d;


static systime_t last_send = 0;
static struct SToRM32_reply_data_struct current_storm32_state;
Vector3d current_angle = {0.0f, 0.0f, 0.0f};
Vector3d cmd_angle = {0.0f, 0.0f, 0.0f};
static bool resend_now = false;

struct SToRM32_reply_data_struct * get_storm_state(void) {
    return &current_storm32_state;
}

enum {
    WAITING_FOR_START,
    RECEIVING_LENGHT,
    RECEIVING_MSG_ID,
    RECEIVING_PAYLOAD,
    RECEIVING_CRC1,
    RECEIVING_CRC2,
    MESSAGE_RECEIVED,
};

bool parse_storm_message(uint8_t c, struct storm32_message * msg) {
    static uint8_t state = WAITING_FOR_START;
    static uint8_t payload_counter = 0;
    static uint16_t crc = 0;

    switch(state) {
    case WAITING_FOR_START:
        if(c == 0xFB) {
            state = RECEIVING_LENGHT;
            payload_counter = 0;
            crc_init(&crc);
        }
        break;
    case RECEIVING_LENGHT:
        msg->lenght = c;
        if(c > 80) {
            state = WAITING_FOR_START;
            break;
        }
        state = RECEIVING_MSG_ID;
        crc_accumulate(c, &crc);
        break;
    case RECEIVING_MSG_ID:
        msg->id = c;
        crc_accumulate(c, &crc);
        if(msg->lenght == 0) {
            state = RECEIVING_CRC1;
        } else {
            state = RECEIVING_PAYLOAD;
        }
        break;
    case RECEIVING_PAYLOAD:
        msg->payload[payload_counter++] = c;
        crc_accumulate(c, &crc);
        if(payload_counter == msg->lenght) {
            state = RECEIVING_CRC1;
        }
        break;
    case RECEIVING_CRC1:
        msg->crc = (msg->crc & 0xff00) | c;
        state = RECEIVING_CRC2;
        break;
    case RECEIVING_CRC2:
        msg->crc = (msg->crc & 0x00FF) | (c << 8);
        state = WAITING_FOR_START;
        uint16_t cr1 = crc;
        if(cr1 == msg->crc)
            return true;
        else
            return false;
    }
    return false;
}


void read_incoming(void) {
    uint8_t buffer[80];
    static struct storm32_message msg;
    uint8_t cnt = chnReadTimeout(&SD3, buffer, 80, TIME_IMMEDIATE);
    for(uint8_t i = 0; i < cnt; i++) {
        if(parse_storm_message(buffer[i], &msg)) {
            handle_storm_message(msg);
        }
    }
}

void handle_storm_message(struct storm32_message msg) {
    switch(msg.id) {
        case CMD_GETDATA: {
            struct SToRM32_reply_data_struct *data;
            data = &msg.payload[2];
            current_angle.pitch = data->imu1_pitch;
            current_angle.roll = data->imu1_roll;
            current_angle.yaw = data->imu1_yaw;
            current_storm32_state = *data;
            break;
        }
        case CMD_GETDATAFIELDS: {
            if(msg.payload[0] == 0x20) {
                struct cmd_get_data_fields_ack_imu *data;
                data = &msg.payload[2];
                current_storm32_state.imu1_pitch = data->pitch;
                current_storm32_state.imu1_roll = data->roll;
                current_storm32_state.imu1_yaw = data->yaw;
            } else if(msg.payload[0] == 0x01) {
                current_storm32_state.state = msg.payload[2];
            }
        }
    }
}

void set_target_angles(float pitch, float roll, float yaw, bool force_update) {
    cmd_angle.pitch = pitch;
    cmd_angle.roll = roll;
    cmd_angle.yaw = yaw;
    resend_now = force_update;
}

void send_angles(void) {
    struct cmd_set_angles_struct send = {
        0xFA,
        0x0E,
        0x11,
        0, // pitch
        0, // roll
        0, // yaw
        0, // flags
        0, // type
        0, // crc
    };

    send.pitch = cmd_angle.pitch;
    send.roll = cmd_angle.roll;
    send.yaw = cmd_angle.yaw;

    uint8_t* buf = (uint8_t*)&send;

    send.crc = crc_calculate(&buf[1], sizeof(struct cmd_set_angles_struct)-3);

    chnWrite(&SD3, buf, sizeof(struct cmd_set_angles_struct));

    last_send = ST2MS(chVTGetSystemTime());
}

void get_data(void) {
  /*  struct cmd_get_data_struct msg = {
        0xFA,
        0x01,
        CMD_GETDATA,
        0,
        0,
    };*/
    struct cmd_get_data_fields msg = {
        0xFA,
        0x02,
        0x06,
        0x0020,
        0,
    };
    uint8_t *buf = (uint8_t*)&msg;
    msg.crc = crc_calculate(&buf[1], sizeof(struct cmd_get_data_fields) - 3);
    chnWrite(&SD3, buf, sizeof(struct cmd_get_data_fields));
}

void get_state(void) {
    struct cmd_get_data_fields msg = {
        0xFA,
        0x02,
        0x06,
        0x0001,
        0,
    };
    uint8_t *buf = (uint8_t*)&msg;
    msg.crc = crc_calculate(&buf[1], sizeof(struct cmd_get_data_fields) - 3);
    chnWrite(&SD3, buf, sizeof(struct cmd_get_data_fields));
}

void update_storm32(void) {
    read_incoming();

    // resend target angles at least once per second
    resend_now = resend_now || ((ST2MS(chVTGetSystemTime()) - last_send) > AP_MOUNT_STORM32_SERIAL_RESEND_MS);


    if(resend_now) {
        send_angles();
        get_data();
        get_state();
        resend_now = false;
    } else {
        get_data();
        get_state();
    }
}

void init_sotmr32_telemetry(void) {
//    chThdCreateStatic(waStormThread, sizeof(waStormThread), NORMALPRIO,
//            StormThread, NULL);
}
