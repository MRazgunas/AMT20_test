#include "ch.h"
#include "hal.h"

#include "telemetry.h"
#include "mavlink_bridge.h" /* Has to be before mavlink.h */
#include "mavlink.h"
#include "chprintf.h"

#define SERIAL_DEVICE SD1

static virtual_timer_t led_vt; //Timer for rx
position_t last_pos;

void requset_gps_data_stream(void);
void handle_mavlink_message(mavlink_message_t msg);
void blink(void);

static void led_cb(void *arg) {
    (void) arg;
    palClearPad(GPIOC, GPIOC_LED2);
}

void blink() {
    palSetPad(GPIOC, GPIOC_LED2);
	chVTSet(&led_vt, MS2ST(50), led_cb, NULL);
}

/*
 * Mavlink receive
 */
static THD_WORKING_AREA(waMavlinkThread, 5000);
static THD_FUNCTION(MavlinkThread, arg) {

	(void) arg;
	static mavlink_message_t msg;
	static mavlink_status_t status;
	msg_t charData;

	event_listener_t serialData;
	eventflags_t flags;
	chEvtRegisterMask((event_source_t *) chnGetEventSource(&SERIAL_DEVICE),
			&serialData, EVENT_MASK(1));

	chRegSetThreadName("mavlink");
	while (true) {
		chEvtWaitOneTimeout(EVENT_MASK(1), TIME_INFINITE);
		flags = chEvtGetAndClearFlags(&serialData);
        do {
            charData = chnGetTimeout(&SERIAL_DEVICE, TIME_IMMEDIATE);
            if(charData > 0xFF || charData == Q_TIMEOUT) {
                break;
            }
            if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)charData, &msg, &status)) {
                handle_mavlink_message(msg);
                //blink(); //Blink led for received packet
            }
        } while(charData != Q_TIMEOUT);
        if(flags & SD_OVERRUN_ERROR) {
            blink();
        }
	}
	/* This point may be reached if shut down is requested. */
}

static THD_WORKING_AREA(waMavlinkTx, 1000);
static THD_FUNCTION(MavlinkTx, arg) {
    (void) arg;

    // Define the system type, in this case an airplane
    uint8_t system_type = MAV_TYPE_GIMBAL;
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t system_mode = MAV_MODE_AUTO_ARMED; ///< Booting up
    uint32_t custom_mode = 0;   ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight


    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type, system_mode, custom_mode, system_state);

	while(true) {

        // Pack the message
        mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type, system_mode, custom_mode, system_state);

        chThdSleepMilliseconds(1000);
	}
}

void handle_mavlink_message(mavlink_message_t msg) {
	switch (msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_heartbeat_t pack;
			mavlink_msg_heartbeat_decode(&msg, &pack);
			break;
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST1\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 0);
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST2\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 1);
			mavlink_msg_param_value_send(MAVLINK_COMM_0, "TEST3\0", 1.0f, MAV_PARAM_TYPE_UINT8, 1, 2);
			break;
		}
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
		    mavlink_global_position_int_t decode;
		    mavlink_msg_global_position_int_decode(&msg,  &decode);
		    last_pos.lat = decode.lat;
		    last_pos.lon = decode.lon;
		    last_pos.alt = decode.alt;
		    last_pos.r_alt = decode.relative_alt;
		    break;
		}
		case MAVLINK_MSG_ID_DATA_STREAM: {
		    mavlink_data_stream_t decode;
		    mavlink_msg_data_stream_decode(&msg, &decode);
		    break;
		}
	}
}

void init_telemetry() {
/*	mavlink_system.sysid = 3;
	mavlink_system.compid = MAV_COMP_ID_CAMERA; */

    palClearPad(GPIOC, GPIOC_LED2);
	chVTObjectInit(&led_vt);

	chThdCreateStatic(waMavlinkThread, sizeof(waMavlinkThread), NORMALPRIO + 2,
			MavlinkThread, NULL);
	chThdCreateStatic(waMavlinkTx, sizeof(waMavlinkTx), NORMALPRIO + 1,
	            MavlinkTx, NULL);
}

