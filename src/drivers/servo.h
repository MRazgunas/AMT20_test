/*
 * servo.h
 *
 *  Created on: 2016-09-23
 *      Author: matas
 */

#ifndef SRC_DRIVERS_SERVO_H_
#define SRC_DRIVERS_SERVO_H_

void set_servo_pwm(uint16_t pulse_width);
void set_servo_position(float pos); /* Position from -1.0 to 1.0 */

void init_servo(void);


#endif /* SRC_DRIVERS_SERVO_H_ */
