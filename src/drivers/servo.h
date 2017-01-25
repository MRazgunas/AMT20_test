#ifndef SRC_DRIVERS_SERVO_H_
#define SRC_DRIVERS_SERVO_H_

void set_thr_pwm(uint16_t pulse_width);
void set_thr_position(float pos); /* Position from 0.0 to 1.0 */

uint16_t get_thr_pwm(void);

void init_servo(void);


#endif /* SRC_DRIVERS_SERVO_H_ */
