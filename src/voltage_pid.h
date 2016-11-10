#ifndef SRC_VOLTAGE_PID_H_
#define SRC_VOLTAGE_PID_H_

extern float volt_p_term;
extern float volt_i_term;
extern float volt_d_term;


uint16_t apply_voltage_pid(float target_voltage, float voltage, float thr);
void reset_volt_integrator(void);

#endif /* SRC_VOLTAGE_PID_H_ */
