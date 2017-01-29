#ifndef SRC_PID_RPM_H_
#define SRC_PID_RPM_H_

#include "ch.h"
#include "hal.h"

void reset_integrator(void);
float apply_rpm_pid(uint16_t target_rpm, uint16_t rpm);

extern float p_term;
extern float i_term;
extern float d_term;
extern float d_term_lpf;

#endif /* SRC_PID_RPM_H_ */
