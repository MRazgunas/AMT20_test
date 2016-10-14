#ifndef SRC_PID_RPM_H_
#define SRC_PID_RPM_H_

#include "ch.h"
#include "hal.h"

void reset_integrator(void);
float apply_rpm_pid(uint16_t target_rpm, uint16_t rpm);

#endif /* SRC_PID_RPM_H_ */
