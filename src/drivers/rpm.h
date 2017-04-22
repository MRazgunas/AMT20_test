#ifndef SRC_DRIVERS_RPM_H_
#define SRC_DRIVERS_RPM_H_

extern uint16_t last_rpm;

void init_rpm(void);

uint16_t get_rpm(void);
uint16_t no_filt_rpm(void);
void apply_filter(void);


#endif /* SRC_DRIVERS_RPM_H_ */
