#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_

typedef struct position_t {
    int32_t lat; /* Latitude, expressed as degrees * 1E7 */
    int32_t lon; /* Longitude, expressed as degrees * 1E7 */
    int32_t alt; /* Altitude in meters, expressed as * 1000 (millimeters), AMSL */
    int32_t r_alt;
} position_t;

void init_telemetry(void);

extern position_t last_pos;

#endif /* SRC_TELEMETRY_H_ */
