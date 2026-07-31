#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "ml_common.h"

#define LINE_SENSOR_WHITE_STABLE_SAMPLES (10U)

typedef struct {
    uint8_t raw_bits;
    uint8_t black_bits;
    bool lost;
    bool io_fault;
} line_sample_t;

typedef struct { uint8_t consecutive_samples; }
    line_sensor_white_guard_t;

ml_status_t line_sensor_init(void);
ml_status_t line_sensor_reassert_inputs(void);
line_sample_t line_sensor_read(void);
void line_sensor_white_guard_reset(line_sensor_white_guard_t *guard);
bool line_sensor_white_guard_update(
    line_sensor_white_guard_t *guard, uint8_t raw_bits);

#endif
