#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

#define LINE_SENSOR_WHITE_LEVELS_EXPECTED (0x0FU)
#define LINE_SENSOR_WHITE_STABLE_SAMPLES  (10U)

#define LINE_SENSOR_LEFT_GROUP_MASK  (0x03U)
#define LINE_SENSOR_RIGHT_GROUP_MASK (0x0CU)

typedef struct {
    uint8_t raw_bits;
    uint8_t black_bits;
    bool left_on;
    bool right_on;
    bool lost;
    bool io_fault;
} line_sample_t;

typedef struct {
    uint8_t consecutive_samples;
} line_sensor_white_guard_t;

ml_status_t line_sensor_init(void);
ml_status_t line_sensor_reassert_inputs(void);
ml_status_t line_sensor_calibrate_white(uint16_t samples, uint16_t delay_ms_each);
line_sample_t line_sensor_read(void);
uint8_t line_sensor_white_levels(void);
void line_sensor_white_guard_reset(line_sensor_white_guard_t *guard);
bool line_sensor_white_guard_update(
    line_sensor_white_guard_t *guard, uint8_t raw_bits);

#endif
