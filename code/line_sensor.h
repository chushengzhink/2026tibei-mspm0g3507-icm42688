#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

typedef struct {
    uint8_t raw_bits;
    uint8_t black_bits;
    uint8_t black_count;
    float error_mm;
    bool lost;
    bool transverse;
    bool centered;
} line_sample_t;

ml_status_t line_sensor_init(void);
ml_status_t line_sensor_calibrate_white(uint16_t samples, uint16_t delay_ms_each);
line_sample_t line_sensor_read(void);
uint8_t line_sensor_white_levels(void);

#endif
