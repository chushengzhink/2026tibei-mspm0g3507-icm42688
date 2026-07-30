#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "line_sensor.h"
#include "ml_common.h"
#include "robot_config.h"

typedef enum {
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_FIND_A,
    MOTION_MODE_STRAIGHT,
    MOTION_MODE_LINE,
    MOTION_MODE_LINE_TO_END,
    MOTION_MODE_TURN,
    MOTION_MODE_CIRCLE
} motion_mode_t;

typedef enum {
    MOTION_RESULT_IDLE = 0,
    MOTION_RESULT_RUNNING,
    MOTION_RESULT_COMPLETE,
    MOTION_RESULT_FAULT,
    MOTION_RESULT_CANCELLED
} motion_result_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_ENCODER,
    MOTION_FAULT_ENCODER_STALL,
    MOTION_FAULT_LINE_LOST,
    MOTION_FAULT_LINE_NOT_FOUND,
    MOTION_FAULT_CIRCLE_SYNC
} motion_fault_t;

typedef struct {
    motion_mode_t mode;
    motion_result_t result;
    motion_fault_t fault;
    float left_distance_mm;
    float right_distance_mm;
    float mean_distance_mm;
    float target_distance_mm;
    line_sample_t line;
    uint32_t uptime_ticks;
} motion_status_t;

ml_status_t motion_init(const robot_calibration_t *calibration);
ml_status_t motion_start_find_a(float speed_mm_s);
ml_status_t motion_start_straight(float distance_mm, float speed_mm_s);
ml_status_t motion_start_line(float distance_mm, float speed_mm_s);
ml_status_t motion_start_line_to_end(float expected_distance_mm,
    float speed_mm_s);
ml_status_t motion_start_turn90(bool turn_right, bool align_to_line);
ml_status_t motion_start_circle(uint16_t radius_mm);
void motion_stop(void);
void motion_emergency_stop(void);
motion_status_t motion_get_status(void);
uint32_t motion_get_uptime_ticks(void);

#endif
