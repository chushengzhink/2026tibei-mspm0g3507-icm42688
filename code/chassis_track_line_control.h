#ifndef CHASSIS_TRACK_LINE_CONTROL_H
#define CHASSIS_TRACK_LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "line_sensor.h"
#include "pid.h"

typedef enum {
    CHASSIS_TRACK_LINE_LOST = 0,
    CHASSIS_TRACK_LINE_CENTERED,
    CHASSIS_TRACK_LINE_LEFT,
    CHASSIS_TRACK_LINE_RIGHT
} chassis_track_line_state_t;

typedef struct {
    float effective_track_mm;
    float maximum_wheel_speed_mm_s;
    float maximum_correction_mm_s;
    float correction_ratio;
    float recovery_speed_mm_s;
    float kp;
    float ki;
    float kd;
    float pid_output_limit;
    float pid_integral_limit;
    uint16_t control_period_ms;
    uint16_t lost_timeout_ms;
} chassis_track_line_control_config_t;

typedef struct {
    chassis_track_line_control_config_t config;
    pid_t pid;
    float last_pid_output;
    chassis_track_line_state_t previous_state;
    uint16_t lost_ms;
    bool lost_fault;
    bool initialized;
} chassis_track_line_control_t;

typedef struct {
    float linear_mm_s;
    float angular_rad_s;
    float left_mm_s;
    float right_mm_s;
    float correction_mm_s;
    uint16_t lost_ms;
    chassis_track_line_state_t line_state;
    bool line_valid;
    bool recovering;
    bool lost_fault;
} chassis_track_line_control_output_t;

extern const chassis_track_line_control_config_t
    g_chassis_track_line_control_default_config;

ml_status_t chassis_track_line_control_init(
    chassis_track_line_control_t *control,
    const chassis_track_line_control_config_t *config);
void chassis_track_line_control_reset(
    chassis_track_line_control_t *control);
ml_status_t chassis_track_line_control_update(
    chassis_track_line_control_t *control, const line_sample_t *sample,
    float requested_linear_mm_s, float requested_angular_rad_s,
    chassis_track_line_control_output_t *output);

#endif
