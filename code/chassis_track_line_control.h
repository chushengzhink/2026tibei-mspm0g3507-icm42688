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
    float outer_single_maximum_correction_mm_s;
    float outer_single_correction_ratio;
    float curve_hold_maximum_correction_mm_s;
    float curve_hold_correction_ratio;
    float curve_exit_minimum_travel_mm;
    float kp;
    float ki;
    float kd;
    float pid_output_limit;
    float pid_integral_limit;
    uint16_t control_period_ms;
    uint16_t outer_lost_full_boost_ms;
    uint16_t outer_lost_taper_ms;
    uint8_t reverse_confirm_cycles;
    uint8_t curve_exit_confirm_cycles;
    bool curve_memory_enabled;
} chassis_track_line_control_config_t;

typedef struct {
    chassis_track_line_control_config_t config;
    pid_t pid;
    float last_line_error;
    float pending_line_error;
    float curve_travel_mm;
    uint8_t last_valid_black_bits;
    int8_t accepted_side;
    int8_t pending_side;
    int8_t curve_memory_side;
    uint8_t pending_cycles;
    uint8_t curve_exit_cycles;
    uint16_t lost_ms;
    bool has_valid_error;
    bool initialized;
} chassis_track_line_control_t;

typedef struct {
    float linear_mm_s;
    float route_feedforward_rad_s;
    float heading_feedback_rad_s;
    float heading_error_deg;
} chassis_track_line_fusion_request_t;

typedef struct {
    float linear_mm_s;
    float angular_rad_s;
    float left_mm_s;
    float right_mm_s;
    float correction_mm_s;
    float route_feedforward_bias_mm_s;
    float heading_feedback_bias_mm_s;
    float line_weight;
    float final_steering_bias_mm_s;
    uint16_t lost_ms;
    chassis_track_line_state_t line_state;
    bool line_valid;
    bool recovering;
} chassis_track_line_control_output_t;

extern const chassis_track_line_control_config_t
    g_chassis_track_line_control_default_config;
extern const chassis_track_line_control_config_t
    g_chassis_track_line_control_line_only_config;

ml_status_t chassis_track_line_control_init(
    chassis_track_line_control_t *control,
    const chassis_track_line_control_config_t *config);
void chassis_track_line_control_reset(
    chassis_track_line_control_t *control);
ml_status_t chassis_track_line_control_update(
    chassis_track_line_control_t *control, const line_sample_t *sample,
    float requested_linear_mm_s, float requested_angular_rad_s,
    chassis_track_line_control_output_t *output);
ml_status_t chassis_track_line_control_update_fused(
    chassis_track_line_control_t *control, const line_sample_t *sample,
    const chassis_track_line_fusion_request_t *request,
    chassis_track_line_control_output_t *output);

#endif
