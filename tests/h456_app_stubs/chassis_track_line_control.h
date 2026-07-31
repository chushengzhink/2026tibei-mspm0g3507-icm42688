#ifndef CHASSIS_TRACK_LINE_CONTROL_H
#define CHASSIS_TRACK_LINE_CONTROL_H

#include "line_sensor.h"

typedef struct {
    float effective_track_mm;
    float maximum_wheel_speed_mm_s;
    float maximum_correction_mm_s;
    float correction_ratio;
    float outer_single_maximum_correction_mm_s;
    float outer_single_correction_ratio;
    float curve_hold_maximum_correction_mm_s;
    float curve_hold_correction_ratio;
    uint16_t control_period_ms;
} chassis_track_line_control_config_t;

typedef struct { bool initialized; }
    chassis_track_line_control_t;

typedef struct {
    float linear_mm_s;
    float route_feedforward_rad_s;
    float heading_feedback_rad_s;
    float heading_error_deg;
    bool heading_only;
} chassis_track_line_fusion_request_t;

typedef struct {
    float linear_mm_s;
    float angular_rad_s;
    float correction_mm_s;
    float final_steering_bias_mm_s;
    bool line_valid;
    bool recovering;
} chassis_track_line_control_output_t;

extern const chassis_track_line_control_config_t
    g_chassis_track_line_control_default_config;

ml_status_t chassis_track_line_control_init(
    chassis_track_line_control_t *control,
    const chassis_track_line_control_config_t *config);
void chassis_track_line_control_reset(
    chassis_track_line_control_t *control);
ml_status_t chassis_track_line_control_update_fused(
    chassis_track_line_control_t *control,
    const line_sample_t *sample,
    const chassis_track_line_fusion_request_t *request,
    chassis_track_line_control_output_t *output);

#endif
