#ifndef CHASSIS_TRACK_MISSION_H
#define CHASSIS_TRACK_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

typedef enum {
    CHASSIS_TRACK_READY = 0,
    CHASSIS_TRACK_AB,
    CHASSIS_TRACK_BC,
    CHASSIS_TRACK_CD,
    CHASSIS_TRACK_DA,
    CHASSIS_TRACK_FINAL_APPROACH,
    CHASSIS_TRACK_FINISH_CHECK,
    CHASSIS_TRACK_BRAKING,
    CHASSIS_TRACK_ALIGNING,
    CHASSIS_TRACK_COMPLETE,
    CHASSIS_TRACK_FAULT_LAP_CHECK,
    CHASSIS_TRACK_FAULT_ALIGNMENT,
    CHASSIS_TRACK_FAULT_EMERGENCY
} chassis_track_state_t;

typedef struct {
    float straight_length_mm;
    float curve_radius_mm;
    float straight_cruise_speed_mm_s;
    float curve_cruise_speed_mm_s;
    float approach_speed_mm_s;
    float acceleration_mm_s2;
    float approach_distance_mm;
    float finish_reference_progress_mm;
    float finish_max_overrun_mm;
    float finish_stop_lead_mm;
    float finish_heading_target_deg;
    float finish_heading_tolerance_deg;
    float finish_alignment_tolerance_deg;
    float finish_alignment_heading_bias_deg;
    float finish_alignment_max_start_error_deg;
    float heading_control_kp;
    float maximum_heading_correction_rad_s;
    float stop_speed_mm_s;
    float pass_time_s;
    float pass_error_mm;
    uint16_t control_period_ms;
    uint16_t finish_alignment_timeout_ms;
    uint8_t finish_heading_confirm_cycles;
    uint8_t finish_alignment_confirm_cycles;
    uint8_t stopped_cycles_required;
} chassis_track_config_t;

typedef struct {
    chassis_track_config_t config;
    chassis_track_state_t state;
    float start_distance_mm;
    float start_heading_deg;
    float progress_mm;
    float commanded_speed_mm_s;
    float stop_error_mm;
    float elapsed_s;
    float heading_progress_deg;
    float expected_heading_deg;
    float route_feedforward_rad_s;
    float heading_feedback_rad_s;
    float heading_error_deg;
    uint32_t start_time_ms;
    uint32_t stop_time_ms;
    uint32_t alignment_start_time_ms;
    uint8_t heading_window_cycles;
    uint8_t alignment_confirm_cycles;
    uint8_t stopped_cycles;
    bool distance_gate_met;
    bool heading_gate_met;
    bool initialized;
} chassis_track_mission_t;

typedef struct {
    float linear_mm_s;
    float angular_rad_s;
    float progress_mm;
    float elapsed_s;
    float stop_error_mm;
    float heading_progress_deg;
    float expected_heading_deg;
    float route_feedforward_rad_s;
    float heading_feedback_rad_s;
    float heading_error_deg;
    chassis_track_state_t state;
    bool command_stop;
    bool finished;
    bool passed;
    bool distance_gate_met;
    bool heading_gate_met;
} chassis_track_output_t;

extern const chassis_track_config_t g_chassis_track_default_config;

float chassis_track_route_length(const chassis_track_config_t *config);
ml_status_t chassis_track_mission_init(chassis_track_mission_t *mission,
    const chassis_track_config_t *config);
ml_status_t chassis_track_mission_start(chassis_track_mission_t *mission,
    float center_distance_mm, float fused_heading_deg, uint32_t now_ms);
ml_status_t chassis_track_mission_update(chassis_track_mission_t *mission,
    float center_distance_mm, float measured_left_mm_s,
    float measured_right_mm_s, float fused_heading_deg,
    uint32_t now_ms, bool emergency_stop,
    chassis_track_output_t *output);
const char *chassis_track_state_text(chassis_track_state_t state);

#endif
