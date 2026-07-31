#ifndef H456_MISSION_H
#define H456_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

typedef enum {
    H456_MODE_4 = 4,
    H456_MODE_5 = 5,
    H456_MODE_6 = 6
} h456_mode_t;

typedef enum {
    H456_MISSION_READY = 0,
    H456_MISSION_RUNNING,
    H456_MISSION_BRAKING,
    H456_MISSION_COMPLETE,
    H456_MISSION_FAULT_LAP_GATE,
    H456_MISSION_FAULT_EMERGENCY
} h456_mission_state_t;

typedef enum {
    H456_MISSION_RESULT_PENDING = 0,
    H456_MISSION_RESULT_PASS,
    H456_MISSION_RESULT_TIME_LIMIT,
    H456_MISSION_RESULT_FAULT
} h456_mission_result_t;

typedef struct {
    float straight_length_mm;
    float curve_radius_mm;
    float cruise_speed_mm_s;
    float h4_cruise_speed_mm_s;
    float acceleration_mm_s2;
    float h4_launch_acceleration_mm_s2;
    float lap_pass_progress_mm;
    float lap_max_overrun_mm;
    float finish_heading_target_deg;
    float finish_heading_tolerance_deg;
    float heading_control_kp;
    float maximum_heading_correction_rad_s;
    float stop_speed_mm_s;
    uint32_t h4_launch_acceleration_ms;
    uint32_t h4_time_limit_ms;
    uint32_t lap_time_limit_ms;
    uint16_t control_period_ms;
    uint8_t finish_heading_confirm_cycles;
    uint8_t stopped_cycles_required;
} h456_mission_config_t;

typedef struct {
    h456_mission_config_t config;
    h456_mode_t mode;
    h456_mission_state_t state;
    float start_distance_mm;
    float start_heading_deg;
    float progress_mm;
    float commanded_speed_mm_s;
    float expected_heading_deg;
    float heading_progress_deg;
    float heading_error_deg;
    uint32_t start_time_ms;
    uint32_t score_elapsed_ms;
    uint8_t heading_window_cycles;
    uint8_t stopped_cycles;
    bool score_point_passed;
    bool time_limit_failed;
    bool heading_gate_met;
    bool initialized;
} h456_mission_t;

typedef struct {
    h456_mode_t mode;
    h456_mission_state_t state;
    h456_mission_result_t result;
    float linear_mm_s;
    float route_feedforward_rad_s;
    float heading_feedback_rad_s;
    float progress_mm;
    float expected_heading_deg;
    float heading_progress_deg;
    float heading_error_deg;
    uint32_t elapsed_ms;
    uint32_t score_elapsed_ms;
    bool score_point_passed;
    bool heading_gate_met;
    bool command_stop;
    bool finished;
} h456_mission_output_t;

extern const h456_mission_config_t g_h456_mission_default_config;

ml_status_t h456_mission_init(h456_mission_t *mission,
    const h456_mission_config_t *config);
ml_status_t h456_mission_start(h456_mission_t *mission,
    h456_mode_t mode, float center_distance_mm,
    float fused_heading_deg, uint32_t now_ms);
ml_status_t h456_mission_update(h456_mission_t *mission,
    float center_distance_mm, float measured_left_mm_s,
    float measured_right_mm_s, float fused_heading_deg,
    uint32_t now_ms, bool emergency_stop,
    h456_mission_output_t *output);
const char *h456_mission_state_text(h456_mission_state_t state);

#endif
