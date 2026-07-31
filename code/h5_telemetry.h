#ifndef H5_TELEMETRY_H
#define H5_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "h5_mission.h"
#include "ml_common.h"

#define H5_TELEMETRY_CAPACITY       (600U)
#define H5_TELEMETRY_RECORD_BYTES   (44U)
#define H5_TELEMETRY_H5_PERIOD_MS   (50U)
#define H5_TELEMETRY_LAP_PERIOD_MS  (50U)

typedef struct {
    uint16_t timestamp_ms;
    h5_mode_t mode;
    h5_mission_state_t mission_state;
    float progress_mm;
    float fused_heading_deg;
    float expected_heading_deg;
    float heading_error_deg;
    bool heading_gate_met;
    float target_center_mm_s;
    float actual_center_mm_s;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    uint8_t line_bits;
    bool line_usable;
    bool line_recovering;
    float line_correction_mm_s;
    float final_steering_bias_mm_s;
    float ball_target_cm;
    float ball_position_cm;
    float ball_error_min_cm;
    float ball_error_max_cm;
    float ball_velocity_cm_s;
    float ball_control_output_us;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    int16_t raw_x_px;
    uint8_t vision_age_ms;
    uint8_t frame_interval_ms;
    bool vision_ready;
    bool ball_enabled;
    bool ball_violation;
    bool breakaway_fault;
    bool score_point_passed;
} h5_telemetry_sample_t;

ml_status_t h5_telemetry_init(void);
void h5_telemetry_session_start(
    h5_mode_t mode, uint32_t start_time_ms);
ml_status_t h5_telemetry_record(
    const h5_telemetry_sample_t *sample, bool force);
void h5_telemetry_session_finish(
    const h5_telemetry_sample_t *sample);
void h5_telemetry_set_result(uint32_t score_elapsed_ms,
    float maximum_ball_error_cm, bool ball_score_passed);
bool h5_telemetry_session_active(void);
uint16_t h5_telemetry_count(void);
bool h5_telemetry_full(void);
uint32_t h5_telemetry_storage_bytes(void);
ml_status_t h5_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms);

#endif
