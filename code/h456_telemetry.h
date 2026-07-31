#ifndef H456_TELEMETRY_H
#define H456_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "h456_mission.h"
#include "ml_common.h"

#define H456_TELEMETRY_CAPACITY       (600U)
#define H456_TELEMETRY_RECORD_BYTES   (44U)
#define H456_TELEMETRY_H4_PERIOD_MS   (20U)
#define H456_TELEMETRY_LAP_PERIOD_MS  (50U)

typedef struct {
    uint32_t timestamp_ms;
    h456_mode_t mode;
    h456_mission_state_t mission_state;
    float progress_mm;
    float fused_heading_deg;
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
    int16_t raw_y_px;
    uint32_t vision_age_ms;
    uint32_t frame_interval_ms;
    bool vision_ready;
    bool ball_enabled;
    bool ball_violation;
    bool breakaway_fault;
    bool score_point_passed;
} h456_telemetry_sample_t;

ml_status_t h456_telemetry_init(void);
void h456_telemetry_session_start(
    h456_mode_t mode, uint32_t start_time_ms);
ml_status_t h456_telemetry_record(
    const h456_telemetry_sample_t *sample, bool force);
void h456_telemetry_session_finish(
    const h456_telemetry_sample_t *sample);
void h456_telemetry_set_result(uint32_t score_elapsed_ms,
    float maximum_ball_error_cm, bool ball_score_passed);
bool h456_telemetry_session_active(void);
uint16_t h456_telemetry_count(void);
bool h456_telemetry_full(void);
uint32_t h456_telemetry_storage_bytes(void);
ml_status_t h456_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms);

#endif
