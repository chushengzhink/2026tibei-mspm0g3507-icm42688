#ifndef CHASSIS_TELEMETRY_H
#define CHASSIS_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

#ifndef CHASSIS_TELEMETRY_CAPACITY
#define CHASSIS_TELEMETRY_CAPACITY (600U)
#endif

#define CHASSIS_TELEMETRY_LINE_USABLE          (0x01U)
#define CHASSIS_TELEMETRY_LINE_RECOVERING      (0x02U)
#define CHASSIS_TELEMETRY_LINE_PATTERN_INVALID (0x04U)

typedef struct {
    uint32_t timestamp_ms;
    int32_t left_total_ticks;
    int32_t right_total_ticks;
    float x_mm;
    float y_mm;
    int32_t encoder_heading_cdeg;
    int32_t fused_heading_cdeg;
    int16_t imu_yaw_cdeg;
    int16_t fused_yaw_rate_cdps;
    int16_t target_center_dmm_s;
    int16_t actual_center_dmm_s;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    uint8_t fusion_active;
    uint8_t line_bits;
    uint8_t line_state_flags;
    int8_t line_correction_mm_s;
    uint16_t mission_progress_mm;
    uint16_t expected_heading_cdeg;
    int8_t route_feedforward_bias_mm_s;
    int8_t heading_feedback_bias_mm_s;
    uint8_t line_weight_pct;
    int8_t final_steering_bias_mm_s;
} chassis_telemetry_record_t;

typedef ml_status_t (*chassis_telemetry_writer_t)(
    const char *data, uint16_t length, void *context);

void chassis_telemetry_init(void);
void chassis_telemetry_clear(void);
void chassis_telemetry_session_start(uint32_t start_time_ms);
void chassis_telemetry_session_finish(uint32_t stop_time_ms);
void chassis_telemetry_set_line_bits(uint8_t line_bits);
void chassis_telemetry_set_line_state(uint8_t line_bits,
    bool line_usable, bool line_recovering, bool pattern_invalid);
void chassis_telemetry_set_line_correction(float correction_mm_s);
void chassis_telemetry_set_track_fusion(float mission_progress_mm,
    float expected_heading_deg, float route_feedforward_bias_mm_s,
    float heading_feedback_bias_mm_s, float line_weight,
    float final_steering_bias_mm_s);
ml_status_t chassis_telemetry_record(uint32_t timestamp_ms,
    int32_t left_total_ticks, int32_t right_total_ticks,
    float x_mm, float y_mm, float encoder_heading_deg,
    float fused_heading_deg, float imu_yaw_deg,
    float fused_yaw_rate_dps, float target_center_mm_s,
    float actual_center_mm_s, uint16_t pwm_left_count,
    uint16_t pwm_right_count, bool fusion_active);
uint16_t chassis_telemetry_count(void);
bool chassis_telemetry_overflowed(void);
ml_status_t chassis_telemetry_get(
    uint16_t index, chassis_telemetry_record_t *record);
ml_status_t chassis_telemetry_export_csv(
    chassis_telemetry_writer_t writer, void *context);

#endif
