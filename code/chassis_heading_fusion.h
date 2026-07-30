#ifndef CHASSIS_HEADING_FUSION_H
#define CHASSIS_HEADING_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

typedef struct {
    float left_mm_per_tick;
    float right_mm_per_tick;
    float effective_track_mm;
    float heading_time_constant_s;
    float imu_rate_weight;
    float imu_heading_sign;
    float imu_max_delta_deg;
    uint16_t imu_stale_ms;
} chassis_heading_fusion_config_t;

typedef struct {
    chassis_heading_fusion_config_t config;
    float encoder_heading_rad;
    float fused_heading_rad;
    float fused_yaw_rate_rad_s;
    float latest_body_gyro_z_dps;
    float latest_imu_yaw_deg;
    uint32_t latest_imu_timestamp_ms;
    uint32_t consumed_imu_timestamp_ms;
    uint16_t imu_age_ms;
    bool latest_imu_valid;
    bool imu_active;
    bool initialized;
} chassis_heading_fusion_t;

ml_status_t chassis_heading_fusion_init(
    chassis_heading_fusion_t *fusion,
    const chassis_heading_fusion_config_t *config);
ml_status_t chassis_heading_fusion_reset(
    chassis_heading_fusion_t *fusion, float heading_rad);
void chassis_heading_fusion_set_imu(
    chassis_heading_fusion_t *fusion, float imu_yaw_deg,
    float body_gyro_z_dps, uint32_t timestamp_ms, bool valid);
ml_status_t chassis_heading_fusion_update(
    chassis_heading_fusion_t *fusion,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    uint8_t elapsed_cycles, uint16_t control_period_ms);
float chassis_heading_fusion_encoder_heading(
    const chassis_heading_fusion_t *fusion);
float chassis_heading_fusion_heading(
    const chassis_heading_fusion_t *fusion);
float chassis_heading_fusion_yaw_rate(
    const chassis_heading_fusion_t *fusion);
bool chassis_heading_fusion_active(
    const chassis_heading_fusion_t *fusion);

#endif
