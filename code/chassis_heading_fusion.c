#include "chassis_heading_fusion.h"

#include <float.h>
#include <math.h>

#define CHASSIS_HEADING_FUSION_PI (3.14159265358979323846f)

static bool fusion_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float fusion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float fusion_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float fusion_wrap(float angle_rad)
{
    while (angle_rad >= CHASSIS_HEADING_FUSION_PI) {
        angle_rad -= 2.0f * CHASSIS_HEADING_FUSION_PI;
    }
    while (angle_rad < -CHASSIS_HEADING_FUSION_PI) {
        angle_rad += 2.0f * CHASSIS_HEADING_FUSION_PI;
    }
    return angle_rad;
}

static bool fusion_config_valid(
    const chassis_heading_fusion_config_t *config)
{
    return (config != 0) &&
        fusion_float_valid(config->left_mm_per_tick) &&
        fusion_float_valid(config->right_mm_per_tick) &&
        fusion_float_valid(config->effective_track_mm) &&
        fusion_float_valid(config->heading_time_constant_s) &&
        fusion_float_valid(config->imu_rate_weight) &&
        fusion_float_valid(config->imu_heading_sign) &&
        fusion_float_valid(config->imu_max_delta_deg) &&
        (config->left_mm_per_tick > 0.0f) &&
        (config->right_mm_per_tick > 0.0f) &&
        (config->effective_track_mm > 0.0f) &&
        (config->heading_time_constant_s > 0.0f) &&
        (config->imu_rate_weight >= 0.0f) &&
        (config->imu_rate_weight <= 1.0f) &&
        ((config->imu_heading_sign == 1.0f) ||
         (config->imu_heading_sign == -1.0f)) &&
        (config->imu_max_delta_deg > 0.0f) &&
        (config->imu_stale_ms > 0U);
}

ml_status_t chassis_heading_fusion_init(
    chassis_heading_fusion_t *fusion,
    const chassis_heading_fusion_config_t *config)
{
    if ((fusion == 0) || !fusion_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    fusion->config = *config;
    fusion->initialized = true;
    return chassis_heading_fusion_reset(fusion, 0.0f);
}

ml_status_t chassis_heading_fusion_reset(
    chassis_heading_fusion_t *fusion, float heading_rad)
{
    if ((fusion == 0) || !fusion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!fusion_float_valid(heading_rad)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    fusion->encoder_heading_rad = heading_rad;
    fusion->fused_heading_rad = heading_rad;
    fusion->fused_yaw_rate_rad_s = 0.0f;
    fusion->latest_body_gyro_z_dps = 0.0f;
    fusion->latest_imu_yaw_deg = 0.0f;
    fusion->latest_imu_timestamp_ms = 0U;
    fusion->consumed_imu_timestamp_ms = 0U;
    fusion->imu_age_ms = fusion->config.imu_stale_ms;
    fusion->latest_imu_valid = false;
    fusion->imu_active = false;
    return ML_STATUS_OK;
}

void chassis_heading_fusion_set_imu(
    chassis_heading_fusion_t *fusion, float imu_yaw_deg,
    float body_gyro_z_dps, uint32_t timestamp_ms, bool valid)
{
    if ((fusion == 0) || !fusion->initialized) {
        return;
    }
    fusion->latest_imu_valid = valid &&
        fusion_float_valid(imu_yaw_deg) &&
        fusion_float_valid(body_gyro_z_dps);
    if (!fusion->latest_imu_valid) {
        return;
    }
    if (timestamp_ms != fusion->latest_imu_timestamp_ms) {
        fusion->imu_age_ms = 0U;
    }
    fusion->latest_imu_yaw_deg = imu_yaw_deg;
    fusion->latest_body_gyro_z_dps = body_gyro_z_dps;
    fusion->latest_imu_timestamp_ms = timestamp_ms;
}

ml_status_t chassis_heading_fusion_update(
    chassis_heading_fusion_t *fusion,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    uint8_t elapsed_cycles, uint16_t control_period_ms)
{
    float left_delta_mm;
    float right_delta_mm;
    float encoder_delta_rad;
    float encoder_rate_rad_s;
    float imu_rate_rad_s;
    float imu_delta_rad;
    float predicted_heading_rad;
    float heading_error_rad;
    float correction_gain;
    float elapsed_s;
    float maximum_imu_delta_rad;
    uint32_t elapsed_ms;
    bool new_imu_sample;
    bool imu_valid;

    if ((fusion == 0) || !fusion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((elapsed_cycles == 0U) || (control_period_ms == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    elapsed_ms = (uint32_t) elapsed_cycles * control_period_ms;
    elapsed_s = (float) elapsed_ms / 1000.0f;
    left_delta_mm = (float) left_delta_ticks *
        fusion->config.left_mm_per_tick;
    right_delta_mm = (float) right_delta_ticks *
        fusion->config.right_mm_per_tick;
    encoder_delta_rad = (right_delta_mm - left_delta_mm) /
        fusion->config.effective_track_mm;
    encoder_rate_rad_s = encoder_delta_rad / elapsed_s;
    fusion->encoder_heading_rad += encoder_delta_rad;

    new_imu_sample = fusion->latest_imu_valid &&
        (fusion->latest_imu_timestamp_ms !=
         fusion->consumed_imu_timestamp_ms);
    if (new_imu_sample) {
        fusion->consumed_imu_timestamp_ms =
            fusion->latest_imu_timestamp_ms;
        fusion->imu_age_ms = 0U;
    } else if (fusion->imu_age_ms < UINT16_MAX - elapsed_ms) {
        fusion->imu_age_ms = (uint16_t) (fusion->imu_age_ms + elapsed_ms);
    } else {
        fusion->imu_age_ms = UINT16_MAX;
    }

    imu_rate_rad_s = fusion->config.imu_heading_sign *
        fusion->latest_body_gyro_z_dps *
        CHASSIS_HEADING_FUSION_PI / 180.0f;
    imu_delta_rad = imu_rate_rad_s * elapsed_s;
    maximum_imu_delta_rad = fusion->config.imu_max_delta_deg *
        (float) elapsed_cycles * CHASSIS_HEADING_FUSION_PI / 180.0f;
    imu_valid = fusion->latest_imu_valid &&
        (fusion->imu_age_ms <= fusion->config.imu_stale_ms) &&
        fusion_float_valid(imu_rate_rad_s) &&
        (fusion_abs(imu_delta_rad) <= maximum_imu_delta_rad);

    if (!imu_valid) {
        fusion->fused_heading_rad += encoder_delta_rad;
        fusion->fused_yaw_rate_rad_s = encoder_rate_rad_s;
        fusion->imu_active = false;
        return ML_STATUS_OK;
    }

    predicted_heading_rad = fusion->fused_heading_rad + imu_delta_rad;
    heading_error_rad = fusion_wrap(
        fusion->encoder_heading_rad - predicted_heading_rad);
    correction_gain = fusion_clamp(
        elapsed_s / fusion->config.heading_time_constant_s,
        0.0f, 1.0f);
    fusion->fused_heading_rad = predicted_heading_rad +
        correction_gain * heading_error_rad;
    fusion->fused_yaw_rate_rad_s =
        fusion->config.imu_rate_weight * imu_rate_rad_s +
        (1.0f - fusion->config.imu_rate_weight) *
        encoder_rate_rad_s;
    fusion->imu_active = true;
    return ML_STATUS_OK;
}

float chassis_heading_fusion_encoder_heading(
    const chassis_heading_fusion_t *fusion)
{
    return ((fusion != 0) && fusion->initialized) ?
        fusion->encoder_heading_rad : 0.0f;
}

float chassis_heading_fusion_heading(
    const chassis_heading_fusion_t *fusion)
{
    return ((fusion != 0) && fusion->initialized) ?
        fusion->fused_heading_rad : 0.0f;
}

float chassis_heading_fusion_yaw_rate(
    const chassis_heading_fusion_t *fusion)
{
    return ((fusion != 0) && fusion->initialized) ?
        fusion->fused_yaw_rate_rad_s : 0.0f;
}

bool chassis_heading_fusion_active(
    const chassis_heading_fusion_t *fusion)
{
    return (fusion != 0) && fusion->initialized && fusion->imu_active;
}
