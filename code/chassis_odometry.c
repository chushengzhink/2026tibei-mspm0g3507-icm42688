#include "chassis_odometry.h"

#include <float.h>
#include <math.h>

#define CHASSIS_ODOMETRY_PI (3.14159265358979323846f)

static bool odometry_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float odometry_normalize_heading(float heading)
{
    while (heading >= CHASSIS_ODOMETRY_PI) {
        heading -= 2.0f * CHASSIS_ODOMETRY_PI;
    }
    while (heading < -CHASSIS_ODOMETRY_PI) {
        heading += 2.0f * CHASSIS_ODOMETRY_PI;
    }
    return heading;
}

ml_status_t chassis_odometry_init(chassis_odometry_t *odometry,
    float left_mm_per_tick, float right_mm_per_tick,
    float effective_track_mm)
{
    if ((odometry == 0) || !odometry_float_valid(left_mm_per_tick) ||
        !odometry_float_valid(right_mm_per_tick) ||
        !odometry_float_valid(effective_track_mm) ||
        (left_mm_per_tick <= 0.0f) || (right_mm_per_tick <= 0.0f) ||
        (effective_track_mm <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    odometry->left_mm_per_tick = left_mm_per_tick;
    odometry->right_mm_per_tick = right_mm_per_tick;
    odometry->effective_track_mm = effective_track_mm;
    odometry->initialized = true;
    return chassis_odometry_reset(odometry, 0.0f, 0.0f, 0.0f);
}

ml_status_t chassis_odometry_reset(chassis_odometry_t *odometry,
    float x_mm, float y_mm, float heading_rad)
{
    if ((odometry == 0) || !odometry->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!odometry_float_valid(x_mm) || !odometry_float_valid(y_mm) ||
        !odometry_float_valid(heading_rad)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    odometry->pose.x_mm = x_mm;
    odometry->pose.y_mm = y_mm;
    odometry->pose.heading_rad =
        odometry_normalize_heading(heading_rad);
    odometry->pose.encoder_heading_rad =
        odometry_normalize_heading(heading_rad);
    odometry->pose.left_distance_mm = 0.0f;
    odometry->pose.right_distance_mm = 0.0f;
    odometry->pose.left_ticks = 0;
    odometry->pose.right_ticks = 0;
    return ML_STATUS_OK;
}

ml_status_t chassis_odometry_update(chassis_odometry_t *odometry,
    int32_t left_delta_ticks, int32_t right_delta_ticks)
{
    float left_delta_mm;
    float right_delta_mm;
    float encoder_heading_rad;

    if ((odometry == 0) || !odometry->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    left_delta_mm = (float) left_delta_ticks *
        odometry->left_mm_per_tick;
    right_delta_mm = (float) right_delta_ticks *
        odometry->right_mm_per_tick;
    encoder_heading_rad = odometry->pose.encoder_heading_rad +
        (right_delta_mm - left_delta_mm) /
        odometry->effective_track_mm;
    return chassis_odometry_update_with_heading(odometry,
        left_delta_ticks, right_delta_ticks, encoder_heading_rad);
}

ml_status_t chassis_odometry_update_with_heading(
    chassis_odometry_t *odometry,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    float fused_heading_rad)
{
    float left_delta_mm;
    float right_delta_mm;
    float center_delta_mm;
    float encoder_heading_delta;
    float fused_heading_delta;
    float midpoint_heading;

    if ((odometry == 0) || !odometry->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!odometry_float_valid(fused_heading_rad)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    left_delta_mm = (float) left_delta_ticks *
        odometry->left_mm_per_tick;
    right_delta_mm = (float) right_delta_ticks *
        odometry->right_mm_per_tick;
    center_delta_mm = (left_delta_mm + right_delta_mm) * 0.5f;
    encoder_heading_delta = (right_delta_mm - left_delta_mm) /
        odometry->effective_track_mm;
    fused_heading_rad = odometry_normalize_heading(fused_heading_rad);
    fused_heading_delta = odometry_normalize_heading(
        fused_heading_rad - odometry->pose.heading_rad);
    midpoint_heading = odometry->pose.heading_rad +
        (fused_heading_delta * 0.5f);

    odometry->pose.x_mm += center_delta_mm * cosf(midpoint_heading);
    odometry->pose.y_mm += center_delta_mm * sinf(midpoint_heading);
    odometry->pose.heading_rad = fused_heading_rad;
    odometry->pose.encoder_heading_rad = odometry_normalize_heading(
        odometry->pose.encoder_heading_rad + encoder_heading_delta);
    odometry->pose.left_distance_mm += left_delta_mm;
    odometry->pose.right_distance_mm += right_delta_mm;
    odometry->pose.left_ticks += left_delta_ticks;
    odometry->pose.right_ticks += right_delta_ticks;
    return ML_STATUS_OK;
}

chassis_pose_t chassis_odometry_get(const chassis_odometry_t *odometry)
{
    chassis_pose_t result = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0
    };

    if ((odometry != 0) && odometry->initialized) {
        result = odometry->pose;
    }
    return result;
}
