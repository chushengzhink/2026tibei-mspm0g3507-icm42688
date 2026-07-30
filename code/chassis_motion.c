#include "chassis_motion.h"

#include <float.h>
#include <math.h>

#define CHASSIS_PI (3.14159265358979323846f)

static bool motion_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float motion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float motion_sign(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float motion_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float motion_wrap(float angle_rad)
{
    while (angle_rad >= CHASSIS_PI) {
        angle_rad -= 2.0f * CHASSIS_PI;
    }
    while (angle_rad < -CHASSIS_PI) {
        angle_rad += 2.0f * CHASSIS_PI;
    }
    return angle_rad;
}

static float motion_ramp(float current, float target, float step)
{
    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

static bool motion_wheel_valid(const chassis_wheel_config_t *wheel)
{
    return (wheel != 0) && motion_float_valid(wheel->mm_per_tick) &&
        (wheel->mm_per_tick > 0.0f) &&
        ((wheel->encoder_sign == 1) || (wheel->encoder_sign == -1)) &&
        ((wheel->motor_sign == 1) || (wheel->motor_sign == -1));
}

static bool motion_config_valid(const chassis_config_t *config)
{
    return (config != 0) && motion_wheel_valid(&config->left) &&
        motion_wheel_valid(&config->right) &&
        motion_float_valid(config->effective_track_mm) &&
        motion_float_valid(config->acceleration_mm_s2) &&
        motion_float_valid(config->maximum_wheel_speed_mm_s) &&
        motion_float_valid(config->distance_tolerance_mm) &&
        motion_float_valid(config->angle_tolerance_deg) &&
        motion_float_valid(config->heading_fusion_time_constant_s) &&
        motion_float_valid(config->heading_rate_imu_weight) &&
        motion_float_valid(config->imu_heading_sign) &&
        motion_float_valid(config->imu_max_delta_deg) &&
        motion_float_valid(config->heading_control_kp) &&
        motion_float_valid(config->yaw_rate_control_kp) &&
        motion_float_valid(config->heading_correction_limit_ratio) &&
        (config->effective_track_mm > 0.0f) &&
        (config->acceleration_mm_s2 > 0.0f) &&
        (config->maximum_wheel_speed_mm_s > 0.0f) &&
        (config->heading_fusion_time_constant_s > 0.0f) &&
        (config->heading_rate_imu_weight >= 0.0f) &&
        (config->heading_rate_imu_weight <= 1.0f) &&
        ((config->imu_heading_sign == 1.0f) ||
         (config->imu_heading_sign == -1.0f)) &&
        (config->imu_max_delta_deg > 0.0f) &&
        (config->heading_control_kp >= 0.0f) &&
        (config->yaw_rate_control_kp >= 0.0f) &&
        (config->heading_correction_limit_ratio >= 0.0f) &&
        (config->heading_correction_limit_ratio <= 1.0f) &&
        (config->control_period_ms > 0U) &&
        (config->imu_stale_ms > 0U) &&
        (config->completion_cycles > 0U) &&
        (config->stall_cycles > 0U);
}

static bool motion_wheel_speeds_valid(const chassis_motion_t *motion,
    float left_mm_s, float right_mm_s)
{
    return motion_float_valid(left_mm_s) &&
        motion_float_valid(right_mm_s) &&
        (motion_abs(left_mm_s) <=
         motion->config.maximum_wheel_speed_mm_s) &&
        (motion_abs(right_mm_s) <=
         motion->config.maximum_wheel_speed_mm_s);
}

static bool motion_busy(const chassis_motion_t *motion)
{
    return (motion->status.result == CHASSIS_RESULT_RUNNING) &&
        ((motion->status.mode == CHASSIS_MODE_MOVE) ||
         (motion->status.mode == CHASSIS_MODE_ROTATE) ||
         (motion->status.mode == CHASSIS_MODE_ARC));
}

static void motion_begin(chassis_motion_t *motion, chassis_mode_t mode)
{
    motion->status.mode = mode;
    motion->status.result = CHASSIS_RESULT_RUNNING;
    motion->status.fault = CHASSIS_FAULT_NONE;
    motion->status.command_progress = 0.0f;
    motion->status.command_remaining = 0.0f;
    motion->start_left_mm = motion->status.pose.left_distance_mm;
    motion->start_right_mm = motion->status.pose.right_distance_mm;
    motion->start_fused_heading_rad =
        chassis_heading_fusion_heading(&motion->heading_fusion);
    motion->requested_left_mm_s = 0.0f;
    motion->requested_right_mm_s = 0.0f;
    motion->profile_speed_mm_s = 0.0f;
    motion->left_stall_cycles = 0U;
    motion->right_stall_cycles = 0U;
    motion->completion_count = 0U;
}

static void motion_finish(chassis_motion_t *motion)
{
    motion->status.mode = CHASSIS_MODE_IDLE;
    motion->status.result = CHASSIS_RESULT_COMPLETE;
    motion->status.target_left_mm_s = 0.0f;
    motion->status.target_right_mm_s = 0.0f;
    motion->profile_speed_mm_s = 0.0f;
}

static void motion_apply_yaw_rate_feedback(
    chassis_motion_t *motion, float target_yaw_rate_rad_s)
{
    float angular_correction_rad_s;
    float wheel_correction_mm_s;
    float correction_limit_mm_s;

    if (!motion->status.heading_fusion_active) {
        return;
    }
    angular_correction_rad_s = motion->config.yaw_rate_control_kp *
        (target_yaw_rate_rad_s -
         chassis_heading_fusion_yaw_rate(&motion->heading_fusion));
    wheel_correction_mm_s = angular_correction_rad_s *
        motion->config.effective_track_mm * 0.5f;
    correction_limit_mm_s =
        motion->config.maximum_wheel_speed_mm_s *
        motion->config.heading_correction_limit_ratio;
    wheel_correction_mm_s = motion_clamp(wheel_correction_mm_s,
        -correction_limit_mm_s, correction_limit_mm_s);
    motion->status.target_left_mm_s -= wheel_correction_mm_s;
    motion->status.target_right_mm_s += wheel_correction_mm_s;
}

static void motion_apply_straight_heading_feedback(
    chassis_motion_t *motion)
{
    float heading_error_rad;
    float angular_correction_rad_s;
    float wheel_correction_mm_s;
    float correction_limit_mm_s;

    heading_error_rad = motion_wrap(
        motion->start_fused_heading_rad -
        chassis_heading_fusion_heading(&motion->heading_fusion));
    angular_correction_rad_s =
        motion->config.heading_control_kp * heading_error_rad -
        motion->config.yaw_rate_control_kp *
        chassis_heading_fusion_yaw_rate(&motion->heading_fusion);
    wheel_correction_mm_s = angular_correction_rad_s *
        motion->config.effective_track_mm * 0.5f;
    correction_limit_mm_s =
        motion->config.maximum_wheel_speed_mm_s *
        motion->config.heading_correction_limit_ratio;
    wheel_correction_mm_s = motion_clamp(wheel_correction_mm_s,
        -correction_limit_mm_s, correction_limit_mm_s);
    motion->status.target_left_mm_s -= wheel_correction_mm_s;
    motion->status.target_right_mm_s += wheel_correction_mm_s;
}

static void motion_fail(chassis_motion_t *motion, chassis_fault_t fault)
{
    motion->status.mode = CHASSIS_MODE_IDLE;
    motion->status.result = CHASSIS_RESULT_FAULT;
    motion->status.fault = fault;
    motion->status.target_left_mm_s = 0.0f;
    motion->status.target_right_mm_s = 0.0f;
    motion->profile_speed_mm_s = 0.0f;
}

static float motion_profile_speed(chassis_motion_t *motion,
    float remaining_mm, float maximum_mm_s, float elapsed_s)
{
    float desired = sqrtf(2.0f * motion->config.acceleration_mm_s2 *
        motion_abs(remaining_mm));

    if (desired > maximum_mm_s) {
        desired = maximum_mm_s;
    }
    if ((motion_abs(remaining_mm) > motion->config.distance_tolerance_mm) &&
        (desired < motion->config.minimum_profile_speed_mm_s)) {
        desired = motion->config.minimum_profile_speed_mm_s;
    }
    motion->profile_speed_mm_s = motion_ramp(
        motion->profile_speed_mm_s, desired,
        motion->config.acceleration_mm_s2 * elapsed_s);
    return motion->profile_speed_mm_s;
}

static bool motion_completion(chassis_motion_t *motion, bool within)
{
    if (within) {
        if (motion->completion_count < UINT8_MAX) {
            ++motion->completion_count;
        }
    } else {
        motion->completion_count = 0U;
    }
    if (motion->completion_count >= motion->config.completion_cycles) {
        motion_finish(motion);
        return true;
    }
    return false;
}

static void motion_update_stall(chassis_motion_t *motion,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    uint8_t elapsed_cycles)
{
    if ((motion_abs(motion->status.target_left_mm_s) >=
         motion->config.stall_command_speed_mm_s) &&
        (left_delta_ticks == 0)) {
        motion->left_stall_cycles += elapsed_cycles;
    } else {
        motion->left_stall_cycles = 0U;
    }
    if ((motion_abs(motion->status.target_right_mm_s) >=
         motion->config.stall_command_speed_mm_s) &&
        (right_delta_ticks == 0)) {
        motion->right_stall_cycles += elapsed_cycles;
    } else {
        motion->right_stall_cycles = 0U;
    }
    if ((motion->left_stall_cycles >= motion->config.stall_cycles) ||
        (motion->right_stall_cycles >= motion->config.stall_cycles)) {
        motion_fail(motion, CHASSIS_FAULT_STALL);
    }
}

static void motion_update_move(chassis_motion_t *motion, float elapsed_s)
{
    float left = motion->status.pose.left_distance_mm -
        motion->start_left_mm;
    float right = motion->status.pose.right_distance_mm -
        motion->start_right_mm;
    float progress = (left + right) * 0.5f;
    float remaining = motion->command_target - progress;
    float speed;
    float correction;

    motion->status.command_progress = progress;
    motion->status.command_remaining = remaining;
    if (motion_completion(motion,
        motion_abs(remaining) <= motion->config.distance_tolerance_mm)) {
        return;
    }
    speed = motion_profile_speed(motion, remaining,
        motion->command_limit, elapsed_s) * motion_sign(remaining);
    motion->status.target_left_mm_s = speed;
    motion->status.target_right_mm_s = speed;
    if (motion->status.heading_fusion_active) {
        motion_apply_straight_heading_feedback(motion);
    } else {
        correction = motion_clamp((left - right) *
            motion->config.straight_correction_kp,
            -motion->config.maximum_wheel_speed_mm_s *
                motion->config.heading_correction_limit_ratio,
            motion->config.maximum_wheel_speed_mm_s *
                motion->config.heading_correction_limit_ratio);
        motion->status.target_left_mm_s -= correction;
        motion->status.target_right_mm_s += correction;
    }
}

static void motion_update_rotate(chassis_motion_t *motion, float elapsed_s)
{
    float progress = chassis_heading_fusion_heading(
        &motion->heading_fusion) - motion->start_fused_heading_rad;
    float remaining = motion->command_target - progress;
    float tolerance = motion->config.angle_tolerance_deg *
        CHASSIS_PI / 180.0f;
    float wheel_limit = motion->command_limit *
        motion->config.effective_track_mm * 0.5f;
    float wheel_remaining = motion_abs(remaining) *
        motion->config.effective_track_mm * 0.5f;
    float speed;

    motion->status.command_progress = progress;
    motion->status.command_remaining = remaining;
    if (motion_completion(motion, motion_abs(remaining) <= tolerance)) {
        return;
    }
    speed = motion_profile_speed(motion, wheel_remaining,
        wheel_limit, elapsed_s) * motion_sign(remaining);
    motion->status.target_left_mm_s = -speed;
    motion->status.target_right_mm_s = speed;
    motion_apply_yaw_rate_feedback(motion,
        2.0f * speed / motion->config.effective_track_mm);
}

static void motion_update_arc(chassis_motion_t *motion, float elapsed_s)
{
    float progress = chassis_heading_fusion_heading(
        &motion->heading_fusion) - motion->start_fused_heading_rad;
    float remaining = motion->command_target - progress;
    float tolerance = motion->config.angle_tolerance_deg *
        CHASSIS_PI / 180.0f;
    float center_remaining = motion_abs(motion->arc_radius_mm * remaining);
    float linear_speed;
    float angular_speed;

    motion->status.command_progress = progress;
    motion->status.command_remaining = remaining;
    if (motion_completion(motion, motion_abs(remaining) <= tolerance)) {
        return;
    }
    linear_speed = motion_profile_speed(motion, center_remaining,
        motion->command_limit, elapsed_s);
    angular_speed = motion_sign(remaining) * linear_speed /
        motion->arc_radius_mm;
    motion->status.target_left_mm_s = linear_speed -
        angular_speed * motion->config.effective_track_mm * 0.5f;
    motion->status.target_right_mm_s = linear_speed +
        angular_speed * motion->config.effective_track_mm * 0.5f;
    motion_apply_yaw_rate_feedback(motion, angular_speed);
}

ml_status_t chassis_motion_init(
    chassis_motion_t *motion, const chassis_config_t *config)
{
    chassis_heading_fusion_config_t fusion_config;
    ml_status_t status;

    if ((motion == 0) || !motion_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion->config = *config;
    status = chassis_odometry_init(&motion->odometry,
        config->left.mm_per_tick, config->right.mm_per_tick,
        config->effective_track_mm);
    if (status != ML_STATUS_OK) {
        return status;
    }
    fusion_config.left_mm_per_tick = config->left.mm_per_tick;
    fusion_config.right_mm_per_tick = config->right.mm_per_tick;
    fusion_config.effective_track_mm = config->effective_track_mm;
    fusion_config.heading_time_constant_s =
        config->heading_fusion_time_constant_s;
    fusion_config.imu_rate_weight = config->heading_rate_imu_weight;
    fusion_config.imu_heading_sign = config->imu_heading_sign;
    fusion_config.imu_max_delta_deg = config->imu_max_delta_deg;
    fusion_config.imu_stale_ms = config->imu_stale_ms;
    status = chassis_heading_fusion_init(&motion->heading_fusion,
        &fusion_config);
    if (status != ML_STATUS_OK) {
        return status;
    }
    motion->status.mode = CHASSIS_MODE_IDLE;
    motion->status.result = CHASSIS_RESULT_IDLE;
    motion->status.fault = CHASSIS_FAULT_NONE;
    motion->status.pose = chassis_odometry_get(&motion->odometry);
    motion->status.target_left_mm_s = 0.0f;
    motion->status.target_right_mm_s = 0.0f;
    motion->status.measured_left_mm_s = 0.0f;
    motion->status.measured_right_mm_s = 0.0f;
    motion->status.command_progress = 0.0f;
    motion->status.command_remaining = 0.0f;
    motion->status.imu_yaw_deg = 0.0f;
    motion->status.encoder_heading_deg = 0.0f;
    motion->status.fused_heading_deg = 0.0f;
    motion->status.fused_yaw_rate_dps = 0.0f;
    motion->status.encoder_total_left = 0;
    motion->status.encoder_total_right = 0;
    motion->status.encoder_invalid_left = 0U;
    motion->status.encoder_invalid_right = 0U;
    motion->status.timestamp_ms = 0U;
    motion->status.pwm_left_count = 0U;
    motion->status.pwm_right_count = 0U;
    motion->status.emergency_stop_latched = false;
    motion->status.heading_fusion_active = false;
    motion->initialized = true;
    motion_begin(motion, CHASSIS_MODE_IDLE);
    motion->status.result = CHASSIS_RESULT_IDLE;
    return ML_STATUS_OK;
}

ml_status_t chassis_motion_set_wheel_speed(chassis_motion_t *motion,
    float left_mm_s, float right_mm_s, chassis_mode_t mode)
{
    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (motion->status.emergency_stop_latched) {
        return ML_STATUS_BUSY;
    }
    if (((mode != CHASSIS_MODE_WHEEL_SPEED) &&
         (mode != CHASSIS_MODE_VELOCITY)) ||
        !motion_wheel_speeds_valid(motion, left_mm_s, right_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion_begin(motion, mode);
    motion->requested_left_mm_s = left_mm_s;
    motion->requested_right_mm_s = right_mm_s;
    motion->status.target_left_mm_s = left_mm_s;
    motion->status.target_right_mm_s = right_mm_s;
    return ML_STATUS_OK;
}

ml_status_t chassis_motion_update_wheel_speed(chassis_motion_t *motion,
    float left_mm_s, float right_mm_s, chassis_mode_t mode)
{
    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (motion->status.emergency_stop_latched ||
        (motion->status.result != CHASSIS_RESULT_RUNNING) ||
        (motion->status.mode != mode)) {
        return ML_STATUS_BUSY;
    }
    if (((mode != CHASSIS_MODE_WHEEL_SPEED) &&
         (mode != CHASSIS_MODE_VELOCITY)) ||
        !motion_wheel_speeds_valid(motion, left_mm_s, right_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion->requested_left_mm_s = left_mm_s;
    motion->requested_right_mm_s = right_mm_s;
    return ML_STATUS_OK;
}

ml_status_t chassis_motion_move(chassis_motion_t *motion,
    float distance_mm, float max_speed_mm_s)
{
    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (motion->status.emergency_stop_latched || motion_busy(motion)) {
        return ML_STATUS_BUSY;
    }
    if (!motion_float_valid(distance_mm) ||
        !motion_float_valid(max_speed_mm_s) || (distance_mm == 0.0f) ||
        (max_speed_mm_s <= 0.0f) ||
        (max_speed_mm_s > motion->config.maximum_wheel_speed_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion_begin(motion, CHASSIS_MODE_MOVE);
    motion->command_target = distance_mm;
    motion->command_limit = max_speed_mm_s;
    return ML_STATUS_OK;
}

ml_status_t chassis_motion_rotate(chassis_motion_t *motion,
    float angle_rad, float max_angular_rad_s)
{
    float wheel_limit;

    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (motion->status.emergency_stop_latched || motion_busy(motion)) {
        return ML_STATUS_BUSY;
    }
    wheel_limit = max_angular_rad_s *
        motion->config.effective_track_mm * 0.5f;
    if (!motion_float_valid(angle_rad) ||
        !motion_float_valid(max_angular_rad_s) || (angle_rad == 0.0f) ||
        (max_angular_rad_s <= 0.0f) ||
        (wheel_limit > motion->config.maximum_wheel_speed_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion_begin(motion, CHASSIS_MODE_ROTATE);
    motion->command_target = angle_rad;
    motion->command_limit = max_angular_rad_s;
    return ML_STATUS_OK;
}

ml_status_t chassis_motion_arc(chassis_motion_t *motion,
    float radius_mm, float angle_rad, float max_speed_mm_s)
{
    float outer_ratio;

    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (motion->status.emergency_stop_latched || motion_busy(motion)) {
        return ML_STATUS_BUSY;
    }
    if (!motion_float_valid(radius_mm) ||
        (radius_mm <= motion->config.effective_track_mm * 0.5f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    outer_ratio = 1.0f +
        (motion->config.effective_track_mm * 0.5f / radius_mm);
    if (!motion_float_valid(angle_rad) ||
        !motion_float_valid(max_speed_mm_s) ||
        (angle_rad == 0.0f) || (max_speed_mm_s <= 0.0f) ||
        ((max_speed_mm_s * outer_ratio) >
         motion->config.maximum_wheel_speed_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    motion_begin(motion, CHASSIS_MODE_ARC);
    motion->command_target = angle_rad;
    motion->command_limit = max_speed_mm_s;
    motion->arc_radius_mm = radius_mm;
    return ML_STATUS_OK;
}

void chassis_motion_stop(chassis_motion_t *motion, bool emergency)
{
    if ((motion == 0) || !motion->initialized) {
        return;
    }
    motion->status.mode = CHASSIS_MODE_IDLE;
    motion->status.target_left_mm_s = 0.0f;
    motion->status.target_right_mm_s = 0.0f;
    motion->requested_left_mm_s = 0.0f;
    motion->requested_right_mm_s = 0.0f;
    motion->profile_speed_mm_s = 0.0f;
    if (emergency) {
        motion->status.result = CHASSIS_RESULT_FAULT;
        motion->status.fault = CHASSIS_FAULT_EMERGENCY_STOP;
        motion->status.emergency_stop_latched = true;
    } else {
        motion->status.result = CHASSIS_RESULT_CANCELLED;
    }
}

ml_status_t chassis_motion_reset_pose(chassis_motion_t *motion,
    float x_mm, float y_mm, float heading_rad)
{
    ml_status_t status;

    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    status = chassis_odometry_reset(&motion->odometry,
        x_mm, y_mm, heading_rad);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = chassis_heading_fusion_reset(&motion->heading_fusion,
        heading_rad);
    if (status == ML_STATUS_OK) {
        motion->status.pose = chassis_odometry_get(&motion->odometry);
        motion->status.encoder_heading_deg =
            heading_rad * 180.0f / CHASSIS_PI;
        motion->status.fused_heading_deg =
            motion->status.encoder_heading_deg;
        motion->status.fused_yaw_rate_dps = 0.0f;
        motion->status.heading_fusion_active = false;
    }
    return status;
}

void chassis_motion_set_imu_sample(chassis_motion_t *motion,
    float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid)
{
    if ((motion == 0) || !motion->initialized) {
        return;
    }
    if (valid && motion_float_valid(yaw_deg)) {
        motion->status.imu_yaw_deg = yaw_deg;
    }
    chassis_heading_fusion_set_imu(&motion->heading_fusion,
        yaw_deg, body_gyro_z_dps, timestamp_ms, valid);
}

ml_status_t chassis_motion_update(chassis_motion_t *motion,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    uint8_t elapsed_cycles)
{
    float elapsed_s;
    ml_status_t status;

    if ((motion == 0) || !motion->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (elapsed_cycles == 0U) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = chassis_heading_fusion_update(&motion->heading_fusion,
        left_delta_ticks, right_delta_ticks,
        elapsed_cycles, motion->config.control_period_ms);
    if (status == ML_STATUS_OK) {
        status = chassis_odometry_update_with_heading(&motion->odometry,
            left_delta_ticks, right_delta_ticks,
            chassis_heading_fusion_heading(&motion->heading_fusion));
    }
    if (status != ML_STATUS_OK) {
        motion_fail(motion, CHASSIS_FAULT_ENCODER);
        return status;
    }
    motion->status.pose = chassis_odometry_get(&motion->odometry);
    motion->status.encoder_heading_deg =
        chassis_heading_fusion_encoder_heading(
            &motion->heading_fusion) * 180.0f / CHASSIS_PI;
    motion->status.fused_heading_deg =
        chassis_heading_fusion_heading(
            &motion->heading_fusion) * 180.0f / CHASSIS_PI;
    motion->status.fused_yaw_rate_dps =
        chassis_heading_fusion_yaw_rate(
            &motion->heading_fusion) * 180.0f / CHASSIS_PI;
    motion->status.heading_fusion_active =
        chassis_heading_fusion_active(&motion->heading_fusion);
    motion->status.timestamp_ms +=
        (uint32_t) elapsed_cycles * motion->config.control_period_ms;
    elapsed_s = (float) elapsed_cycles *
        (float) motion->config.control_period_ms / 1000.0f;

    if (motion->status.result == CHASSIS_RESULT_RUNNING) {
        if (motion->status.mode == CHASSIS_MODE_MOVE) {
            motion_update_move(motion, elapsed_s);
        } else if (motion->status.mode == CHASSIS_MODE_ROTATE) {
            motion_update_rotate(motion, elapsed_s);
        } else if (motion->status.mode == CHASSIS_MODE_ARC) {
            motion_update_arc(motion, elapsed_s);
        } else if (motion->status.mode == CHASSIS_MODE_VELOCITY) {
            float target_yaw_rate_rad_s;

            motion->status.target_left_mm_s =
                motion->requested_left_mm_s;
            motion->status.target_right_mm_s =
                motion->requested_right_mm_s;
            target_yaw_rate_rad_s =
                (motion->requested_right_mm_s -
                 motion->requested_left_mm_s) /
                motion->config.effective_track_mm;
            motion_apply_yaw_rate_feedback(motion,
                target_yaw_rate_rad_s);
        } else if (motion->status.mode == CHASSIS_MODE_WHEEL_SPEED) {
            motion->status.target_left_mm_s =
                motion->requested_left_mm_s;
            motion->status.target_right_mm_s =
                motion->requested_right_mm_s;
        }
        if (motion->status.result == CHASSIS_RESULT_RUNNING) {
            motion->status.target_left_mm_s = motion_clamp(
                motion->status.target_left_mm_s,
                -motion->config.maximum_wheel_speed_mm_s,
                motion->config.maximum_wheel_speed_mm_s);
            motion->status.target_right_mm_s = motion_clamp(
                motion->status.target_right_mm_s,
                -motion->config.maximum_wheel_speed_mm_s,
                motion->config.maximum_wheel_speed_mm_s);
            motion_update_stall(motion, left_delta_ticks,
                right_delta_ticks, elapsed_cycles);
        }
    }
    return ML_STATUS_OK;
}
