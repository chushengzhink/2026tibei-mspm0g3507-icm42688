#include "chassis_track_line_control.h"

#include <float.h>

#define LINE_CONTROL_OUTER_POSITION_QUARTER_MM (161)

static const int16_t g_line_sensor_positions_quarter_mm[4] = {
    -161, -29, 29, 161
};

const chassis_track_line_control_config_t
    g_chassis_track_line_control_default_config = {
        214.2f,
        500.0f,
        90.0f,
        0.22f,
        120.0f,
        0.35f,
        1.8f,
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        20U
    };

static bool line_control_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float line_control_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float line_control_min(float left, float right)
{
    return left < right ? left : right;
}

static bool line_control_config_valid(
    const chassis_track_line_control_config_t *config)
{
    return (config != 0) &&
        line_control_float_valid(config->effective_track_mm) &&
        line_control_float_valid(config->maximum_wheel_speed_mm_s) &&
        line_control_float_valid(config->maximum_correction_mm_s) &&
        line_control_float_valid(config->correction_ratio) &&
        line_control_float_valid(
            config->outer_single_maximum_correction_mm_s) &&
        line_control_float_valid(
            config->outer_single_correction_ratio) &&
        line_control_float_valid(config->kp) &&
        line_control_float_valid(config->ki) &&
        line_control_float_valid(config->kd) &&
        line_control_float_valid(config->pid_output_limit) &&
        line_control_float_valid(config->pid_integral_limit) &&
        (config->effective_track_mm > 0.0f) &&
        (config->maximum_wheel_speed_mm_s > 0.0f) &&
        (config->maximum_correction_mm_s > 0.0f) &&
        (config->maximum_correction_mm_s <=
         config->maximum_wheel_speed_mm_s) &&
        (config->correction_ratio > 0.0f) &&
        (config->correction_ratio <= 1.0f) &&
        (config->outer_single_maximum_correction_mm_s > 0.0f) &&
        (config->outer_single_maximum_correction_mm_s <=
         config->maximum_wheel_speed_mm_s) &&
        (config->outer_single_correction_ratio > 0.0f) &&
        (config->outer_single_correction_ratio <= 1.0f) &&
        (config->kp > 0.0f) &&
        (config->ki >= 0.0f) &&
        (config->kd >= 0.0f) &&
        (config->pid_output_limit > 0.0f) &&
        (config->pid_output_limit <= 1.0f) &&
        (config->pid_integral_limit >= 0.0f) &&
        (config->pid_integral_limit <= config->pid_output_limit) &&
        (config->control_period_ms > 0U);
}

static bool line_control_centroid_error(
    uint8_t black_bits, float *error)
{
    int16_t position_sum = 0;
    uint8_t active_count = 0U;
    uint8_t index;

    for (index = 0U; index < 4U; ++index) {
        if ((black_bits & (uint8_t) (1U << index)) != 0U) {
            position_sum = (int16_t) (position_sum +
                g_line_sensor_positions_quarter_mm[index]);
            ++active_count;
        }
    }
    if (active_count == 0U) {
        return false;
    }
    *error = -(float) position_sum /
        ((float) active_count *
         (float) LINE_CONTROL_OUTER_POSITION_QUARTER_MM);
    return true;
}

static chassis_track_line_state_t line_control_classify(
    bool line_valid, float error)
{
    if (!line_valid) {
        return CHASSIS_TRACK_LINE_LOST;
    }
    if (error > 0.0f) {
        return CHASSIS_TRACK_LINE_LEFT;
    }
    if (error < 0.0f) {
        return CHASSIS_TRACK_LINE_RIGHT;
    }
    return CHASSIS_TRACK_LINE_CENTERED;
}

static float line_control_correction(
    const chassis_track_line_control_t *control,
    float pid_output, float linear_mm_s, bool outer_single_boost)
{
    float correction_ratio = outer_single_boost ?
        control->config.outer_single_correction_ratio :
        control->config.correction_ratio;
    float maximum_correction_mm_s = outer_single_boost ?
        control->config.outer_single_maximum_correction_mm_s :
        control->config.maximum_correction_mm_s;
    float magnitude = line_control_min(
        linear_mm_s * correction_ratio,
        maximum_correction_mm_s);

    return pid_output * magnitude;
}

static bool line_control_outer_single(uint8_t black_bits)
{
    return (black_bits == 0x01U) || (black_bits == 0x08U);
}

static float line_control_steering_bias(
    float line_correction_mm_s, float route_correction_mm_s,
    float maximum_correction_mm_s)
{
    float combined;

    if (line_correction_mm_s > 0.0f) {
        if (route_correction_mm_s < 0.0f) {
            route_correction_mm_s = 0.0f;
        }
        combined = line_correction_mm_s + route_correction_mm_s;
        return line_control_min(combined, maximum_correction_mm_s);
    }
    if (line_correction_mm_s < 0.0f) {
        if (route_correction_mm_s > 0.0f) {
            route_correction_mm_s = 0.0f;
        }
        combined = line_correction_mm_s + route_correction_mm_s;
        if (combined < -maximum_correction_mm_s) {
            return -maximum_correction_mm_s;
        }
        return combined;
    }
    if (route_correction_mm_s > maximum_correction_mm_s) {
        return maximum_correction_mm_s;
    }
    if (route_correction_mm_s < -maximum_correction_mm_s) {
        return -maximum_correction_mm_s;
    }
    return route_correction_mm_s;
}

static void line_control_limit_wheels(
    const chassis_track_line_control_t *control,
    chassis_track_line_control_output_t *output)
{
    float maximum_magnitude = line_control_abs(output->left_mm_s);
    float scale;

    if (line_control_abs(output->right_mm_s) > maximum_magnitude) {
        maximum_magnitude = line_control_abs(output->right_mm_s);
    }
    if (maximum_magnitude > control->config.maximum_wheel_speed_mm_s) {
        scale = control->config.maximum_wheel_speed_mm_s /
            maximum_magnitude;
        output->left_mm_s *= scale;
        output->right_mm_s *= scale;
        output->correction_mm_s *= scale;
    }
}

ml_status_t chassis_track_line_control_init(
    chassis_track_line_control_t *control,
    const chassis_track_line_control_config_t *config)
{
    ml_status_t status;

    if ((control == 0) || !line_control_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    control->config = *config;
    control->initialized = false;
    status = pid_init(&control->pid, POSITION_PID,
        config->kp, config->ki, config->kd);
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(&control->pid,
            -config->pid_output_limit, config->pid_output_limit,
            -config->pid_integral_limit, config->pid_integral_limit);
    }
    if (status == ML_STATUS_OK) {
        control->initialized = true;
        chassis_track_line_control_reset(control);
    }
    return status;
}

void chassis_track_line_control_reset(
    chassis_track_line_control_t *control)
{
    if ((control == 0) || !control->initialized) {
        return;
    }
    (void) pid_reset(&control->pid);
    control->last_line_error = 0.0f;
    control->last_valid_black_bits = 0U;
    control->lost_ms = 0U;
}

ml_status_t chassis_track_line_control_update(
    chassis_track_line_control_t *control, const line_sample_t *sample,
    float requested_linear_mm_s, float requested_angular_rad_s,
    chassis_track_line_control_output_t *output)
{
    chassis_track_line_state_t state;
    float line_error;
    float pid_output;
    float linear_mm_s = requested_linear_mm_s;
    float angular_rad_s = requested_angular_rad_s;
    float half_track;
    float steering_bias_mm_s;
    float maximum_correction_mm_s;
    bool line_valid;
    bool outer_single_boost;
    ml_status_t status;

    if ((control == 0) || !control->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((sample == 0) || (output == 0) || sample->io_fault ||
        !line_control_float_valid(requested_linear_mm_s) ||
        !line_control_float_valid(requested_angular_rad_s) ||
        (requested_linear_mm_s < 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    line_valid = line_control_centroid_error(
        sample->black_bits, &line_error);
    if (!line_valid) {
        line_error = control->last_line_error;
        if (control->lost_ms <= UINT16_MAX -
            control->config.control_period_ms) {
            control->lost_ms = (uint16_t) (control->lost_ms +
                control->config.control_period_ms);
        } else {
            control->lost_ms = UINT16_MAX;
        }
    } else {
        control->last_line_error = line_error;
        control->last_valid_black_bits = sample->black_bits;
        control->lost_ms = 0U;
    }
    outer_single_boost = line_control_outer_single(
        control->last_valid_black_bits);
    maximum_correction_mm_s = outer_single_boost ?
        control->config.outer_single_maximum_correction_mm_s :
        control->config.maximum_correction_mm_s;
    state = line_control_classify(line_valid, line_error);
    control->pid.target = line_error;
    control->pid.now = 0.0f;
    status = pid_cal(&control->pid);
    if (status != ML_STATUS_OK) {
        return status;
    }
    pid_output = control->pid.out;

    output->correction_mm_s = line_control_correction(
        control, pid_output, linear_mm_s, outer_single_boost);
    half_track = control->config.effective_track_mm * 0.5f;
    steering_bias_mm_s = line_control_steering_bias(
        output->correction_mm_s, angular_rad_s * half_track,
        maximum_correction_mm_s);
    output->left_mm_s = linear_mm_s - steering_bias_mm_s;
    output->right_mm_s = linear_mm_s + steering_bias_mm_s;
    line_control_limit_wheels(control, output);
    output->linear_mm_s = 0.5f *
        (output->left_mm_s + output->right_mm_s);
    output->angular_rad_s =
        (output->right_mm_s - output->left_mm_s) /
        control->config.effective_track_mm;
    output->lost_ms = control->lost_ms;
    output->line_state = state;
    output->line_valid = line_valid;
    output->recovering = !line_valid;
    return ML_STATUS_OK;
}
