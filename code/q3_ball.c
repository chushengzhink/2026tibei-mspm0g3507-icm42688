#include "q3_ball.h"

#include <float.h>
#include <string.h>

#include "maix_ball_protocol.h"
#include "ml_tim.h"
#include "ml_uart.h"
#include "q3_ball_config.h"
#include "rds3230.h"

#define Q3_CAL_VALID_ROLL_PLUS  (0x01U)
#define Q3_CAL_VALID_ROLL_MINUS (0x02U)
#define Q3_CAL_VALID_BREAK_PLUS (0x04U)
#define Q3_CAL_VALID_BREAK_MINUS (0x08U)
#define Q3_CAL_VALID_ACCEL_PLUS (0x10U)
#define Q3_CAL_VALID_ACCEL_MINUS (0x20U)

typedef struct {
    rds3230_t servo;
    maix_ball_parser_t parser;
    const q3_ball_profile_t *profile;
    q3_calibration_point_t calibration[Q3_PROFILE_POINT_COUNT];
    q3_state_t state;
    q3_mode_t mode;
    q3_rescue_stage_t rescue_stage;
    bool initialized;
    bool vision_ready;
    bool profile_valid;
    bool capture_initialized;
    bool new_measurement;
    bool sequence_started;
    bool sequence_completed;
    bool plus_captured;
    bool final_captured;
    bool brake_active;
    bool boot_axis_retry_used;
    uint8_t consecutive_valid_frames;
    uint8_t profile_index;
    uint8_t rescue_attempts;
    int8_t axis_sign;
    int8_t rescue_direction;
    uint32_t now_ms;
    uint32_t last_control_ms;
    uint32_t state_start_ms;
    uint32_t sequence_start_ms;
    uint32_t boot_start_ms;
    uint32_t map_start_ms;
    uint32_t last_valid_receive_ms;
    uint32_t last_capture_ms;
    uint32_t vision_frame_interval_ms;
    uint32_t confirm_start_ms;
    uint32_t stall_anchor_ms;
    uint32_t rescue_start_ms;
    float neutral_us;
    float response_scale;
    float target_cm;
    float position_cm;
    float velocity_cm_per_s;
    float control_output_us;
    float predicted_stop_cm;
    float stall_anchor_position_cm;
    float stall_progress_cm;
    float rescue_anchor_position_cm;
    float boot_probe_start_cm;
    float boot_plus_displacement_cm;
    float map_previous_velocity_cm_s;
    int16_t raw_center_x_px;
    int16_t raw_center_y_px;
    float raw_score;
    uint32_t observer_outliers;
} q3_ball_context_t;

static q3_ball_context_t g_q3;

static float q3_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float q3_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int8_t q3_sign(float value)
{
    return (value > 0.0f) ? 1 : ((value < 0.0f) ? -1 : 0);
}

static bool q3_finite(float value)
{
    return (value == value) && (value >= -FLT_MAX) &&
        (value <= FLT_MAX);
}

static bool q3_in_range(float value, float minimum, float maximum)
{
    return (value >= minimum) && (value <= maximum);
}

static uint32_t q3_state_elapsed(void)
{
    return g_q3.now_ms - g_q3.state_start_ms;
}

static uint32_t q3_sequence_elapsed(void)
{
    return g_q3.sequence_started ?
        (g_q3.now_ms - g_q3.sequence_start_ms) : 0U;
}

static bool q3_formal_motion_active(void)
{
    return (g_q3.state >= Q3_STATE_PLUS_DRIVE) &&
        (g_q3.state <= Q3_STATE_COMPLETE);
}

static bool q3_map_motion_active(void)
{
    return (g_q3.state == Q3_STATE_MAP_TO_PLUS) ||
        (g_q3.state == Q3_STATE_MAP_TO_MINUS) ||
        (g_q3.state == Q3_STATE_MAP_RETURN_CENTER);
}

static bool q3_boot_motion_active(void)
{
    return (g_q3.state >= Q3_STATE_BOOT_SETTLE) &&
        (g_q3.state <= Q3_STATE_BOOT_RECENTER);
}

static bool q3_motion_active(void)
{
    return q3_formal_motion_active() || q3_map_motion_active() ||
        q3_boot_motion_active();
}

static void q3_reset_rescue(void)
{
    g_q3.rescue_stage = Q3_RESCUE_NONE;
    g_q3.rescue_attempts = 0U;
    g_q3.rescue_direction = 0;
    g_q3.stall_anchor_ms = g_q3.now_ms;
    g_q3.stall_anchor_position_cm = g_q3.position_cm;
    g_q3.stall_progress_cm = 0.0f;
    g_q3.rescue_anchor_position_cm = g_q3.position_cm;
}

static void q3_change_state(q3_state_t state)
{
    g_q3.state = state;
    g_q3.state_start_ms = g_q3.now_ms;
    g_q3.confirm_start_ms = 0U;
    g_q3.brake_active = false;
    q3_reset_rescue();
}

static uint16_t q3_clamp_pulse(float pulse_us)
{
    pulse_us = q3_clamp(pulse_us, (float) Q3_SERVO_MINIMUM_US,
        (float) Q3_SERVO_MAXIMUM_US);
    return (uint16_t) (pulse_us + 0.5f);
}

static void q3_set_raw_pulse(float pulse_us)
{
    (void) rds3230_set_target_us(&g_q3.servo,
        q3_clamp_pulse(pulse_us));
}

/* command_us is physical: positive must accelerate the ball toward +cm. */
static void q3_set_physical_command(float command_us)
{
    float pulse_us = g_q3.neutral_us +
        (float) g_q3.axis_sign * command_us;

    g_q3.control_output_us = command_us;
    q3_set_raw_pulse(pulse_us);
}

static void q3_set_safe(void)
{
    g_q3.control_output_us = 0.0f;
    q3_set_raw_pulse((float) Q3_SERVO_SAFE_US);
}

static void q3_tick_1ms(void *context)
{
    (void) context;
    ++g_q3.now_ms;
}

static bool q3_sample_profile(q3_profile_sample_t *sample)
{
    if (!q3_profile_sample(g_q3.profile, g_q3.position_cm, sample)) {
        return false;
    }
    g_q3.profile_index = sample->lower_index;
    return true;
}

static float q3_direction_rolling(const q3_profile_sample_t *sample,
    int8_t direction)
{
    return ((direction >= 0) ? sample->rolling_plus_us :
        sample->rolling_minus_us) * g_q3.response_scale;
}

static float q3_direction_breakaway(const q3_profile_sample_t *sample,
    int8_t direction)
{
    return ((direction >= 0) ? sample->breakaway_plus_us :
        sample->breakaway_minus_us) * g_q3.response_scale;
}

static float q3_direction_acceleration(const q3_profile_sample_t *sample,
    int8_t direction)
{
    return ((direction >= 0) ? sample->acceleration_plus_cm_s2 :
        sample->acceleration_minus_cm_s2) * g_q3.response_scale;
}

static float q3_command_to_boundary(int8_t physical_direction)
{
    float pulse_us;

    if (physical_direction > 0) {
        pulse_us = (g_q3.axis_sign > 0) ?
            (float) Q3_SERVO_MAXIMUM_US :
            (float) Q3_SERVO_MINIMUM_US;
    } else {
        pulse_us = (g_q3.axis_sign > 0) ?
            (float) Q3_SERVO_MINIMUM_US :
            (float) Q3_SERVO_MAXIMUM_US;
    }
    return (pulse_us - g_q3.neutral_us) /
        (float) g_q3.axis_sign;
}

static float q3_predict_stop(const q3_profile_sample_t *sample)
{
    int8_t direction = q3_sign(g_q3.velocity_cm_per_s);
    float acceleration;
    float distance;

    if (direction == 0) {
        return g_q3.position_cm;
    }
    acceleration = q3_direction_acceleration(sample,
        (int8_t) -direction);
    if (acceleration < 1.0f) {
        acceleration = 1.0f;
    }
    distance = (g_q3.velocity_cm_per_s * g_q3.velocity_cm_per_s) /
        (2.0f * acceleration) + Q3_BRAKE_MARGIN_CM;
    return g_q3.position_cm + (float) direction * distance;
}

static void q3_note_invalid_measurement(void)
{
    g_q3.consecutive_valid_frames = 0U;
}

static void q3_handle_measurement(
    const maix_ball_measurement_t *measurement)
{
    uint32_t capture_interval_ms;
    float dt_s;
    float predicted;
    float residual;

    if (measurement == 0) {
        return;
    }
    g_q3.raw_center_x_px = measurement->center_x_px;
    g_q3.raw_center_y_px = measurement->center_y_px;
    g_q3.raw_score = measurement->score;
    if (!measurement->valid ||
        (measurement->score < Q3_VISION_MINIMUM_SCORE) ||
        !q3_finite(measurement->position_cm) ||
        (measurement->position_cm < Q3_MEASUREMENT_MINIMUM_CM) ||
        (measurement->position_cm > Q3_MEASUREMENT_MAXIMUM_CM)) {
        q3_note_invalid_measurement();
        return;
    }

    g_q3.last_valid_receive_ms = g_q3.now_ms;
    g_q3.new_measurement = true;
    if (!g_q3.capture_initialized) {
        g_q3.capture_initialized = true;
        g_q3.last_capture_ms = measurement->capture_ms;
        g_q3.position_cm = measurement->position_cm;
        g_q3.velocity_cm_per_s = 0.0f;
        g_q3.consecutive_valid_frames = 1U;
        return;
    }

    capture_interval_ms = measurement->capture_ms - g_q3.last_capture_ms;
    if (capture_interval_ms == 0U) {
        g_q3.new_measurement = false;
        return;
    }
    g_q3.vision_frame_interval_ms = capture_interval_ms;
    g_q3.last_capture_ms = measurement->capture_ms;
    if (capture_interval_ms > Q3_MAX_CAPTURE_INTERVAL_MS) {
        g_q3.position_cm = measurement->position_cm;
        g_q3.velocity_cm_per_s = 0.0f;
        g_q3.consecutive_valid_frames = 1U;
        return;
    }
    dt_s = (float) capture_interval_ms / 1000.0f;
    predicted = g_q3.position_cm + g_q3.velocity_cm_per_s * dt_s;
    residual = measurement->position_cm - predicted;
    if (q3_abs(residual) > Q3_MAX_OBSERVER_RESIDUAL_CM) {
        ++g_q3.observer_outliers;
        g_q3.new_measurement = false;
        q3_note_invalid_measurement();
        return;
    }
    g_q3.position_cm = predicted + Q3_OBSERVER_ALPHA * residual;
    g_q3.velocity_cm_per_s += Q3_OBSERVER_BETA * residual / dt_s;
    if (g_q3.consecutive_valid_frames < Q3_VISION_REACQUIRE_FRAMES) {
        ++g_q3.consecutive_valid_frames;
    }
    if (g_q3.consecutive_valid_frames >= Q3_VISION_REACQUIRE_FRAMES) {
        g_q3.vision_ready = true;
    }
}

static void q3_fail(q3_state_t state)
{
    q3_set_safe();
    q3_change_state(state);
}

static float q3_profile_drive_command(float target_cm,
    const q3_profile_sample_t *sample)
{
    float error = target_cm - g_q3.position_cm;
    int8_t direction = q3_sign(error);
    float command;

    if (direction == 0) {
        return sample->balance_command_us;
    }
    command = sample->balance_command_us +
        (float) direction *
        (q3_direction_rolling(sample, direction) +
         Q3_DRIVE_POSITION_GAIN_US_PER_CM * q3_abs(error)) -
        Q3_DRIVE_SPEED_GAIN_US_PER_CM_S * g_q3.velocity_cm_per_s;
    return q3_clamp(command, -Q3_NORMAL_COMMAND_LIMIT_US,
        Q3_NORMAL_COMMAND_LIMIT_US);
}

static float q3_waveform_command(float target_cm,
    const q3_profile_sample_t *sample)
{
    int8_t direction = q3_sign(target_cm - g_q3.position_cm);
    uint32_t elapsed = q3_state_elapsed();
    float magnitude;

    if (elapsed < 550U) {
        magnitude = 130.0f;
    } else if (elapsed < 900U) {
        magnitude = 75.0f;
    } else {
        magnitude = q3_direction_rolling(sample, direction) + 15.0f;
    }
    return sample->balance_command_us + (float) direction * magnitude;
}

static float q3_drive_command(float target_cm,
    const q3_profile_sample_t *sample)
{
    return (g_q3.mode == Q3_MODE_WAVEFORM) ?
        q3_waveform_command(target_cm, sample) :
        q3_profile_drive_command(target_cm, sample);
}

static float q3_brake_command(int8_t travel_direction,
    const q3_profile_sample_t *sample)
{
    int8_t brake_direction = (int8_t) -travel_direction;
    float magnitude = q3_direction_rolling(sample, brake_direction) +
        15.0f + Q3_BRAKE_SPEED_GAIN_US_PER_CM_S *
        q3_abs(g_q3.velocity_cm_per_s);

    return q3_clamp(sample->balance_command_us +
        (float) brake_direction * magnitude,
        -Q3_NORMAL_COMMAND_LIMIT_US, Q3_NORMAL_COMMAND_LIMIT_US);
}

static float q3_capture_command(float target_cm,
    const q3_profile_sample_t *sample)
{
    float error = target_cm - g_q3.position_cm;
    float command = sample->balance_command_us +
        Q3_CAPTURE_POSITION_GAIN_US_PER_CM * error -
        Q3_CAPTURE_SPEED_GAIN_US_PER_CM_S * g_q3.velocity_cm_per_s;
    int8_t direction = q3_sign(error);

    if ((q3_abs(error) > Q3_CAPTURE_INNER_ERROR_CM) &&
        (direction != 0)) {
        command += (float) direction *
            q3_direction_rolling(sample, direction) * 0.60f;
    }
    return q3_clamp(command, -Q3_CAPTURE_COMMAND_LIMIT_US,
        Q3_CAPTURE_COMMAND_LIMIT_US);
}

static bool q3_update_rescue(float target_cm,
    const q3_profile_sample_t *sample, bool rescue_allowed,
    float *command)
{
    int8_t direction = q3_sign(target_cm - g_q3.position_cm);
    float progress;
    float speed_along;
    uint32_t elapsed;

    if ((command == 0) || !rescue_allowed || (direction == 0)) {
        if (g_q3.rescue_stage != Q3_RESCUE_NONE) {
            q3_reset_rescue();
        }
        return false;
    }
    if (g_q3.rescue_direction != direction) {
        q3_reset_rescue();
        g_q3.rescue_direction = direction;
    }
    progress = (float) direction *
        (g_q3.position_cm - g_q3.stall_anchor_position_cm);
    speed_along = (float) direction * g_q3.velocity_cm_per_s;
    g_q3.stall_progress_cm = progress;

    if ((progress >= Q3_STALL_PROGRESS_CM) ||
        (speed_along > Q3_STALL_SPEED_CM_S)) {
        q3_reset_rescue();
        g_q3.rescue_direction = direction;
        return false;
    }
    if (g_q3.rescue_stage == Q3_RESCUE_NONE) {
        if ((g_q3.now_ms - g_q3.stall_anchor_ms) < Q3_STALL_ARM_MS) {
            return false;
        }
        ++g_q3.rescue_attempts;
        g_q3.rescue_stage = Q3_RESCUE_KICK;
        g_q3.rescue_start_ms = g_q3.now_ms;
        g_q3.rescue_anchor_position_cm = g_q3.position_cm;
    }

    elapsed = g_q3.now_ms - g_q3.rescue_start_ms;
    progress = (float) direction *
        (g_q3.position_cm - g_q3.rescue_anchor_position_cm);
    if (progress >= Q3_STALL_PROGRESS_CM) {
        q3_reset_rescue();
        g_q3.rescue_direction = direction;
        return false;
    }
    switch (g_q3.rescue_stage) {
        case Q3_RESCUE_KICK:
            *command = sample->balance_command_us +
                (float) direction *
                (q3_direction_breakaway(sample, direction) +
                 Q3_RESCUE_KICK_MARGIN_US);
            if (elapsed >= Q3_RESCUE_KICK_MS) {
                if (g_q3.rescue_attempts <= Q3_RESCUE_MAXIMUM_ATTEMPTS) {
                    g_q3.rescue_stage = Q3_RESCUE_ROCK;
                } else {
                    g_q3.rescue_stage = Q3_RESCUE_HOLD;
                }
                g_q3.rescue_start_ms = g_q3.now_ms;
                g_q3.rescue_anchor_position_cm = g_q3.position_cm;
            }
            break;
        case Q3_RESCUE_ROCK:
            *command = sample->balance_command_us -
                (float) direction * q3_clamp(
                    q3_direction_breakaway(sample,
                        (int8_t) -direction) * 0.75f,
                    45.0f, 105.0f);
            if ((elapsed >= Q3_RESCUE_ROCK_MS) ||
                ((float) direction *
                 (g_q3.rescue_anchor_position_cm - g_q3.position_cm) >=
                 Q3_RESCUE_ROCK_LIMIT_CM)) {
                g_q3.rescue_stage = Q3_RESCUE_BURST;
                g_q3.rescue_start_ms = g_q3.now_ms;
                g_q3.rescue_anchor_position_cm = g_q3.position_cm;
            }
            break;
        case Q3_RESCUE_BURST:
            *command = q3_command_to_boundary(direction);
            if (elapsed >= Q3_RESCUE_BURST_MS) {
                if (g_q3.rescue_attempts < Q3_RESCUE_MAXIMUM_ATTEMPTS) {
                    g_q3.rescue_stage = Q3_RESCUE_NONE;
                    g_q3.stall_anchor_ms = g_q3.now_ms;
                    g_q3.stall_anchor_position_cm = g_q3.position_cm;
                } else {
                    g_q3.rescue_stage = Q3_RESCUE_HOLD;
                    g_q3.rescue_start_ms = g_q3.now_ms;
                }
            }
            break;
        case Q3_RESCUE_HOLD:
            *command = q3_command_to_boundary(direction);
            break;
        default:
            return false;
    }
    *command = q3_clamp(*command,
        q3_command_to_boundary(-1), q3_command_to_boundary(1));
    return true;
}

static void q3_process_boot(const q3_profile_sample_t *sample)
{
    float command;
    float displacement;
    float mean_response;
    float trim;

    if ((g_q3.now_ms - g_q3.boot_start_ms) >= Q3_BOOT_TIMEOUT_MS ||
        (q3_abs(g_q3.position_cm) > Q3_BOOT_POSITION_LIMIT_CM)) {
        q3_fail(Q3_STATE_CALIBRATION_FAULT);
        return;
    }
    switch (g_q3.state) {
        case Q3_STATE_BOOT_SETTLE:
            q3_set_physical_command(sample->balance_command_us);
            if (q3_state_elapsed() >= Q3_BOOT_SETTLE_MS) {
                g_q3.boot_probe_start_cm = g_q3.position_cm;
                q3_change_state(Q3_STATE_BOOT_PROBE_PLUS);
            }
            break;
        case Q3_STATE_BOOT_PROBE_PLUS:
            command = sample->balance_command_us + q3_clamp(
                q3_direction_breakaway(sample, 1) * 0.80f,
                q3_direction_rolling(sample, 1) + 10.0f, 110.0f);
            q3_set_physical_command(command);
            if (q3_state_elapsed() >= Q3_BOOT_PROBE_MS) {
                displacement = g_q3.position_cm - g_q3.boot_probe_start_cm;
                if ((displacement < -Q3_BOOT_DIRECTION_MINIMUM_CM) &&
                    !g_q3.boot_axis_retry_used) {
                    g_q3.axis_sign = (int8_t) -g_q3.axis_sign;
                    g_q3.boot_axis_retry_used = true;
                    g_q3.boot_start_ms = g_q3.now_ms;
                    q3_change_state(Q3_STATE_BOOT_SETTLE);
                    return;
                }
                if (displacement < -Q3_BOOT_DIRECTION_MINIMUM_CM) {
                    q3_fail(Q3_STATE_CALIBRATION_FAULT);
                    return;
                }
                g_q3.boot_plus_displacement_cm = displacement;
                q3_change_state(Q3_STATE_BOOT_RETURN_PLUS);
            }
            break;
        case Q3_STATE_BOOT_RETURN_PLUS:
            q3_set_physical_command(q3_capture_command(0.0f, sample));
            if (q3_state_elapsed() >= Q3_BOOT_RETURN_MS) {
                g_q3.boot_probe_start_cm = g_q3.position_cm;
                q3_change_state(Q3_STATE_BOOT_PROBE_MINUS);
            }
            break;
        case Q3_STATE_BOOT_PROBE_MINUS:
            command = sample->balance_command_us - q3_clamp(
                q3_direction_breakaway(sample, -1) * 0.80f,
                q3_direction_rolling(sample, -1) + 10.0f, 110.0f);
            q3_set_physical_command(command);
            if (q3_state_elapsed() >= Q3_BOOT_PROBE_MS) {
                displacement = g_q3.position_cm - g_q3.boot_probe_start_cm;
                if (displacement > Q3_BOOT_DIRECTION_MINIMUM_CM) {
                    q3_fail(Q3_STATE_CALIBRATION_FAULT);
                    return;
                }
                mean_response = (q3_abs(g_q3.boot_plus_displacement_cm) +
                    q3_abs(displacement)) * 0.5f;
                if (mean_response >= Q3_BOOT_DIRECTION_MINIMUM_CM) {
                    g_q3.response_scale = q3_clamp(
                        g_q3.profile->probe_displacement_cm / mean_response,
                        Q3_BOOT_RESPONSE_SCALE_MINIMUM,
                        Q3_BOOT_RESPONSE_SCALE_MAXIMUM);
                } else {
                    g_q3.response_scale =
                        Q3_BOOT_RESPONSE_SCALE_MAXIMUM;
                }
                trim = (q3_abs(displacement) -
                    q3_abs(g_q3.boot_plus_displacement_cm)) * 10.0f;
                trim = q3_clamp(trim,
                    -Q3_BOOT_NEUTRAL_TRIM_LIMIT_US,
                    Q3_BOOT_NEUTRAL_TRIM_LIMIT_US);
                g_q3.neutral_us = q3_clamp(g_q3.neutral_us +
                    (float) g_q3.axis_sign * trim,
                    (float) Q3_SERVO_MINIMUM_US + 20.0f,
                    (float) Q3_SERVO_MAXIMUM_US - 20.0f);
                q3_change_state(Q3_STATE_BOOT_RECENTER);
            }
            break;
        case Q3_STATE_BOOT_RECENTER:
            q3_set_physical_command(q3_capture_command(0.0f, sample));
            if ((q3_abs(g_q3.position_cm) <= Q3_READY_ERROR_CM) &&
                (q3_abs(g_q3.velocity_cm_per_s) <=
                 Q3_READY_SPEED_CM_S)) {
                if (g_q3.confirm_start_ms == 0U) {
                    g_q3.confirm_start_ms = g_q3.now_ms;
                } else if ((g_q3.now_ms - g_q3.confirm_start_ms) >=
                           Q3_READY_CONFIRM_MS) {
                    q3_change_state(Q3_STATE_READY);
                    q3_set_physical_command(sample->balance_command_us);
                }
            } else {
                g_q3.confirm_start_ms = 0U;
            }
            break;
        default:
            break;
    }
}

static void q3_process_formal(const q3_profile_sample_t *sample)
{
    float command = sample->balance_command_us;
    bool rescue_allowed = false;
    bool in_final_band = q3_in_range(g_q3.position_cm,
        Q3_MINUS_VALID_MINIMUM_CM, Q3_MINUS_VALID_MAXIMUM_CM);

    if ((g_q3.state != Q3_STATE_COMPLETE) &&
        (q3_sequence_elapsed() >= Q3_SEQUENCE_TIMEOUT_MS)) {
        q3_fail(Q3_STATE_TIMEOUT);
        return;
    }
    g_q3.predicted_stop_cm = q3_predict_stop(sample);
    switch (g_q3.state) {
        case Q3_STATE_PLUS_DRIVE:
            g_q3.target_cm = Q3_PLUS_BRAKE_TARGET_CM;
            if (q3_in_range(g_q3.position_cm,
                    Q3_PLUS_VALID_MINIMUM_CM,
                    Q3_PLUS_VALID_MAXIMUM_CM)) {
                g_q3.plus_captured = true;
                q3_change_state(Q3_STATE_REVERSAL);
                command = q3_brake_command(1, sample);
                break;
            }
            if ((g_q3.predicted_stop_cm >=
                 Q3_PLUS_BRAKE_TARGET_CM) &&
                (g_q3.velocity_cm_per_s > 0.30f)) {
                q3_change_state(Q3_STATE_PLUS_BRAKE);
                command = q3_brake_command(1, sample);
                break;
            }
            command = q3_drive_command(g_q3.target_cm, sample);
            rescue_allowed = true;
            break;
        case Q3_STATE_PLUS_BRAKE:
            g_q3.brake_active = true;
            if (q3_in_range(g_q3.position_cm,
                    Q3_PLUS_VALID_MINIMUM_CM,
                    Q3_PLUS_VALID_MAXIMUM_CM)) {
                g_q3.plus_captured = true;
                q3_change_state(Q3_STATE_REVERSAL);
            } else if ((g_q3.predicted_stop_cm < 4.35f) &&
                       (g_q3.velocity_cm_per_s <= 0.20f)) {
                q3_change_state(Q3_STATE_PLUS_DRIVE);
                command = q3_drive_command(
                    Q3_PLUS_BRAKE_TARGET_CM, sample);
                rescue_allowed = true;
                break;
            }
            command = q3_brake_command(1, sample);
            break;
        case Q3_STATE_REVERSAL:
            g_q3.target_cm = Q3_MINUS_CONTROL_TARGET_CM;
            command = q3_brake_command(1, sample);
            if ((g_q3.velocity_cm_per_s <=
                 Q3_REVERSAL_EXIT_SPEED_CM_S) ||
                (q3_state_elapsed() >= Q3_REVERSAL_TIMEOUT_MS)) {
                q3_change_state(Q3_STATE_MINUS_DRIVE);
            }
            break;
        case Q3_STATE_MINUS_DRIVE:
            g_q3.target_cm = Q3_MINUS_CONTROL_TARGET_CM;
            if (in_final_band) {
                q3_change_state(Q3_STATE_FINAL_CAPTURE);
                command = q3_capture_command(g_q3.target_cm, sample);
                break;
            }
            if ((g_q3.predicted_stop_cm <=
                 Q3_MINUS_CONTROL_TARGET_CM) &&
                (g_q3.velocity_cm_per_s < -0.30f)) {
                q3_change_state(Q3_STATE_MINUS_BRAKE);
                command = q3_brake_command(-1, sample);
                break;
            }
            command = q3_drive_command(g_q3.target_cm, sample);
            rescue_allowed = true;
            break;
        case Q3_STATE_MINUS_BRAKE:
            g_q3.target_cm = Q3_MINUS_CONTROL_TARGET_CM;
            g_q3.brake_active = true;
            if (in_final_band) {
                q3_change_state(Q3_STATE_FINAL_CAPTURE);
                command = q3_capture_command(g_q3.target_cm, sample);
                break;
            }
            if ((g_q3.predicted_stop_cm > -4.35f) &&
                (g_q3.velocity_cm_per_s >= -0.20f)) {
                q3_change_state(Q3_STATE_MINUS_DRIVE);
                command = q3_drive_command(g_q3.target_cm, sample);
                rescue_allowed = true;
                break;
            }
            command = q3_brake_command(-1, sample);
            break;
        case Q3_STATE_FINAL_CAPTURE:
            g_q3.target_cm = Q3_MINUS_CONTROL_TARGET_CM;
            command = q3_capture_command(g_q3.target_cm, sample);
            rescue_allowed = !in_final_band;
            if (in_final_band &&
                (q3_abs(g_q3.velocity_cm_per_s) <=
                 Q3_FINAL_CAPTURE_SPEED_CM_S)) {
                g_q3.final_captured = true;
                if (g_q3.confirm_start_ms == 0U) {
                    g_q3.confirm_start_ms = g_q3.now_ms;
                } else if ((g_q3.now_ms - g_q3.confirm_start_ms) >=
                           Q3_FINAL_CAPTURE_MS) {
                    g_q3.sequence_completed = g_q3.plus_captured;
                    q3_change_state(Q3_STATE_COMPLETE);
                }
            } else {
                g_q3.confirm_start_ms = 0U;
            }
            break;
        case Q3_STATE_COMPLETE:
            g_q3.target_cm = Q3_MINUS_CONTROL_TARGET_CM;
            command = q3_capture_command(g_q3.target_cm, sample);
            break;
        default:
            return;
    }
    if (!q3_update_rescue(g_q3.target_cm, sample,
            rescue_allowed, &command)) {
        g_q3.stall_progress_cm =
            (float) q3_sign(g_q3.target_cm - g_q3.position_cm) *
            (g_q3.position_cm - g_q3.stall_anchor_position_cm);
    }
    q3_set_physical_command(command);
}

static void q3_calibration_reset(void)
{
    uint8_t index;

    for (index = 0U; index < Q3_PROFILE_POINT_COUNT; ++index) {
        const q3_profile_point_t *source = &g_q3.profile->point[index];
        q3_calibration_point_t *point = &g_q3.calibration[index];

        point->position_cm = source->position_cm;
        point->balance_command_us = source->balance_command_us;
        point->rolling_plus_us = source->rolling_plus_us;
        point->rolling_minus_us = source->rolling_minus_us;
        point->breakaway_plus_us = source->breakaway_plus_us;
        point->breakaway_minus_us = source->breakaway_minus_us;
        point->acceleration_plus_cm_s2 =
            source->acceleration_plus_cm_s2;
        point->acceleration_minus_cm_s2 =
            source->acceleration_minus_cm_s2;
        point->valid_mask = 0U;
    }
}

static void q3_calibration_observe(int8_t direction,
    const q3_profile_sample_t *sample, float command)
{
    float coordinate = (g_q3.position_cm - Q3_PROFILE_MINIMUM_CM) /
        Q3_PROFILE_STEP_CM;
    uint8_t index;
    q3_calibration_point_t *point;
    float magnitude = q3_abs(command - sample->balance_command_us);
    float speed_along = (float) direction * g_q3.velocity_cm_per_s;

    if (coordinate <= 0.0f) {
        index = 0U;
    } else if (coordinate >=
               (float) (Q3_PROFILE_POINT_COUNT - 1U)) {
        index = Q3_PROFILE_POINT_COUNT - 1U;
    } else {
        index = (uint8_t) (coordinate + 0.5f);
    }
    point = &g_q3.calibration[index];
    if (index >= Q3_PROFILE_POINT_COUNT) {
        return;
    }
    if (speed_along >= Q3_MAP_VALID_ROLL_SPEED_CM_S) {
        if (direction > 0) {
            if (!(point->valid_mask & Q3_CAL_VALID_ROLL_PLUS) ||
                (magnitude < point->rolling_plus_us)) {
                point->rolling_plus_us = magnitude;
            }
            point->valid_mask |= Q3_CAL_VALID_ROLL_PLUS;
        } else {
            if (!(point->valid_mask & Q3_CAL_VALID_ROLL_MINUS) ||
                (magnitude < point->rolling_minus_us)) {
                point->rolling_minus_us = magnitude;
            }
            point->valid_mask |= Q3_CAL_VALID_ROLL_MINUS;
        }
    }
    if (g_q3.rescue_stage != Q3_RESCUE_NONE) {
        if (direction > 0) {
            if (magnitude > point->breakaway_plus_us) {
                point->breakaway_plus_us = magnitude;
            }
            point->valid_mask |= Q3_CAL_VALID_BREAK_PLUS;
        } else {
            if (magnitude > point->breakaway_minus_us) {
                point->breakaway_minus_us = magnitude;
            }
            point->valid_mask |= Q3_CAL_VALID_BREAK_MINUS;
        }
    }
    if (g_q3.new_measurement &&
        (g_q3.vision_frame_interval_ms > 0U)) {
        float dt_s = (float) g_q3.vision_frame_interval_ms / 1000.0f;
        float acceleration = (g_q3.velocity_cm_per_s -
            g_q3.map_previous_velocity_cm_s) / dt_s;
        float acceleration_along = (float) direction * acceleration;

        if ((acceleration_along > 1.0f) &&
            (acceleration_along <= 80.0f)) {
            if (direction > 0) {
                if (!(point->valid_mask & Q3_CAL_VALID_ROLL_PLUS) ||
                    (magnitude < point->rolling_plus_us)) {
                    point->rolling_plus_us = magnitude;
                }
                point->valid_mask |= Q3_CAL_VALID_ROLL_PLUS;
                point->acceleration_plus_cm_s2 =
                    (point->valid_mask & Q3_CAL_VALID_ACCEL_PLUS) ?
                    (0.75f * point->acceleration_plus_cm_s2 +
                     0.25f * acceleration_along) : acceleration_along;
                point->valid_mask |= Q3_CAL_VALID_ACCEL_PLUS;
            } else {
                if (!(point->valid_mask & Q3_CAL_VALID_ROLL_MINUS) ||
                    (magnitude < point->rolling_minus_us)) {
                    point->rolling_minus_us = magnitude;
                }
                point->valid_mask |= Q3_CAL_VALID_ROLL_MINUS;
                point->acceleration_minus_cm_s2 =
                    (point->valid_mask & Q3_CAL_VALID_ACCEL_MINUS) ?
                    (0.75f * point->acceleration_minus_cm_s2 +
                     0.25f * acceleration_along) : acceleration_along;
                point->valid_mask |= Q3_CAL_VALID_ACCEL_MINUS;
            }
        }
        g_q3.map_previous_velocity_cm_s = g_q3.velocity_cm_per_s;
    }
}

static float q3_map_drive_command(int8_t direction,
    const q3_profile_sample_t *sample)
{
    float desired_speed = (float) direction *
        Q3_MAP_TARGET_SPEED_CM_S;
    float command = sample->balance_command_us +
        (float) direction * q3_direction_rolling(sample, direction) +
        12.0f * (desired_speed - g_q3.velocity_cm_per_s);

    return q3_clamp(command, -Q3_NORMAL_COMMAND_LIMIT_US,
        Q3_NORMAL_COMMAND_LIMIT_US);
}

static void q3_process_map(const q3_profile_sample_t *sample)
{
    int8_t direction;
    float command;
    bool rescue_allowed;

    if ((g_q3.now_ms - g_q3.map_start_ms) >= Q3_MAP_TIMEOUT_MS) {
        q3_fail(Q3_STATE_CALIBRATION_FAULT);
        return;
    }
    if (g_q3.state == Q3_STATE_MAP_RETURN_CENTER) {
        g_q3.target_cm = 0.0f;
        command = (g_q3.position_cm < -Q3_MAP_CENTER_ERROR_CM) ?
            q3_map_drive_command(1, sample) :
            q3_capture_command(0.0f, sample);
        rescue_allowed = q3_abs(g_q3.position_cm) >
            Q3_MAP_CENTER_ERROR_CM;
        (void) q3_update_rescue(0.0f, sample,
            rescue_allowed, &command);
        q3_calibration_observe(1, sample, command);
        q3_set_physical_command(command);
        if ((q3_abs(g_q3.position_cm) <= Q3_MAP_CENTER_ERROR_CM) &&
            (q3_abs(g_q3.velocity_cm_per_s) <=
             Q3_MAP_CENTER_SPEED_CM_S)) {
            if (g_q3.confirm_start_ms == 0U) {
                g_q3.confirm_start_ms = g_q3.now_ms;
            } else if ((g_q3.now_ms - g_q3.confirm_start_ms) >=
                       Q3_MAP_CENTER_CONFIRM_MS) {
                q3_change_state(Q3_STATE_MAP_COMPLETE);
                q3_set_safe();
            }
        } else {
            g_q3.confirm_start_ms = 0U;
        }
        return;
    }
    direction = (g_q3.state == Q3_STATE_MAP_TO_PLUS) ? 1 : -1;
    g_q3.target_cm = (float) direction * Q3_MAP_LIMIT_CM;
    command = q3_map_drive_command(direction, sample);
    (void) q3_update_rescue(g_q3.target_cm, sample, true, &command);
    q3_calibration_observe(direction, sample, command);
    q3_set_physical_command(command);
    if ((direction > 0) &&
        (g_q3.position_cm >= Q3_MAP_LIMIT_CM - 0.20f)) {
        q3_change_state(Q3_STATE_MAP_TO_MINUS);
    } else if ((direction < 0) &&
               (g_q3.position_cm <= -Q3_MAP_LIMIT_CM + 0.20f)) {
        q3_change_state(Q3_STATE_MAP_RETURN_CENTER);
    }
}

static void q3_process_control(void)
{
    q3_profile_sample_t sample;
    uint32_t vision_age;

    if ((g_q3.now_ms - g_q3.last_control_ms) < Q3_CONTROL_PERIOD_MS) {
        return;
    }
    g_q3.last_control_ms = g_q3.now_ms;
    if (!g_q3.profile_valid || !q3_sample_profile(&sample)) {
        q3_fail(Q3_STATE_PROFILE_FAULT);
        return;
    }

    vision_age = g_q3.capture_initialized ?
        (g_q3.now_ms - g_q3.last_valid_receive_ms) : 0xFFFFFFFFUL;
    if (q3_motion_active() && (vision_age >= Q3_VISION_TIMEOUT_MS)) {
        q3_fail(Q3_STATE_VISION_FAULT);
        return;
    }
    if (q3_motion_active() && (vision_age >= Q3_VISION_SOFT_HOLD_MS)) {
        q3_reset_rescue();
        q3_set_physical_command(sample.balance_command_us);
        return;
    }

    if ((g_q3.state == Q3_STATE_WAIT_VISION) && g_q3.vision_ready &&
        (q3_abs(g_q3.position_cm) <= Q3_READY_ERROR_CM + 0.30f)) {
        g_q3.boot_start_ms = g_q3.now_ms;
        q3_change_state(Q3_STATE_BOOT_SETTLE);
    }
    if (q3_boot_motion_active()) {
        q3_process_boot(&sample);
    } else if (q3_formal_motion_active()) {
        q3_process_formal(&sample);
    } else if (q3_map_motion_active()) {
        q3_process_map(&sample);
    } else if ((g_q3.state == Q3_STATE_READY) ||
               (g_q3.state == Q3_STATE_MAP_ARMED)) {
        q3_set_physical_command(sample.balance_command_us);
    } else {
        q3_set_safe();
    }
    g_q3.new_measurement = false;
}

ml_status_t q3_ball_init(void)
{
    ml_status_t status;

    memset(&g_q3, 0, sizeof(g_q3));
    maix_ball_parser_init(&g_q3.parser);
    g_q3.profile = q3_profile_get();
    g_q3.profile_valid = q3_profile_validate(g_q3.profile);
    g_q3.mode = Q3_MODE_PROFILE;
    g_q3.axis_sign = g_q3.profile_valid ?
        g_q3.profile->axis_sign : -1;
    g_q3.neutral_us = g_q3.profile_valid ?
        g_q3.profile->neutral_us : (float) Q3_SERVO_NEUTRAL_DEFAULT_US;
    g_q3.response_scale = 1.0f;
    g_q3.state = g_q3.profile_valid ?
        Q3_STATE_WAIT_VISION : Q3_STATE_PROFILE_FAULT;
    q3_calibration_reset();

    status = uart_init(Q3_VISION_UART, Q3_VISION_UART_BAUD,
        Q3_VISION_UART_PRIORITY);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = rds3230_init(&g_q3.servo, Q3_SERVO_TIMER,
        Q3_SERVO_CHANNEL, Q3_SERVO_FREQUENCY_HZ,
        Q3_SERVO_MINIMUM_US, Q3_SERVO_SAFE_US,
        Q3_SERVO_MAXIMUM_US, Q3_SERVO_MAX_SLEW_US_PER_S);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = tim_interrupt_ms_init_ex(Q3_TIMEBASE_TIMER, 1U,
        Q3_TIMEBASE_PRIORITY, q3_tick_1ms, 0);
    if (status != ML_STATUS_OK) {
        return status;
    }
    g_q3.initialized = true;
    q3_set_safe();
    return ML_STATUS_OK;
}

void q3_ball_process(void)
{
    uint8_t byte;
    maix_ball_measurement_t measurement;
    uint32_t vision_age;

    if (!g_q3.initialized) {
        return;
    }
    while (uart_try_read(Q3_VISION_UART, &byte) == ML_STATUS_OK) {
        if (maix_ball_parser_push(&g_q3.parser, byte, &measurement)) {
            q3_handle_measurement(&measurement);
        }
    }
    vision_age = g_q3.capture_initialized ?
        (g_q3.now_ms - g_q3.last_valid_receive_ms) : 0xFFFFFFFFUL;
    if (g_q3.vision_ready && (vision_age >= Q3_VISION_TIMEOUT_MS) &&
        !q3_motion_active()) {
        g_q3.vision_ready = false;
        g_q3.consecutive_valid_frames = 0U;
        if ((g_q3.state == Q3_STATE_READY) ||
            (g_q3.state == Q3_STATE_MAP_ARMED)) {
            q3_change_state(Q3_STATE_WAIT_VISION);
        }
    }
    (void) rds3230_update(&g_q3.servo, g_q3.now_ms);
    q3_process_control();
}

ml_status_t q3_ball_start(void)
{
    if (!g_q3.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((g_q3.state != Q3_STATE_READY) ||
        (g_q3.mode == Q3_MODE_MANUAL) || !g_q3.vision_ready ||
        (q3_abs(g_q3.position_cm) > Q3_READY_ERROR_CM) ||
        (q3_abs(g_q3.velocity_cm_per_s) > Q3_READY_SPEED_CM_S)) {
        return ML_STATUS_BUSY;
    }
    g_q3.sequence_started = true;
    g_q3.sequence_completed = false;
    g_q3.plus_captured = false;
    g_q3.final_captured = false;
    g_q3.sequence_start_ms = g_q3.now_ms;
    g_q3.target_cm = Q3_PLUS_BRAKE_TARGET_CM;
    q3_change_state(Q3_STATE_PLUS_DRIVE);
    return ML_STATUS_OK;
}

ml_status_t q3_ball_abort(void)
{
    if (!g_q3.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    q3_set_safe();
    q3_change_state(Q3_STATE_ABORTED);
    return ML_STATUS_OK;
}

ml_status_t q3_ball_set_mode(q3_mode_t mode)
{
    if ((mode != Q3_MODE_PROFILE) && (mode != Q3_MODE_WAVEFORM) &&
        (mode != Q3_MODE_MANUAL)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (q3_motion_active() || (g_q3.state == Q3_STATE_MAP_ARMED)) {
        return ML_STATUS_BUSY;
    }
    g_q3.mode = mode;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_set_manual_pulse(uint16_t pulse_us)
{
    if ((pulse_us < Q3_SERVO_MINIMUM_US) ||
        (pulse_us > Q3_SERVO_MAXIMUM_US)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (g_q3.mode != Q3_MODE_MANUAL) {
        return ML_STATUS_BUSY;
    }
    q3_set_raw_pulse((float) pulse_us);
    return ML_STATUS_OK;
}

ml_status_t q3_ball_arm_map_calibration(void)
{
    if (!g_q3.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((g_q3.state != Q3_STATE_READY) || !g_q3.vision_ready) {
        return ML_STATUS_BUSY;
    }
    q3_change_state(Q3_STATE_MAP_ARMED);
    return ML_STATUS_OK;
}

ml_status_t q3_ball_start_map_calibration(void)
{
    if (g_q3.state != Q3_STATE_MAP_ARMED) {
        return ML_STATUS_BUSY;
    }
    q3_calibration_reset();
    g_q3.map_start_ms = g_q3.now_ms;
    g_q3.map_previous_velocity_cm_s = g_q3.velocity_cm_per_s;
    q3_change_state(Q3_STATE_MAP_TO_PLUS);
    return ML_STATUS_OK;
}

uint8_t q3_ball_calibration_count(void)
{
    return Q3_PROFILE_POINT_COUNT;
}

ml_status_t q3_ball_get_calibration_point(uint8_t index,
    q3_calibration_point_t *point)
{
    if (point == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (index >= Q3_PROFILE_POINT_COUNT) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *point = g_q3.calibration[index];
    return ML_STATUS_OK;
}

ml_status_t q3_ball_get_status(q3_ball_status_t *status)
{
    if ((status == 0) || !g_q3.initialized) {
        return (status == 0) ? ML_STATUS_INVALID_ARGUMENT :
            ML_STATUS_NOT_INITIALIZED;
    }
    memset(status, 0, sizeof(*status));
    status->state = g_q3.state;
    status->mode = g_q3.mode;
    status->rescue_stage = g_q3.rescue_stage;
    status->initialized = g_q3.initialized;
    status->vision_ready = g_q3.vision_ready;
    status->profile_valid = g_q3.profile_valid;
    status->sequence_started = g_q3.sequence_started;
    status->sequence_completed = g_q3.sequence_completed;
    status->plus_captured = g_q3.plus_captured;
    status->final_captured = g_q3.final_captured;
    status->brake_active = g_q3.brake_active;
    status->servo_settled =
        rds3230_get_current_us(&g_q3.servo) ==
        rds3230_get_target_us(&g_q3.servo);
    status->axis_sign = g_q3.axis_sign;
    status->profile_index = g_q3.profile_index;
    status->rescue_attempts = g_q3.rescue_attempts;
    status->neutral_us = g_q3.neutral_us;
    status->response_scale = g_q3.response_scale;
    status->target_cm = g_q3.target_cm;
    status->position_cm = g_q3.position_cm;
    status->velocity_cm_per_s = g_q3.velocity_cm_per_s;
    status->error_cm = g_q3.target_cm - g_q3.position_cm;
    status->control_output_us = g_q3.control_output_us;
    status->predicted_stop_cm = g_q3.predicted_stop_cm;
    status->stall_progress_cm = g_q3.stall_progress_cm;
    status->raw_center_x_px = g_q3.raw_center_x_px;
    status->raw_center_y_px = g_q3.raw_center_y_px;
    status->raw_score = g_q3.raw_score;
    status->servo_target_us = rds3230_get_target_us(&g_q3.servo);
    status->servo_current_us = rds3230_get_current_us(&g_q3.servo);
    status->uptime_ms = g_q3.now_ms;
    status->state_elapsed_ms = q3_state_elapsed();
    status->sequence_elapsed_ms = q3_sequence_elapsed();
    status->vision_age_ms = g_q3.capture_initialized ?
        (g_q3.now_ms - g_q3.last_valid_receive_ms) : 0xFFFFFFFFUL;
    status->vision_frame_interval_ms = g_q3.vision_frame_interval_ms;
    status->stall_elapsed_ms = g_q3.now_ms - g_q3.stall_anchor_ms;
    status->valid_frames = g_q3.parser.frames_ok;
    status->crc_errors = g_q3.parser.crc_errors;
    status->length_errors = g_q3.parser.length_errors;
    status->format_errors = g_q3.parser.format_errors;
    status->observer_outliers = g_q3.observer_outliers;
    status->uart_overflows = uart_get_rx_overflow_count(Q3_VISION_UART);
    return ML_STATUS_OK;
}
