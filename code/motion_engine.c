#include "motion_engine.h"

#define MOTION_PI                         (3.14159265f)
#define MOTION_ACCEL_MM_S2                (400.0f)
#define MOTION_MIN_SPEED_MM_S             (45.0f)
#define MOTION_STALL_SPEED_MM_S           (30.0f)
#define MOTION_STALL_LIMIT_UPDATES        (8U)
#define MOTION_LINE_LOST_LIMIT_TICKS      (100U)
#define MOTION_LINE_ALIGN_SAMPLES         (4U)
#define MOTION_TRANSVERSE_SAMPLES         (3U)
#define MOTION_LINE_END_SAMPLES           (3U)
#define MOTION_CIRCLE_SYNC_LIMIT_UPDATES  (25U)

static float motion_engine_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float motion_engine_clamp(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float motion_engine_mean_distance(const motion_engine_t *engine)
{
    return (engine->left_distance_mm + engine->right_distance_mm) * 0.5f;
}

static float motion_engine_mean_absolute_distance(
    const motion_engine_t *engine)
{
    return (motion_engine_abs(engine->left_distance_mm) +
        motion_engine_abs(engine->right_distance_mm)) * 0.5f;
}

static float motion_engine_ramp(float current, float target)
{
    const float step = MOTION_ACCEL_MM_S2 *
        ((float) ROBOT_CONTROL_TICK_MS *
         (float) ROBOT_SPEED_TICK_DIVIDER / 1000.0f);

    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

static float motion_engine_position_speed(
    float remaining_mm, float cruise_mm_s)
{
    float speed = cruise_mm_s;

    if (remaining_mm < 40.0f) {
        speed = MOTION_MIN_SPEED_MM_S;
    } else if ((remaining_mm < 120.0f) && (speed > 75.0f)) {
        speed = 75.0f;
    }
    return speed;
}

static void motion_engine_finish(motion_engine_t *engine)
{
    engine->current_base_speed_mm_s = 0.0f;
    engine->mode = MOTION_MODE_IDLE;
    engine->result = MOTION_RESULT_COMPLETE;
    engine->stop_requested = true;
}

void motion_engine_fail(motion_engine_t *engine, motion_fault_t fault)
{
    if (engine == 0) {
        return;
    }
    engine->current_base_speed_mm_s = 0.0f;
    engine->mode = MOTION_MODE_IDLE;
    engine->fault = fault;
    engine->result = MOTION_RESULT_FAULT;
    engine->stop_requested = true;
}

static void motion_engine_begin(
    motion_engine_t *engine, motion_mode_t mode, float speed_mm_s)
{
    engine->mode = mode;
    engine->result = MOTION_RESULT_RUNNING;
    engine->fault = MOTION_FAULT_NONE;
    engine->left_distance_mm = 0.0f;
    engine->right_distance_mm = 0.0f;
    engine->target_distance_mm = 0.0f;
    engine->command_speed_mm_s = speed_mm_s;
    engine->current_base_speed_mm_s = 0.0f;
    engine->last_line_error_mm = 0.0f;
    engine->left_stall_updates = 0U;
    engine->right_stall_updates = 0U;
    engine->line_lost_ticks = 0U;
    engine->circle_sync_updates = 0U;
    engine->transverse_samples = 0U;
    engine->clear_samples = 0U;
    engine->line_align_samples = 0U;
    engine->line_end_stage = MOTION_LINE_END_SEARCH;
    engine->line_end_samples = 0U;
    engine->line_end_expected_mm = 0.0f;
    engine->stop_requested = false;
}

static void motion_engine_update_stall(motion_engine_t *engine,
    float left_speed, float right_speed,
    int32_t left_delta, int32_t right_delta)
{
    if ((motion_engine_abs(left_speed) >= MOTION_STALL_SPEED_MM_S) &&
        (left_delta == 0)) {
        ++engine->left_stall_updates;
    } else {
        engine->left_stall_updates = 0U;
    }
    if ((motion_engine_abs(right_speed) >= MOTION_STALL_SPEED_MM_S) &&
        (right_delta == 0)) {
        ++engine->right_stall_updates;
    } else {
        engine->right_stall_updates = 0U;
    }
    if ((engine->left_stall_updates >= MOTION_STALL_LIMIT_UPDATES) ||
        (engine->right_stall_updates >= MOTION_STALL_LIMIT_UPDATES)) {
        motion_engine_fail(engine, MOTION_FAULT_ENCODER_STALL);
    }
}

static void motion_engine_compute_straight_speeds(
    motion_engine_t *engine, float *left_speed,
    float *right_speed, bool follow_line)
{
    float remaining;
    float desired;
    float correction;
    float wheel_difference;

    if ((engine->mode == MOTION_MODE_FIND_A) &&
        (engine->find_stage != MOTION_FIND_A_TO_AXLE)) {
        desired = engine->command_speed_mm_s;
    } else {
        remaining = engine->target_distance_mm -
            motion_engine_mean_distance(engine);
        desired = motion_engine_position_speed(
            remaining, engine->command_speed_mm_s);
    }
    if (follow_line && engine->line.lost && (desired > 60.0f)) {
        desired = 60.0f;
    }
    engine->current_base_speed_mm_s = motion_engine_ramp(
        engine->current_base_speed_mm_s, desired);

    if (follow_line) {
        correction = (ROBOT_LINE_KP * engine->line.error_mm) +
            (ROBOT_LINE_KD *
             (engine->line.error_mm - engine->last_line_error_mm));
        correction = motion_engine_clamp(correction,
            -ROBOT_LINE_MAX_CORRECTION_MM_S,
            ROBOT_LINE_MAX_CORRECTION_MM_S);
        engine->last_line_error_mm = engine->line.error_mm;
        *left_speed = engine->current_base_speed_mm_s + correction;
        *right_speed = engine->current_base_speed_mm_s - correction;
    } else {
        wheel_difference = engine->left_distance_mm -
            engine->right_distance_mm;
        correction = motion_engine_clamp(
            wheel_difference * 2.0f, -20.0f, 20.0f);
        *left_speed = engine->current_base_speed_mm_s - correction;
        *right_speed = engine->current_base_speed_mm_s + correction;
    }
}

static void motion_engine_compute_turn_speeds(
    motion_engine_t *engine, float *left_speed, float *right_speed)
{
    float progress = motion_engine_mean_absolute_distance(engine);
    float desired = engine->command_speed_mm_s;

    if (progress > (engine->turn_target_mm * 0.70f)) {
        desired = MOTION_MIN_SPEED_MM_S;
    }
    engine->current_base_speed_mm_s = motion_engine_ramp(
        engine->current_base_speed_mm_s, desired);
    if (engine->turn_right) {
        *left_speed = engine->current_base_speed_mm_s;
        *right_speed = -engine->current_base_speed_mm_s;
    } else {
        *left_speed = -engine->current_base_speed_mm_s;
        *right_speed = engine->current_base_speed_mm_s;
    }
}

static void motion_engine_compute_circle_speeds(
    motion_engine_t *engine, float *left_speed, float *right_speed)
{
    float left_progress = motion_engine_abs(engine->left_distance_mm) /
        engine->circle_left_target_mm;
    float right_progress = motion_engine_abs(engine->right_distance_mm) /
        engine->circle_right_target_mm;
    float average_progress = (left_progress + right_progress) * 0.5f;
    float center_distance = (engine->circle_left_target_mm +
        engine->circle_right_target_mm) * 0.5f;
    float left_ratio = engine->circle_left_target_mm / center_distance;
    float right_ratio = engine->circle_right_target_mm / center_distance;
    float desired = engine->command_speed_mm_s;
    float correction;

    if (average_progress > 0.97f) {
        desired = MOTION_MIN_SPEED_MM_S;
    } else if ((average_progress > 0.85f) && (desired > 75.0f)) {
        desired = 75.0f;
    }
    engine->current_base_speed_mm_s = motion_engine_ramp(
        engine->current_base_speed_mm_s, desired);
    correction = motion_engine_clamp(
        (left_progress - right_progress) * ROBOT_CIRCLE_SYNC_KP,
        -ROBOT_CIRCLE_MAX_CORRECTION_MM_S,
        ROBOT_CIRCLE_MAX_CORRECTION_MM_S);
    *left_speed = (engine->current_base_speed_mm_s - correction) *
        left_ratio;
    *right_speed = (engine->current_base_speed_mm_s + correction) *
        right_ratio;

    if ((average_progress > 0.10f) &&
        (motion_engine_abs(left_progress - right_progress) > 0.10f)) {
        ++engine->circle_sync_updates;
    } else {
        engine->circle_sync_updates = 0U;
    }
    if (engine->circle_sync_updates >=
        MOTION_CIRCLE_SYNC_LIMIT_UPDATES) {
        motion_engine_fail(engine, MOTION_FAULT_CIRCLE_SYNC);
    }
}

static bool motion_engine_check_completion(motion_engine_t *engine)
{
    float progress;
    float left_progress;
    float right_progress;

    if ((engine->mode == MOTION_MODE_STRAIGHT) ||
        (engine->mode == MOTION_MODE_LINE) ||
        ((engine->mode == MOTION_MODE_LINE_TO_END) &&
         (engine->line_end_stage == MOTION_LINE_END_TO_AXLE)) ||
        ((engine->mode == MOTION_MODE_FIND_A) &&
         (engine->find_stage == MOTION_FIND_A_TO_AXLE))) {
        if (motion_engine_mean_distance(engine) >=
            engine->target_distance_mm) {
            motion_engine_finish(engine);
            return true;
        }
    } else if (engine->mode == MOTION_MODE_TURN) {
        progress = motion_engine_mean_absolute_distance(engine);
        if (engine->align_to_line) {
            if (engine->line_align_samples >=
                MOTION_LINE_ALIGN_SAMPLES) {
                motion_engine_finish(engine);
                return true;
            }
            if (progress >= (engine->turn_target_mm * 1.30f)) {
                motion_engine_fail(engine, MOTION_FAULT_LINE_NOT_FOUND);
                return true;
            }
        } else if (progress >= engine->turn_target_mm) {
            motion_engine_finish(engine);
            return true;
        }
    } else if (engine->mode == MOTION_MODE_CIRCLE) {
        left_progress = motion_engine_abs(engine->left_distance_mm) /
            engine->circle_left_target_mm;
        right_progress = motion_engine_abs(engine->right_distance_mm) /
            engine->circle_right_target_mm;
        if ((left_progress >= 1.000f) && (right_progress >= 1.000f)) {
            motion_engine_finish(engine);
            return true;
        }
    }
    return false;
}

ml_status_t motion_engine_init(motion_engine_t *engine,
    const robot_calibration_t *calibration, line_sample_t initial_line)
{
    if ((engine == 0) || (calibration == 0) ||
        (calibration->left_mm_per_tick <= 0.0f) ||
        (calibration->right_mm_per_tick <= 0.0f) ||
        (calibration->effective_track_mm <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    engine->calibration = calibration;
    engine->mode = MOTION_MODE_IDLE;
    engine->result = MOTION_RESULT_IDLE;
    engine->fault = MOTION_FAULT_NONE;
    engine->uptime_ticks = 0U;
    engine->left_distance_mm = 0.0f;
    engine->right_distance_mm = 0.0f;
    engine->target_distance_mm = 0.0f;
    engine->line = initial_line;
    engine->stop_requested = false;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_find_a(
    motion_engine_t *engine, float speed_mm_s)
{
    if ((engine == 0) || (speed_mm_s <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    motion_engine_begin(engine, MOTION_MODE_FIND_A, speed_mm_s);
    engine->find_stage = MOTION_FIND_A_SEARCH;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_straight(
    motion_engine_t *engine, float distance_mm, float speed_mm_s)
{
    if ((engine == 0) || (distance_mm <= 0.0f) ||
        (speed_mm_s <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    motion_engine_begin(engine, MOTION_MODE_STRAIGHT, speed_mm_s);
    engine->target_distance_mm = distance_mm;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_line(motion_engine_t *engine,
    float distance_mm, float speed_mm_s, line_sample_t line)
{
    if ((engine == 0) || (distance_mm <= 0.0f) ||
        (speed_mm_s <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    motion_engine_begin(engine, MOTION_MODE_LINE, speed_mm_s);
    engine->target_distance_mm = distance_mm;
    engine->line = line;
    engine->last_line_error_mm = line.error_mm;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_line_to_end(motion_engine_t *engine,
    float expected_distance_mm, float speed_mm_s, line_sample_t line)
{
    if ((engine == 0) ||
        (expected_distance_mm <= ROBOT_RETURN_ENDPOINT_WINDOW_MM) ||
        (speed_mm_s <= 0.0f) || (engine->calibration == 0) ||
        (engine->calibration->sensor_to_axle_mm < 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    motion_engine_begin(engine, MOTION_MODE_LINE_TO_END, speed_mm_s);
    engine->line_end_stage = MOTION_LINE_END_SEARCH;
    engine->line_end_expected_mm = expected_distance_mm;
    engine->target_distance_mm = expected_distance_mm +
        ROBOT_RETURN_ENDPOINT_WINDOW_MM;
    engine->line = line;
    engine->last_line_error_mm = line.error_mm;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_turn90(
    motion_engine_t *engine, bool turn_right, bool align_to_line)
{
    float gain;

    if ((engine == 0) || (engine->calibration == 0)) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    motion_engine_begin(engine, MOTION_MODE_TURN, ROBOT_TURN_SPEED_MM_S);
    engine->turn_right = turn_right;
    engine->align_to_line = align_to_line;
    gain = turn_right ? engine->calibration->right_turn_gain :
        engine->calibration->left_turn_gain;
    engine->turn_target_mm = MOTION_PI *
        engine->calibration->effective_track_mm * 0.25f * gain;
    engine->target_distance_mm = engine->turn_target_mm;
    return ML_STATUS_OK;
}

ml_status_t motion_engine_start_circle(
    motion_engine_t *engine, uint16_t radius_mm)
{
    float left_gain = 1.0f;
    float right_gain = 1.0f;
    float half_track;

    if ((engine == 0) || (radius_mm < 300U) ||
        (radius_mm > 600U) || (engine->calibration == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    half_track = engine->calibration->effective_track_mm * 0.5f;
    if ((float) radius_mm <= half_track) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (engine->result == MOTION_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    robot_circle_gains(radius_mm, &left_gain, &right_gain);
    motion_engine_begin(engine, MOTION_MODE_CIRCLE,
        ROBOT_CIRCLE_SPEED_MM_S);
    engine->circle_left_target_mm = 2.0f * MOTION_PI *
        ((float) radius_mm + half_track) * left_gain;
    engine->circle_right_target_mm = 2.0f * MOTION_PI *
        ((float) radius_mm - half_track) * right_gain;
    engine->target_distance_mm = (engine->circle_left_target_mm +
        engine->circle_right_target_mm) * 0.5f;
    return ML_STATUS_OK;
}

void motion_engine_line_tick(
    motion_engine_t *engine, line_sample_t line)
{
    float progress;

    if (engine == 0) {
        return;
    }
    ++engine->uptime_ticks;
    engine->line = line;
    if (engine->mode == MOTION_MODE_FIND_A) {
        if (engine->find_stage == MOTION_FIND_A_SEARCH) {
            if (engine->line.transverse) {
                if (++engine->transverse_samples >=
                    MOTION_TRANSVERSE_SAMPLES) {
                    engine->line_entry_mm =
                        motion_engine_mean_distance(engine);
                    engine->find_stage = MOTION_FIND_A_ON_LINE;
                    engine->clear_samples = 0U;
                }
            } else {
                engine->transverse_samples = 0U;
            }
        } else if (engine->find_stage == MOTION_FIND_A_ON_LINE) {
            if (!engine->line.transverse) {
                if (++engine->clear_samples >=
                    MOTION_TRANSVERSE_SAMPLES) {
                    engine->line_exit_mm =
                        motion_engine_mean_distance(engine);
                    engine->target_distance_mm =
                        ((engine->line_entry_mm + engine->line_exit_mm) *
                         0.5f) + engine->calibration->sensor_to_axle_mm;
                    engine->find_stage = MOTION_FIND_A_TO_AXLE;
                }
            } else {
                engine->clear_samples = 0U;
            }
        }
    }

    if (engine->mode == MOTION_MODE_LINE) {
        if (engine->line.lost) {
            if (++engine->line_lost_ticks >=
                MOTION_LINE_LOST_LIMIT_TICKS) {
                motion_engine_fail(engine, MOTION_FAULT_LINE_LOST);
            }
        } else {
            engine->line_lost_ticks = 0U;
        }
    }

    if ((engine->mode == MOTION_MODE_LINE_TO_END) &&
        (engine->line_end_stage == MOTION_LINE_END_SEARCH)) {
        progress = motion_engine_mean_distance(engine);
        if (engine->line.lost &&
            (progress >= (engine->line_end_expected_mm -
             ROBOT_RETURN_ENDPOINT_WINDOW_MM))) {
            if (++engine->line_end_samples >= MOTION_LINE_END_SAMPLES) {
                engine->line_end_stage = MOTION_LINE_END_TO_AXLE;
                engine->target_distance_mm = progress +
                    engine->calibration->sensor_to_axle_mm;
                engine->line_lost_ticks = 0U;
            }
        } else {
            engine->line_end_samples = 0U;
            if (engine->line.lost) {
                if (++engine->line_lost_ticks >=
                    MOTION_LINE_LOST_LIMIT_TICKS) {
                    motion_engine_fail(engine, MOTION_FAULT_LINE_LOST);
                }
            } else {
                engine->line_lost_ticks = 0U;
            }
        }
        if ((engine->result == MOTION_RESULT_RUNNING) &&
            (engine->line_end_stage == MOTION_LINE_END_SEARCH) &&
            (progress >= (engine->line_end_expected_mm +
             ROBOT_RETURN_ENDPOINT_WINDOW_MM))) {
            motion_engine_fail(engine, MOTION_FAULT_LINE_NOT_FOUND);
        }
    }

    if ((engine->mode == MOTION_MODE_TURN) && engine->align_to_line) {
        progress = motion_engine_mean_absolute_distance(engine);
        if ((progress >= (engine->turn_target_mm * 0.65f)) &&
            engine->line.centered) {
            if (engine->line_align_samples < UINT8_MAX) {
                ++engine->line_align_samples;
            }
        } else {
            engine->line_align_samples = 0U;
        }
    }
}

void motion_engine_velocity_tick(motion_engine_t *engine,
    int32_t count_a, int32_t count_b, motion_engine_output_t *output)
{
    int32_t left_delta;
    int32_t right_delta;
    float left_speed = 0.0f;
    float right_speed = 0.0f;

    if ((engine == 0) || (output == 0) ||
        (engine->calibration == 0)) {
        return;
    }
    output->apply_speeds = false;
    output->left_speed_mm_s = 0.0f;
    output->right_speed_mm_s = 0.0f;
    left_delta = count_a *
        (int32_t) engine->calibration->left_encoder_sign;
    right_delta = count_b *
        (int32_t) engine->calibration->right_encoder_sign;
    output->left_delta_ticks = left_delta;
    output->right_delta_ticks = right_delta;
    engine->left_distance_mm += (float) left_delta *
        engine->calibration->left_mm_per_tick;
    engine->right_distance_mm += (float) right_delta *
        engine->calibration->right_mm_per_tick;

    if (engine->result != MOTION_RESULT_RUNNING) {
        return;
    }
    if ((engine->mode == MOTION_MODE_FIND_A) &&
        (engine->find_stage != MOTION_FIND_A_TO_AXLE) &&
        (motion_engine_mean_distance(engine) > ROBOT_MAX_APPROACH_MM)) {
        motion_engine_fail(engine, MOTION_FAULT_LINE_NOT_FOUND);
        return;
    }
    if (motion_engine_check_completion(engine)) {
        return;
    }

    if ((engine->mode == MOTION_MODE_FIND_A) ||
        (engine->mode == MOTION_MODE_STRAIGHT)) {
        motion_engine_compute_straight_speeds(
            engine, &left_speed, &right_speed, false);
    } else if (engine->mode == MOTION_MODE_LINE) {
        motion_engine_compute_straight_speeds(
            engine, &left_speed, &right_speed, true);
    } else if (engine->mode == MOTION_MODE_LINE_TO_END) {
        motion_engine_compute_straight_speeds(engine,
            &left_speed, &right_speed,
            engine->line_end_stage == MOTION_LINE_END_SEARCH);
    } else if (engine->mode == MOTION_MODE_TURN) {
        motion_engine_compute_turn_speeds(
            engine, &left_speed, &right_speed);
    } else if (engine->mode == MOTION_MODE_CIRCLE) {
        motion_engine_compute_circle_speeds(
            engine, &left_speed, &right_speed);
        if (engine->result != MOTION_RESULT_RUNNING) {
            return;
        }
    }

    motion_engine_update_stall(
        engine, left_speed, right_speed, left_delta, right_delta);
    if (engine->result == MOTION_RESULT_RUNNING) {
        output->left_speed_mm_s = left_speed;
        output->right_speed_mm_s = right_speed;
        output->apply_speeds = true;
    }
}

void motion_engine_stop(motion_engine_t *engine)
{
    if (engine == 0) {
        return;
    }
    engine->mode = MOTION_MODE_IDLE;
    engine->result = MOTION_RESULT_CANCELLED;
    engine->fault = MOTION_FAULT_NONE;
    engine->stop_requested = true;
}

bool motion_engine_take_stop_request(motion_engine_t *engine)
{
    bool requested;

    if (engine == 0) {
        return false;
    }
    requested = engine->stop_requested;
    engine->stop_requested = false;
    return requested;
}

motion_status_t motion_engine_get_status(const motion_engine_t *engine)
{
    motion_status_t result;

    result.mode = MOTION_MODE_IDLE;
    result.result = MOTION_RESULT_IDLE;
    result.fault = MOTION_FAULT_NONE;
    result.left_distance_mm = 0.0f;
    result.right_distance_mm = 0.0f;
    result.mean_distance_mm = 0.0f;
    result.target_distance_mm = 0.0f;
    result.line.raw_bits = 0U;
    result.line.black_bits = 0U;
    result.line.black_count = 0U;
    result.line.error_mm = 0.0f;
    result.line.centered = false;
    result.line.transverse = false;
    result.line.lost = true;
    result.uptime_ticks = 0U;
    if (engine == 0) {
        return result;
    }
    result.mode = engine->mode;
    result.result = engine->result;
    result.fault = engine->fault;
    result.left_distance_mm = engine->left_distance_mm;
    result.right_distance_mm = engine->right_distance_mm;
    result.mean_distance_mm = motion_engine_mean_distance(engine);
    result.target_distance_mm = engine->target_distance_mm;
    result.line = engine->line;
    result.uptime_ticks = engine->uptime_ticks;
    return result;
}

uint32_t motion_engine_get_uptime_ticks(const motion_engine_t *engine)
{
    return (engine == 0) ? 0U : engine->uptime_ticks;
}
