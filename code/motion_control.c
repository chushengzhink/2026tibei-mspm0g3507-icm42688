#include "motion_control.h"

#include "ml_motor.h"
#include "ml_tim.h"
#include "pid.h"

#define MOTION_PI                         (3.14159265f)
#define MOTION_ACCEL_MM_S2                (400.0f)
#define MOTION_MIN_SPEED_MM_S             (45.0f)
#define MOTION_STALL_SPEED_MM_S           (30.0f)
#define MOTION_STALL_LIMIT_UPDATES        (15U)
#define MOTION_LINE_LOST_LIMIT_TICKS      (100U)
#define MOTION_LINE_ALIGN_SAMPLES         (4U)
#define MOTION_TRANSVERSE_SAMPLES         (3U)
#define MOTION_LINE_END_SAMPLES           (3U)
#define MOTION_CIRCLE_SYNC_LIMIT_UPDATES  (25U)

typedef enum {
    FIND_A_SEARCH = 0,
    FIND_A_ON_LINE,
    FIND_A_TO_AXLE
} find_a_stage_t;

typedef enum {
    LINE_END_SEARCH = 0,
    LINE_END_TO_AXLE
} line_end_stage_t;

typedef struct {
    const robot_calibration_t *calibration;
    motion_mode_t mode;
    motion_result_t result;
    motion_fault_t fault;
    volatile uint32_t uptime_ticks;
    uint8_t speed_divider;
    float left_distance_mm;
    float right_distance_mm;
    float target_distance_mm;
    float command_speed_mm_s;
    float current_base_speed_mm_s;
    float last_line_error_mm;
    line_sample_t line;
    uint16_t left_stall_updates;
    uint16_t right_stall_updates;
    uint16_t line_lost_ticks;
    uint16_t circle_sync_updates;
    find_a_stage_t find_stage;
    uint8_t transverse_samples;
    uint8_t clear_samples;
    float line_entry_mm;
    float line_exit_mm;
    bool turn_right;
    bool align_to_line;
    uint8_t line_align_samples;
    line_end_stage_t line_end_stage;
    uint8_t line_end_samples;
    float line_end_expected_mm;
    float turn_target_mm;
    float circle_left_target_mm;
    float circle_right_target_mm;
} motion_state_t;

static motion_state_t g_motion;

static float motion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
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

static float motion_mean_distance(void)
{
    return (g_motion.left_distance_mm + g_motion.right_distance_mm) * 0.5f;
}

static float motion_mean_absolute_distance(void)
{
    return (motion_abs(g_motion.left_distance_mm) +
        motion_abs(g_motion.right_distance_mm)) * 0.5f;
}

static float motion_ramp(float current, float target)
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

static float motion_position_speed(float remaining_mm, float cruise_mm_s)
{
    float speed = cruise_mm_s;

    if (remaining_mm < 40.0f) {
        speed = MOTION_MIN_SPEED_MM_S;
    } else if ((remaining_mm < 120.0f) && (speed > 75.0f)) {
        speed = 75.0f;
    }
    return speed;
}

static void motion_reset_controllers(void)
{
    (void) pid_reset(&motorA);
    (void) pid_reset(&motorB);
}

static void motion_zero_motors(void)
{
    motion_reset_controllers();
    (void) motorA_duty(0);
    (void) motorB_duty(0);
}

static void motion_finish(void)
{
    motion_zero_motors();
    g_motion.current_base_speed_mm_s = 0.0f;
    g_motion.mode = MOTION_MODE_IDLE;
    g_motion.result = MOTION_RESULT_COMPLETE;
}

static void motion_fail(motion_fault_t fault)
{
    motion_zero_motors();
    g_motion.current_base_speed_mm_s = 0.0f;
    g_motion.mode = MOTION_MODE_IDLE;
    g_motion.fault = fault;
    g_motion.result = MOTION_RESULT_FAULT;
}

static void motion_begin(motion_mode_t mode, float speed_mm_s)
{
    g_motion.mode = mode;
    g_motion.result = MOTION_RESULT_RUNNING;
    g_motion.fault = MOTION_FAULT_NONE;
    g_motion.left_distance_mm = 0.0f;
    g_motion.right_distance_mm = 0.0f;
    g_motion.target_distance_mm = 0.0f;
    g_motion.command_speed_mm_s = speed_mm_s;
    g_motion.current_base_speed_mm_s = 0.0f;
    g_motion.last_line_error_mm = 0.0f;
    g_motion.left_stall_updates = 0U;
    g_motion.right_stall_updates = 0U;
    g_motion.line_lost_ticks = 0U;
    g_motion.circle_sync_updates = 0U;
    g_motion.transverse_samples = 0U;
    g_motion.clear_samples = 0U;
    g_motion.line_align_samples = 0U;
    g_motion.line_end_stage = LINE_END_SEARCH;
    g_motion.line_end_samples = 0U;
    g_motion.line_end_expected_mm = 0.0f;
    motion_reset_controllers();
}

static int32_t motion_pwm_from_pid(pid_t *controller, float target_counts,
    float actual_counts, int8_t motor_sign)
{
    float output;

    if (motion_abs(target_counts) < 0.01f) {
        (void) pid_reset(controller);
        return 0;
    }
    controller->target = target_counts;
    controller->now = actual_counts;
    (void) pid_cal(controller);
    output = controller->out + ((target_counts > 0.0f) ?
        ROBOT_MOTOR_FEEDFORWARD : -ROBOT_MOTOR_FEEDFORWARD);
    output = motion_clamp(output,
        -(float) ML_PWM_DUTY_MAX, (float) ML_PWM_DUTY_MAX);
    return (int32_t) (output * (float) motor_sign);
}

static void motion_apply_speeds(float left_mm_s, float right_mm_s,
    int32_t left_delta, int32_t right_delta)
{
    const float period_s = ((float) ROBOT_CONTROL_TICK_MS *
        (float) ROBOT_SPEED_TICK_DIVIDER) / 1000.0f;
    float left_target_counts = left_mm_s * period_s /
        g_motion.calibration->left_mm_per_tick;
    float right_target_counts = right_mm_s * period_s /
        g_motion.calibration->right_mm_per_tick;
    int32_t motor_a_pwm;
    int32_t motor_b_pwm;

    motor_a_pwm = motion_pwm_from_pid(&motorA, left_target_counts,
        (float) left_delta, g_motion.calibration->left_motor_sign);
    motor_b_pwm = motion_pwm_from_pid(&motorB, right_target_counts,
        (float) right_delta, g_motion.calibration->right_motor_sign);
    (void) motorA_duty(motor_a_pwm);
    (void) motorB_duty(motor_b_pwm);
}

static void motion_update_stall(float left_speed, float right_speed,
    int32_t left_delta, int32_t right_delta)
{
    if ((motion_abs(left_speed) >= MOTION_STALL_SPEED_MM_S) &&
        (left_delta == 0)) {
        ++g_motion.left_stall_updates;
    } else {
        g_motion.left_stall_updates = 0U;
    }
    if ((motion_abs(right_speed) >= MOTION_STALL_SPEED_MM_S) &&
        (right_delta == 0)) {
        ++g_motion.right_stall_updates;
    } else {
        g_motion.right_stall_updates = 0U;
    }
    if ((g_motion.left_stall_updates >= MOTION_STALL_LIMIT_UPDATES) ||
        (g_motion.right_stall_updates >= MOTION_STALL_LIMIT_UPDATES)) {
        motion_fail(MOTION_FAULT_ENCODER_STALL);
    }
}

static void motion_compute_straight_speeds(
    float *left_speed, float *right_speed, bool follow_line)
{
    float remaining;
    float desired;
    float correction;
    float wheel_difference;

    if ((g_motion.mode == MOTION_MODE_FIND_A) &&
        (g_motion.find_stage != FIND_A_TO_AXLE)) {
        desired = g_motion.command_speed_mm_s;
    } else {
        remaining = g_motion.target_distance_mm - motion_mean_distance();
        desired = motion_position_speed(remaining, g_motion.command_speed_mm_s);
    }
    if (follow_line && g_motion.line.lost && (desired > 60.0f)) {
        desired = 60.0f;
    }
    g_motion.current_base_speed_mm_s = motion_ramp(
        g_motion.current_base_speed_mm_s, desired);

    if (follow_line) {
        correction = (ROBOT_LINE_KP * g_motion.line.error_mm) +
            (ROBOT_LINE_KD *
             (g_motion.line.error_mm - g_motion.last_line_error_mm));
        correction = motion_clamp(correction,
            -ROBOT_LINE_MAX_CORRECTION_MM_S,
            ROBOT_LINE_MAX_CORRECTION_MM_S);
        g_motion.last_line_error_mm = g_motion.line.error_mm;
        *left_speed = g_motion.current_base_speed_mm_s + correction;
        *right_speed = g_motion.current_base_speed_mm_s - correction;
    } else {
        wheel_difference = g_motion.left_distance_mm -
            g_motion.right_distance_mm;
        correction = motion_clamp(wheel_difference * 2.0f, -20.0f, 20.0f);
        *left_speed = g_motion.current_base_speed_mm_s - correction;
        *right_speed = g_motion.current_base_speed_mm_s + correction;
    }
}

static void motion_compute_turn_speeds(float *left_speed, float *right_speed)
{
    float progress = motion_mean_absolute_distance();
    float desired = g_motion.command_speed_mm_s;

    if (progress > (g_motion.turn_target_mm * 0.70f)) {
        desired = MOTION_MIN_SPEED_MM_S;
    }
    g_motion.current_base_speed_mm_s = motion_ramp(
        g_motion.current_base_speed_mm_s, desired);
    if (g_motion.turn_right) {
        *left_speed = g_motion.current_base_speed_mm_s;
        *right_speed = -g_motion.current_base_speed_mm_s;
    } else {
        *left_speed = -g_motion.current_base_speed_mm_s;
        *right_speed = g_motion.current_base_speed_mm_s;
    }
}

static void motion_compute_circle_speeds(float *left_speed, float *right_speed)
{
    float left_progress = motion_abs(g_motion.left_distance_mm) /
        g_motion.circle_left_target_mm;
    float right_progress = motion_abs(g_motion.right_distance_mm) /
        g_motion.circle_right_target_mm;
    float average_progress = (left_progress + right_progress) * 0.5f;
    float center_distance = (g_motion.circle_left_target_mm +
        g_motion.circle_right_target_mm) * 0.5f;
    float left_ratio = g_motion.circle_left_target_mm / center_distance;
    float right_ratio = g_motion.circle_right_target_mm / center_distance;
    float desired = g_motion.command_speed_mm_s;
    float correction;

    if (average_progress > 0.97f) {
        desired = MOTION_MIN_SPEED_MM_S;
    } else if ((average_progress > 0.85f) && (desired > 75.0f)) {
        desired = 75.0f;
    }
    g_motion.current_base_speed_mm_s = motion_ramp(
        g_motion.current_base_speed_mm_s, desired);
    correction = motion_clamp(
        (left_progress - right_progress) * ROBOT_CIRCLE_SYNC_KP,
        -ROBOT_CIRCLE_MAX_CORRECTION_MM_S,
        ROBOT_CIRCLE_MAX_CORRECTION_MM_S);
    *left_speed = (g_motion.current_base_speed_mm_s - correction) *
        left_ratio;
    *right_speed = (g_motion.current_base_speed_mm_s + correction) *
        right_ratio;

    if ((average_progress > 0.10f) &&
        (motion_abs(left_progress - right_progress) > 0.10f)) {
        ++g_motion.circle_sync_updates;
    } else {
        g_motion.circle_sync_updates = 0U;
    }
    if (g_motion.circle_sync_updates >=
        MOTION_CIRCLE_SYNC_LIMIT_UPDATES) {
        motion_fail(MOTION_FAULT_CIRCLE_SYNC);
    }
}

static bool motion_check_completion(void)
{
    float progress;
    float left_progress;
    float right_progress;

    if ((g_motion.mode == MOTION_MODE_STRAIGHT) ||
        (g_motion.mode == MOTION_MODE_LINE) ||
        ((g_motion.mode == MOTION_MODE_LINE_TO_END) &&
         (g_motion.line_end_stage == LINE_END_TO_AXLE)) ||
        ((g_motion.mode == MOTION_MODE_FIND_A) &&
         (g_motion.find_stage == FIND_A_TO_AXLE))) {
        if (motion_mean_distance() >= g_motion.target_distance_mm) {
            motion_finish();
            return true;
        }
    } else if (g_motion.mode == MOTION_MODE_TURN) {
        progress = motion_mean_absolute_distance();
        if (g_motion.align_to_line) {
            if (g_motion.line_align_samples >= MOTION_LINE_ALIGN_SAMPLES) {
                motion_finish();
                return true;
            }
            if (progress >= (g_motion.turn_target_mm * 1.30f)) {
                motion_fail(MOTION_FAULT_LINE_NOT_FOUND);
                return true;
            }
        } else if (progress >= g_motion.turn_target_mm) {
            motion_finish();
            return true;
        }
    } else if (g_motion.mode == MOTION_MODE_CIRCLE) {
        left_progress = motion_abs(g_motion.left_distance_mm) /
            g_motion.circle_left_target_mm;
        right_progress = motion_abs(g_motion.right_distance_mm) /
            g_motion.circle_right_target_mm;
        if ((left_progress >= 1.000f) && (right_progress >= 1.000f)) {
            motion_finish();
            return true;
        }
    }
    return false;
}

static void motion_speed_update(void)
{
    int32_t count_a;
    int32_t count_b;
    int32_t left_delta;
    int32_t right_delta;
    float left_speed = 0.0f;
    float right_speed = 0.0f;
    ml_status_t status;

    status = encoder_get_and_clear(&count_a, &count_b);
    if (status != ML_STATUS_OK) {
        motion_fail(MOTION_FAULT_ENCODER);
        return;
    }
    left_delta = count_a * (int32_t) g_motion.calibration->left_encoder_sign;
    right_delta = count_b * (int32_t) g_motion.calibration->right_encoder_sign;
    g_motion.left_distance_mm += (float) left_delta *
        g_motion.calibration->left_mm_per_tick;
    g_motion.right_distance_mm += (float) right_delta *
        g_motion.calibration->right_mm_per_tick;

    if (g_motion.result != MOTION_RESULT_RUNNING) {
        return;
    }
    if ((g_motion.mode == MOTION_MODE_FIND_A) &&
        (g_motion.find_stage != FIND_A_TO_AXLE) &&
        (motion_mean_distance() > ROBOT_MAX_APPROACH_MM)) {
        motion_fail(MOTION_FAULT_LINE_NOT_FOUND);
        return;
    }
    if (motion_check_completion()) {
        return;
    }

    if ((g_motion.mode == MOTION_MODE_FIND_A) ||
        (g_motion.mode == MOTION_MODE_STRAIGHT)) {
        motion_compute_straight_speeds(&left_speed, &right_speed, false);
    } else if (g_motion.mode == MOTION_MODE_LINE) {
        motion_compute_straight_speeds(&left_speed, &right_speed, true);
    } else if (g_motion.mode == MOTION_MODE_LINE_TO_END) {
        motion_compute_straight_speeds(&left_speed, &right_speed,
            g_motion.line_end_stage == LINE_END_SEARCH);
    } else if (g_motion.mode == MOTION_MODE_TURN) {
        motion_compute_turn_speeds(&left_speed, &right_speed);
    } else if (g_motion.mode == MOTION_MODE_CIRCLE) {
        motion_compute_circle_speeds(&left_speed, &right_speed);
        if (g_motion.result != MOTION_RESULT_RUNNING) {
            return;
        }
    }

    motion_update_stall(left_speed, right_speed, left_delta, right_delta);
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        motion_apply_speeds(left_speed, right_speed, left_delta, right_delta);
    }
}

static void motion_line_update(void)
{
    float progress;

    g_motion.line = line_sensor_read();
    if (g_motion.mode == MOTION_MODE_FIND_A) {
        if (g_motion.find_stage == FIND_A_SEARCH) {
            if (g_motion.line.transverse) {
                if (++g_motion.transverse_samples >=
                    MOTION_TRANSVERSE_SAMPLES) {
                    g_motion.line_entry_mm = motion_mean_distance();
                    g_motion.find_stage = FIND_A_ON_LINE;
                    g_motion.clear_samples = 0U;
                }
            } else {
                g_motion.transverse_samples = 0U;
            }
        } else if (g_motion.find_stage == FIND_A_ON_LINE) {
            if (!g_motion.line.transverse) {
                if (++g_motion.clear_samples >=
                    MOTION_TRANSVERSE_SAMPLES) {
                    g_motion.line_exit_mm = motion_mean_distance();
                    g_motion.target_distance_mm =
                        ((g_motion.line_entry_mm + g_motion.line_exit_mm) *
                         0.5f) + g_motion.calibration->sensor_to_axle_mm;
                    g_motion.find_stage = FIND_A_TO_AXLE;
                }
            } else {
                g_motion.clear_samples = 0U;
            }
        }
    }

    if (g_motion.mode == MOTION_MODE_LINE) {
        if (g_motion.line.lost) {
            if (++g_motion.line_lost_ticks >=
                MOTION_LINE_LOST_LIMIT_TICKS) {
                motion_fail(MOTION_FAULT_LINE_LOST);
            }
        } else {
            g_motion.line_lost_ticks = 0U;
        }
    }

    if ((g_motion.mode == MOTION_MODE_LINE_TO_END) &&
        (g_motion.line_end_stage == LINE_END_SEARCH)) {
        progress = motion_mean_distance();
        if (g_motion.line.lost &&
            (progress >= (g_motion.line_end_expected_mm -
             ROBOT_RETURN_ENDPOINT_WINDOW_MM))) {
            if (++g_motion.line_end_samples >= MOTION_LINE_END_SAMPLES) {
                g_motion.line_end_stage = LINE_END_TO_AXLE;
                g_motion.target_distance_mm = progress +
                    g_motion.calibration->sensor_to_axle_mm;
                g_motion.line_lost_ticks = 0U;
            }
        } else {
            g_motion.line_end_samples = 0U;
            if (g_motion.line.lost) {
                if (++g_motion.line_lost_ticks >=
                    MOTION_LINE_LOST_LIMIT_TICKS) {
                    motion_fail(MOTION_FAULT_LINE_LOST);
                }
            } else {
                g_motion.line_lost_ticks = 0U;
            }
        }
        if ((g_motion.result == MOTION_RESULT_RUNNING) &&
            (g_motion.line_end_stage == LINE_END_SEARCH) &&
            (progress >= (g_motion.line_end_expected_mm +
             ROBOT_RETURN_ENDPOINT_WINDOW_MM))) {
            motion_fail(MOTION_FAULT_LINE_NOT_FOUND);
        }
    }

    if ((g_motion.mode == MOTION_MODE_TURN) &&
        g_motion.align_to_line) {
        progress = motion_mean_absolute_distance();
        if ((progress >= (g_motion.turn_target_mm * 0.65f)) &&
            g_motion.line.centered) {
            if (g_motion.line_align_samples < UINT8_MAX) {
                ++g_motion.line_align_samples;
            }
        } else {
            g_motion.line_align_samples = 0U;
        }
    }
}

static void motion_timer_callback(void *context)
{
    (void) context;
    ++g_motion.uptime_ticks;
    motion_line_update();
    if (++g_motion.speed_divider >= ROBOT_SPEED_TICK_DIVIDER) {
        g_motion.speed_divider = 0U;
        motion_speed_update();
    }
}

ml_status_t motion_init(const robot_calibration_t *calibration)
{
    ml_status_t status;

    if ((calibration == 0) || (calibration->left_mm_per_tick <= 0.0f) ||
        (calibration->right_mm_per_tick <= 0.0f) ||
        (calibration->effective_track_mm <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    g_motion.calibration = calibration;
    g_motion.mode = MOTION_MODE_IDLE;
    g_motion.result = MOTION_RESULT_IDLE;
    g_motion.fault = MOTION_FAULT_NONE;
    g_motion.uptime_ticks = 0U;
    g_motion.speed_divider = 0U;
    g_motion.line = line_sensor_read();

    status = pid_init(&motorA, POSITION_PID,
        ROBOT_VELOCITY_KP, ROBOT_VELOCITY_KI, ROBOT_VELOCITY_KD);
    if (status == ML_STATUS_OK) {
        status = pid_init(&motorB, POSITION_PID,
            ROBOT_VELOCITY_KP, ROBOT_VELOCITY_KI, ROBOT_VELOCITY_KD);
    }
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(&motorA, -40000.0f, 40000.0f,
            -20000.0f, 20000.0f);
    }
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(&motorB, -40000.0f, 40000.0f,
            -20000.0f, 20000.0f);
    }
    if (status == ML_STATUS_OK) {
        status = tim_interrupt_ms_init_ex(TIMG0, ROBOT_CONTROL_TICK_MS,
            0U, motion_timer_callback, 0);
    }
    return status;
}

ml_status_t motion_start_find_a(float speed_mm_s)
{
    uint32_t interrupt_state;

    if (speed_mm_s <= 0.0f) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_FIND_A, speed_mm_s);
    g_motion.find_stage = FIND_A_SEARCH;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

ml_status_t motion_start_straight(float distance_mm, float speed_mm_s)
{
    uint32_t interrupt_state;

    if ((distance_mm <= 0.0f) || (speed_mm_s <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_STRAIGHT, speed_mm_s);
    g_motion.target_distance_mm = distance_mm;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

ml_status_t motion_start_line(float distance_mm, float speed_mm_s)
{
    uint32_t interrupt_state;

    if ((distance_mm <= 0.0f) || (speed_mm_s <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_LINE, speed_mm_s);
    g_motion.target_distance_mm = distance_mm;
    g_motion.line = line_sensor_read();
    g_motion.last_line_error_mm = g_motion.line.error_mm;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

ml_status_t motion_start_line_to_end(float expected_distance_mm,
    float speed_mm_s)
{
    uint32_t interrupt_state;

    if ((expected_distance_mm <= ROBOT_RETURN_ENDPOINT_WINDOW_MM) ||
        (speed_mm_s <= 0.0f) || (g_motion.calibration == 0) ||
        (g_motion.calibration->sensor_to_axle_mm < 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_LINE_TO_END, speed_mm_s);
    g_motion.line_end_stage = LINE_END_SEARCH;
    g_motion.line_end_expected_mm = expected_distance_mm;
    g_motion.target_distance_mm = expected_distance_mm +
        ROBOT_RETURN_ENDPOINT_WINDOW_MM;
    g_motion.line = line_sensor_read();
    g_motion.last_line_error_mm = g_motion.line.error_mm;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

ml_status_t motion_start_turn90(bool turn_right, bool align_to_line)
{
    uint32_t interrupt_state;
    float gain;

    if (g_motion.calibration == 0) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_TURN, ROBOT_TURN_SPEED_MM_S);
    g_motion.turn_right = turn_right;
    g_motion.align_to_line = align_to_line;
    gain = turn_right ? g_motion.calibration->right_turn_gain :
        g_motion.calibration->left_turn_gain;
    g_motion.turn_target_mm = MOTION_PI *
        g_motion.calibration->effective_track_mm * 0.25f * gain;
    g_motion.target_distance_mm = g_motion.turn_target_mm;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

ml_status_t motion_start_circle(uint16_t radius_mm)
{
    uint32_t interrupt_state;
    float left_gain = 1.0f;
    float right_gain = 1.0f;
    float half_track;

    if ((radius_mm < 300U) || (radius_mm > 600U) ||
        (g_motion.calibration == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    half_track = g_motion.calibration->effective_track_mm * 0.5f;
    if ((float) radius_mm <= half_track) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    robot_circle_gains(radius_mm, &left_gain, &right_gain);

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_motion.result == MOTION_RESULT_RUNNING) {
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        return ML_STATUS_BUSY;
    }
    motion_begin(MOTION_MODE_CIRCLE, ROBOT_CIRCLE_SPEED_MM_S);
    g_motion.circle_left_target_mm = 2.0f * MOTION_PI *
        ((float) radius_mm + half_track) * left_gain;
    g_motion.circle_right_target_mm = 2.0f * MOTION_PI *
        ((float) radius_mm - half_track) * right_gain;
    g_motion.target_distance_mm = (g_motion.circle_left_target_mm +
        g_motion.circle_right_target_mm) * 0.5f;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}

void motion_stop(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    motion_zero_motors();
    g_motion.mode = MOTION_MODE_IDLE;
    g_motion.result = MOTION_RESULT_CANCELLED;
    g_motion.fault = MOTION_FAULT_NONE;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

void motion_emergency_stop(void)
{
    motion_stop();
}

motion_status_t motion_get_status(void)
{
    motion_status_t result;
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    result.mode = g_motion.mode;
    result.result = g_motion.result;
    result.fault = g_motion.fault;
    result.left_distance_mm = g_motion.left_distance_mm;
    result.right_distance_mm = g_motion.right_distance_mm;
    result.mean_distance_mm = motion_mean_distance();
    result.target_distance_mm = g_motion.target_distance_mm;
    result.line = g_motion.line;
    result.uptime_ticks = g_motion.uptime_ticks;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return result;
}

uint32_t motion_get_uptime_ticks(void)
{
    return g_motion.uptime_ticks;
}
