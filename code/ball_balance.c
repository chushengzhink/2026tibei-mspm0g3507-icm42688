#include "ball_balance.h"

#include <float.h>
#include <string.h>

#include "ball_balance_config.h"
#include "maix_ball_protocol.h"
#include "ml_tim.h"
#include "ml_uart.h"
#include "rds3230.h"

typedef struct {
    rds3230_t servo;
    maix_ball_parser_t parser;
    ball_balance_state_t state;
    ball_balance_sequence_state_t sequence_state;
    ball_balance_control_mode_t control_mode;
    bool initialized;
    bool enabled;
    bool vision_ready;
    bool capture_time_initialized;
    bool report_time_initialized;
    bool has_received_valid_frame;
    bool sequence_settle_active;
    bool sequence_started_once;
    uint8_t consecutive_valid_frames;
    uint32_t last_capture_ms;
    uint32_t last_report_capture_ms;
    uint32_t vision_frame_interval_ms;
    uint32_t last_valid_receive_ms;
    uint32_t last_control_ms;
    uint32_t sequence_start_ms;
    uint32_t sequence_elapsed_ms;
    uint32_t sequence_settle_start_ms;
    uint32_t observer_outliers;
    int16_t raw_center_x_px;
    int16_t raw_center_y_px;
    int16_t manual_servo_offset_us;
    float raw_score;
    float target_cm;
    float position_cm;
    float velocity_cm_per_s;
    float error_cm;
    float integral_cm_s;
    float target_velocity_cm_per_s;
    float speed_error_cm_per_s;
    float control_output_us;
    float previous_velocity_cm_per_s;
    float filtered_acceleration_cm_per_s2;
} ball_balance_context_t;

static ball_balance_context_t g_ball;
static volatile uint32_t g_ball_time_ms;

static float ball_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ball_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static bool ball_float_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static uint32_t ball_now_ms(void)
{
    return g_ball_time_ms;
}

static bool ball_sequence_is_running(void)
{
    return (g_ball.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) ||
        (g_ball.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
}

static void ball_tick_1ms(void *context)
{
    (void) context;
    ++g_ball_time_ms;
}

static void ball_reset_pid_state(void)
{
    g_ball.error_cm = g_ball.target_cm - g_ball.position_cm;
    g_ball.integral_cm_s = 0.0f;
    g_ball.target_velocity_cm_per_s = 0.0f;
    g_ball.speed_error_cm_per_s = -g_ball.velocity_cm_per_s;
    g_ball.control_output_us = 0.0f;
    g_ball.previous_velocity_cm_per_s = g_ball.velocity_cm_per_s;
    g_ball.filtered_acceleration_cm_per_s2 = 0.0f;
    g_ball.last_control_ms = ball_now_ms();
}

static void ball_recenter(void)
{
    ball_reset_pid_state();
    (void) rds3230_set_center(&g_ball.servo);
}

static void ball_reset_controller(void)
{
    int32_t pulse_us;

    ball_reset_pid_state();
    if (!g_ball.enabled && (g_ball.manual_servo_offset_us != 0)) {
        pulse_us = (int32_t) BALL_SERVO_CENTER_US +
            g_ball.manual_servo_offset_us;
        (void) rds3230_set_target_us(
            &g_ball.servo, (uint16_t) pulse_us);
    } else {
        (void) rds3230_set_center(&g_ball.servo);
    }
}

static void ball_invalidate_vision(void)
{
    g_ball.vision_ready = false;
    g_ball.capture_time_initialized = false;
    g_ball.consecutive_valid_frames = 0U;
    g_ball.velocity_cm_per_s = 0.0f;
    ball_reset_pid_state();
}

static void ball_set_servo_from_control(float control_us)
{
    float pulse = (float) BALL_SERVO_CENTER_US +
        (BALL_CONTROL_DIRECTION * control_us);
    uint16_t pulse_us;

    pulse = ball_clamp(pulse, (float) BALL_SERVO_MINIMUM_US,
        (float) BALL_SERVO_MAXIMUM_US);
    pulse_us = (uint16_t) (pulse + 0.5f);
    (void) rds3230_set_target_us(&g_ball.servo, pulse_us);
}

static void ball_run_controller(void)
{
    float candidate_integral;
    float candidate_velocity;
    float acceleration;
    float target_velocity;
    float speed_error;
    float candidate_control;
    bool outer_saturated;
    bool inner_saturated;

    g_ball.error_cm = g_ball.target_cm - g_ball.position_cm;
    acceleration = (g_ball.velocity_cm_per_s -
        g_ball.previous_velocity_cm_per_s) /
        ((float) BALL_CONTROL_PERIOD_MS / 1000.0f);
    g_ball.previous_velocity_cm_per_s = g_ball.velocity_cm_per_s;
    g_ball.filtered_acceleration_cm_per_s2 +=
        BALL_SPEED_D_FILTER_ALPHA *
        (acceleration - g_ball.filtered_acceleration_cm_per_s2);

    if (g_ball.control_mode == BALL_CONTROL_SPEED_TEST) {
        g_ball.integral_cm_s = 0.0f;
        target_velocity = 0.0f;
    } else {
        if ((BALL_POSITION_KI_PER_S2 <= 0.0f) ||
            (ball_abs(g_ball.error_cm) >
             BALL_POSITION_INTEGRAL_SEPARATION_CM)) {
            candidate_integral = 0.0f;
            g_ball.integral_cm_s = 0.0f;
        } else {
            candidate_integral = ball_clamp(
                g_ball.integral_cm_s +
                (g_ball.error_cm *
                 ((float) BALL_CONTROL_PERIOD_MS / 1000.0f)),
                -BALL_POSITION_INTEGRAL_LIMIT_CM_S,
                BALL_POSITION_INTEGRAL_LIMIT_CM_S);
        }

        candidate_velocity =
            (BALL_POSITION_KP_PER_S * g_ball.error_cm) -
            (BALL_POSITION_KD * g_ball.velocity_cm_per_s) +
            (BALL_POSITION_KI_PER_S2 * candidate_integral);
        target_velocity = ball_clamp(candidate_velocity,
            -BALL_SPEED_REFERENCE_LIMIT_CM_PER_S,
            BALL_SPEED_REFERENCE_LIMIT_CM_PER_S);
        speed_error = target_velocity - g_ball.velocity_cm_per_s;
        candidate_control =
            (BALL_SPEED_KP_US_PER_CM_PER_S * speed_error) -
            (BALL_SPEED_KD_US_PER_CM_PER_S2 *
             g_ball.filtered_acceleration_cm_per_s2);
        outer_saturated =
            ((candidate_velocity > BALL_SPEED_REFERENCE_LIMIT_CM_PER_S) &&
             (g_ball.error_cm > 0.0f)) ||
            ((candidate_velocity < -BALL_SPEED_REFERENCE_LIMIT_CM_PER_S) &&
             (g_ball.error_cm < 0.0f));
        inner_saturated =
            ((candidate_control > BALL_CONTROL_LIMIT_US) &&
             (g_ball.error_cm > 0.0f)) ||
            ((candidate_control < -BALL_CONTROL_LIMIT_US) &&
             (g_ball.error_cm < 0.0f));
        if ((ball_abs(g_ball.error_cm) <=
             BALL_POSITION_INTEGRAL_SEPARATION_CM) &&
            !outer_saturated && !inner_saturated) {
            g_ball.integral_cm_s = candidate_integral;
        }

        candidate_velocity =
            (BALL_POSITION_KP_PER_S * g_ball.error_cm) -
            (BALL_POSITION_KD * g_ball.velocity_cm_per_s) +
            (BALL_POSITION_KI_PER_S2 * g_ball.integral_cm_s);
        target_velocity = ball_clamp(candidate_velocity,
            -BALL_SPEED_REFERENCE_LIMIT_CM_PER_S,
            BALL_SPEED_REFERENCE_LIMIT_CM_PER_S);
    }

    g_ball.target_velocity_cm_per_s = target_velocity;
    g_ball.speed_error_cm_per_s =
        target_velocity - g_ball.velocity_cm_per_s;
    candidate_control =
        (BALL_SPEED_KP_US_PER_CM_PER_S *
         g_ball.speed_error_cm_per_s) -
        (BALL_SPEED_KD_US_PER_CM_PER_S2 *
         g_ball.filtered_acceleration_cm_per_s2);
    g_ball.control_output_us = ball_clamp(candidate_control,
        -BALL_CONTROL_LIMIT_US, BALL_CONTROL_LIMIT_US);
    ball_set_servo_from_control(g_ball.control_output_us);
}

static void ball_stop_sequence(ball_balance_sequence_state_t reason,
    ball_balance_state_t state, uint32_t now_ms)
{
    if (ball_sequence_is_running()) {
        g_ball.sequence_elapsed_ms = now_ms - g_ball.sequence_start_ms;
    }
    g_ball.sequence_state = reason;
    g_ball.sequence_settle_active = false;
    g_ball.enabled = false;
    g_ball.control_mode = BALL_CONTROL_DISABLED;
    g_ball.manual_servo_offset_us = 0;
    g_ball.state = state;
    ball_recenter();
}

static void ball_mark_vision_lost(uint32_t now_ms)
{
    bool was_enabled = g_ball.enabled;
    bool sequence_running = ball_sequence_is_running();

    ball_invalidate_vision();
    if (sequence_running) {
        ball_stop_sequence(
            BALL_SEQUENCE_VISION_LOST, BALL_BALANCE_VISION_LOST, now_ms);
    } else if (was_enabled) {
        g_ball.enabled = false;
        g_ball.control_mode = BALL_CONTROL_DISABLED;
        g_ball.manual_servo_offset_us = 0;
        g_ball.state = BALL_BALANCE_VISION_LOST;
        ball_recenter();
    } else if (g_ball.manual_servo_offset_us != 0) {
        g_ball.state = BALL_BALANCE_MANUAL_SERVO;
    } else {
        g_ball.state = BALL_BALANCE_DISABLED;
    }
}

static void ball_handle_measurement(
    const maix_ball_measurement_t *measurement, uint32_t receive_ms)
{
    uint32_t capture_interval_ms;
    float measured_position_cm;
    float dt_s;
    float predicted_position;
    float residual;

    g_ball.raw_center_x_px = measurement->center_x_px;
    g_ball.raw_center_y_px = measurement->center_y_px;
    g_ball.raw_score = measurement->score;
    if (g_ball.report_time_initialized) {
        g_ball.vision_frame_interval_ms =
            measurement->capture_ms - g_ball.last_report_capture_ms;
    } else {
        g_ball.vision_frame_interval_ms = 0U;
        g_ball.report_time_initialized = true;
    }
    g_ball.last_report_capture_ms = measurement->capture_ms;

    if (!measurement->valid) {
        ball_mark_vision_lost(receive_ms);
        return;
    }
    measured_position_cm = measurement->position_cm;
    if ((measurement->score < BALL_MINIMUM_SCORE) ||
        !ball_float_is_finite(measured_position_cm) ||
        (measured_position_cm < BALL_MEASUREMENT_MINIMUM_CM) ||
        (measured_position_cm > BALL_MEASUREMENT_MAXIMUM_CM)) {
        ball_mark_vision_lost(receive_ms);
        return;
    }

    if (!g_ball.capture_time_initialized) {
        g_ball.last_capture_ms = measurement->capture_ms;
        g_ball.capture_time_initialized = true;
        g_ball.position_cm = measured_position_cm;
        g_ball.velocity_cm_per_s = 0.0f;
        g_ball.consecutive_valid_frames = 1U;
        g_ball.last_valid_receive_ms = receive_ms;
        g_ball.has_received_valid_frame = true;
        if (g_ball.enabled) {
            g_ball.state = BALL_BALANCE_WAITING_FOR_VISION;
        } else if (g_ball.manual_servo_offset_us != 0) {
            g_ball.state = BALL_BALANCE_MANUAL_SERVO;
        } else {
            g_ball.state = BALL_BALANCE_DISABLED;
        }
        return;
    }

    capture_interval_ms = measurement->capture_ms - g_ball.last_capture_ms;
    if (capture_interval_ms == 0U) {
        return;
    }
    if (capture_interval_ms > BALL_MAX_CAPTURE_INTERVAL_MS) {
        ball_invalidate_vision();
        g_ball.last_capture_ms = measurement->capture_ms;
        g_ball.capture_time_initialized = true;
        g_ball.position_cm = measured_position_cm;
        g_ball.consecutive_valid_frames = 1U;
        g_ball.last_valid_receive_ms = receive_ms;
        g_ball.has_received_valid_frame = true;
        return;
    }

    dt_s = (float) capture_interval_ms / 1000.0f;
    predicted_position = g_ball.position_cm +
        (g_ball.velocity_cm_per_s * dt_s);
    residual = measured_position_cm - predicted_position;
    if (ball_abs(residual) > BALL_MAX_OBSERVER_RESIDUAL_CM) {
        ++g_ball.observer_outliers;
        return;
    }

    g_ball.position_cm = predicted_position +
        (BALL_OBSERVER_ALPHA * residual);
    g_ball.velocity_cm_per_s +=
        (BALL_OBSERVER_BETA * residual) / dt_s;
    g_ball.last_capture_ms = measurement->capture_ms;
    g_ball.last_valid_receive_ms = receive_ms;
    g_ball.has_received_valid_frame = true;

    if (g_ball.consecutive_valid_frames < BALL_REACQUIRE_FRAME_COUNT) {
        ++g_ball.consecutive_valid_frames;
    }
    if (!g_ball.vision_ready &&
        (g_ball.consecutive_valid_frames >= BALL_REACQUIRE_FRAME_COUNT)) {
        g_ball.vision_ready = true;
        ball_reset_controller();
    }

    if (g_ball.enabled && g_ball.vision_ready) {
        g_ball.state = BALL_BALANCE_ACTIVE;
    }
}

static void ball_set_internal_target(float target_cm)
{
    g_ball.target_cm = target_cm;
    ball_reset_controller();
}

static void ball_update_sequence(uint32_t now_ms)
{
    float allowed_error;
    uint32_t settle_time_ms;

    if (!ball_sequence_is_running()) {
        return;
    }
    g_ball.sequence_elapsed_ms = now_ms - g_ball.sequence_start_ms;
    if (g_ball.sequence_elapsed_ms >= BALL_SEQUENCE_TIMEOUT_MS) {
        ball_stop_sequence(
            BALL_SEQUENCE_TIMEOUT, BALL_BALANCE_DISABLED, now_ms);
        return;
    }
    if (!g_ball.vision_ready ||
        (g_ball.state != BALL_BALANCE_ACTIVE)) {
        g_ball.sequence_settle_active = false;
        return;
    }

    if (g_ball.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) {
        allowed_error = BALL_SEQUENCE_PLUS_ERROR_CM;
        settle_time_ms = BALL_SEQUENCE_PLUS_SETTLE_MS;
    } else {
        allowed_error = BALL_SEQUENCE_FINAL_ERROR_CM;
        settle_time_ms = BALL_SEQUENCE_FINAL_SETTLE_MS;
    }

    if (ball_abs(g_ball.error_cm) > allowed_error) {
        g_ball.sequence_settle_active = false;
        return;
    }
    if (!g_ball.sequence_settle_active) {
        g_ball.sequence_settle_active = true;
        g_ball.sequence_settle_start_ms = now_ms;
        return;
    }
    if ((now_ms - g_ball.sequence_settle_start_ms) < settle_time_ms) {
        return;
    }

    g_ball.sequence_settle_active = false;
    if (g_ball.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) {
        g_ball.sequence_state = BALL_SEQUENCE_TO_MINUS_5_CM;
        ball_set_internal_target(BALL_SEQUENCE_MINUS_CM);
    } else {
        g_ball.sequence_state = BALL_SEQUENCE_COMPLETE;
        g_ball.sequence_elapsed_ms = now_ms - g_ball.sequence_start_ms;
    }
}

ml_status_t ball_balance_init(void)
{
    ml_status_t status;

    memset(&g_ball, 0, sizeof(g_ball));
    g_ball_time_ms = 0U;
    maix_ball_parser_init(&g_ball.parser);

    status = uart_init(BALL_VISION_UART, BALL_VISION_UART_BAUD,
        BALL_VISION_UART_PRIORITY);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = rds3230_init(&g_ball.servo,
        BALL_SERVO_TIMER, BALL_SERVO_CHANNEL, BALL_SERVO_FREQUENCY_HZ,
        BALL_SERVO_MINIMUM_US, BALL_SERVO_CENTER_US,
        BALL_SERVO_MAXIMUM_US, BALL_SERVO_MAX_SLEW_US_PER_S);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = tim_interrupt_ms_init_ex(BALL_TIMEBASE_TIMER, 1U,
        BALL_TIMEBASE_PRIORITY, ball_tick_1ms, 0);
    if (status != ML_STATUS_OK) {
        return status;
    }

    g_ball.target_cm = 0.0f;
    g_ball.state = BALL_BALANCE_DISABLED;
    g_ball.sequence_state = BALL_SEQUENCE_IDLE;
    g_ball.control_mode = BALL_CONTROL_DISABLED;
    g_ball.initialized = true;
    return ML_STATUS_OK;
}

void ball_balance_process(void)
{
    uint8_t byte;
    uint32_t now_ms;
    maix_ball_measurement_t measurement;

    if (!g_ball.initialized) {
        return;
    }

    now_ms = ball_now_ms();
    while (uart_try_read(BALL_VISION_UART, &byte) == ML_STATUS_OK) {
        if (maix_ball_parser_push(&g_ball.parser, byte, &measurement)) {
            ball_handle_measurement(&measurement, now_ms);
        }
    }

    if (g_ball.enabled && g_ball.vision_ready &&
        ((now_ms - g_ball.last_valid_receive_ms) >
         BALL_VISION_TIMEOUT_MS)) {
        ball_mark_vision_lost(now_ms);
    }
    if (g_ball.enabled && g_ball.vision_ready &&
        ((now_ms - g_ball.last_control_ms) >= BALL_CONTROL_PERIOD_MS)) {
        g_ball.last_control_ms += BALL_CONTROL_PERIOD_MS;
        ball_run_controller();
    }
    ball_update_sequence(now_ms);
    (void) rds3230_update(&g_ball.servo, now_ms);
}

ml_status_t ball_balance_enable(bool enable)
{
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!enable && ball_sequence_is_running()) {
        return ball_balance_abort_sequence();
    }
    if ((enable == g_ball.enabled) &&
        (!enable ||
         (g_ball.control_mode == BALL_CONTROL_CASCADE))) {
        if (!enable) {
            g_ball.manual_servo_offset_us = 0;
            g_ball.control_mode = BALL_CONTROL_DISABLED;
            g_ball.state = BALL_BALANCE_DISABLED;
            ball_recenter();
        }
        return ML_STATUS_OK;
    }

    g_ball.enabled = enable;
    g_ball.control_mode = enable ?
        BALL_CONTROL_CASCADE : BALL_CONTROL_DISABLED;
    g_ball.manual_servo_offset_us = 0;
    ball_reset_controller();
    if (enable) {
        g_ball.state = g_ball.vision_ready ?
            BALL_BALANCE_ACTIVE : BALL_BALANCE_WAITING_FOR_VISION;
    } else {
        g_ball.state = BALL_BALANCE_DISABLED;
        g_ball.sequence_settle_active = false;
    }
    return ML_STATUS_OK;
}

ml_status_t ball_balance_enable_speed_test(bool enable)
{
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!enable) {
        return ball_balance_enable(false);
    }
    if (!g_ball.vision_ready || ball_sequence_is_running()) {
        return ML_STATUS_BUSY;
    }

    g_ball.enabled = true;
    g_ball.control_mode = BALL_CONTROL_SPEED_TEST;
    g_ball.manual_servo_offset_us = 0;
    g_ball.state = BALL_BALANCE_ACTIVE;
    ball_reset_controller();
    return ML_STATUS_OK;
}

ml_status_t ball_balance_set_manual_servo_offset_us(int16_t offset_us)
{
    int32_t pulse_us;

    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (g_ball.enabled || ball_sequence_is_running()) {
        return ML_STATUS_BUSY;
    }
    if ((offset_us < -BALL_MANUAL_MAX_OFFSET_US) ||
        (offset_us > BALL_MANUAL_MAX_OFFSET_US)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    pulse_us = (int32_t) BALL_SERVO_CENTER_US + offset_us;
    if ((pulse_us < (int32_t) BALL_SERVO_MINIMUM_US) ||
        (pulse_us > (int32_t) BALL_SERVO_MAXIMUM_US)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    g_ball.manual_servo_offset_us = offset_us;
    g_ball.control_mode = BALL_CONTROL_DISABLED;
    g_ball.sequence_settle_active = false;
    ball_reset_pid_state();
    g_ball.state = (offset_us == 0) ?
        BALL_BALANCE_DISABLED : BALL_BALANCE_MANUAL_SERVO;
    return rds3230_set_target_us(&g_ball.servo, (uint16_t) pulse_us);
}

ml_status_t ball_balance_set_target_cm(float target_cm)
{
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (ball_sequence_is_running()) {
        return ML_STATUS_BUSY;
    }
    if (!ball_float_is_finite(target_cm) ||
        (target_cm < BALL_TARGET_MINIMUM_CM) ||
        (target_cm > BALL_TARGET_MAXIMUM_CM)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    g_ball.sequence_state = BALL_SEQUENCE_IDLE;
    g_ball.sequence_settle_active = false;
    ball_set_internal_target(target_cm);
    return ML_STATUS_OK;
}

ml_status_t ball_balance_start_pm5_sequence(void)
{
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
#if !BALL_BALANCE_ALLOW_SEQUENCE
    return ML_STATUS_UNSUPPORTED;
#else
    if (!g_ball.vision_ready || ball_sequence_is_running() ||
        g_ball.sequence_started_once) {
        return ML_STATUS_BUSY;
    }

    g_ball.enabled = true;
    g_ball.control_mode = BALL_CONTROL_CASCADE;
    g_ball.manual_servo_offset_us = 0;
    g_ball.state = BALL_BALANCE_ACTIVE;
    g_ball.sequence_state = BALL_SEQUENCE_TO_PLUS_5_CM;
    g_ball.sequence_started_once = true;
    g_ball.sequence_start_ms = ball_now_ms();
    g_ball.sequence_elapsed_ms = 0U;
    g_ball.sequence_settle_active = false;
    ball_set_internal_target(BALL_SEQUENCE_PLUS_CM);
    return ML_STATUS_OK;
#endif
}

ml_status_t ball_balance_abort_sequence(void)
{
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!ball_sequence_is_running()) {
        return ML_STATUS_BUSY;
    }
    ball_stop_sequence(
        BALL_SEQUENCE_ABORTED, BALL_BALANCE_DISABLED, ball_now_ms());
    return ML_STATUS_OK;
}

ml_status_t ball_balance_get_status(ball_balance_status_t *status)
{
    uint32_t now_ms;

    if (status == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_ball.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    now_ms = ball_now_ms();
    status->state = g_ball.state;
    status->sequence_state = g_ball.sequence_state;
    status->control_mode = g_ball.control_mode;
    status->enabled = g_ball.enabled;
    status->vision_ready = g_ball.vision_ready;
    status->sequence_started_once = g_ball.sequence_started_once;
    status->target_cm = g_ball.target_cm;
    status->position_cm = g_ball.position_cm;
    status->velocity_cm_per_s = g_ball.velocity_cm_per_s;
    status->error_cm = g_ball.error_cm;
    status->integral_cm_s = g_ball.integral_cm_s;
    status->target_velocity_cm_per_s =
        g_ball.target_velocity_cm_per_s;
    status->speed_error_cm_per_s = g_ball.speed_error_cm_per_s;
    status->control_output_us = g_ball.control_output_us;
    status->raw_center_x_px = g_ball.raw_center_x_px;
    status->raw_center_y_px = g_ball.raw_center_y_px;
    status->raw_score = g_ball.raw_score;
    status->manual_servo_offset_us = g_ball.manual_servo_offset_us;
    status->servo_current_us = rds3230_get_current_us(&g_ball.servo);
    status->servo_target_us = rds3230_get_target_us(&g_ball.servo);
    status->uptime_ms = now_ms;
    status->sequence_elapsed_ms = g_ball.sequence_elapsed_ms;
    status->vision_frame_interval_ms = g_ball.vision_frame_interval_ms;
    status->vision_age_ms = g_ball.has_received_valid_frame ?
        (now_ms - g_ball.last_valid_receive_ms) : 0xFFFFFFFFUL;
    status->valid_frames = g_ball.parser.frames_ok;
    status->crc_errors = g_ball.parser.crc_errors;
    status->length_errors = g_ball.parser.length_errors;
    status->format_errors = g_ball.parser.format_errors;
    status->observer_outliers = g_ball.observer_outliers;
    status->uart_overflows =
        uart_get_rx_overflow_count(BALL_VISION_UART);
    return ML_STATUS_OK;
}
