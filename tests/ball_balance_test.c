#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_balance.h"
#include "ball_balance_config.h"
#include "maix_ball_protocol.h"
#include "ml_pwm.h"
#include "ml_tim.h"
#include "ml_uart.h"

UART_Regs g_test_uart2;
GPTIMER_Regs g_test_tima1;
GPTIMER_Regs g_test_timg6;

static uint8_t g_uart_bytes[32768];
static uint32_t g_uart_head;
static uint32_t g_uart_tail;
static tim_callback_t g_tick_callback;
static void *g_tick_context;
static uint32_t g_last_pwm_duty;

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
    data[2] = (uint8_t) (value >> 16U);
    data[3] = (uint8_t) (value >> 24U);
}

static void queue_frame(uint32_t capture_ms, int16_t x, int16_t y,
    float position_cm, float score, uint8_t valid)
{
    uint8_t frame[MAIX_BALL_FRAME_SIZE];
    uint16_t crc;
    uint32_t i;

    memset(frame, 0, sizeof(frame));
    frame[0] = 0xAAU;
    frame[1] = 0xCAU;
    frame[2] = 0xACU;
    frame[3] = 0xBBU;
    write_le32(&frame[4], MAIX_BALL_DATA_SIZE);
    frame[8] = 0xA1U;
    frame[9] = 0x02U;
    write_le32(&frame[10], capture_ms);
    write_le16(&frame[14], (uint16_t) x);
    write_le16(&frame[16], (uint16_t) y);
    memcpy(&frame[18], &position_cm, sizeof(position_cm));
    memcpy(&frame[22], &score, sizeof(score));
    frame[26] = valid;
    crc = maix_crc16_ibm(frame, 30U);
    write_le16(&frame[30], crc);

    assert((g_uart_tail + sizeof(frame)) <= sizeof(g_uart_bytes));
    for (i = 0U; i < sizeof(frame); ++i) {
        g_uart_bytes[g_uart_tail++] = frame[i];
    }
}

static void advance_ms(uint32_t elapsed_ms)
{
    uint32_t i;

    assert(g_tick_callback != 0);
    for (i = 0U; i < elapsed_ms; ++i) {
        g_tick_callback(g_tick_context);
        ball_balance_process();
    }
}

static void advance_frame(uint32_t elapsed_ms, uint32_t capture_ms,
    float position_cm, uint8_t valid)
{
    advance_ms(elapsed_ms);
    queue_frame(capture_ms, 0, 0, position_cm,
        valid ? 0.9f : 0.0f, valid);
    ball_balance_process();
}

static void advance_frame_with_x(uint32_t elapsed_ms, uint32_t capture_ms,
    int16_t center_x_px, float position_cm)
{
    advance_ms(elapsed_ms);
    queue_frame(capture_ms, center_x_px, 119, position_cm, 0.9f, 1U);
    ball_balance_process();
}

static void reset_controller(void)
{
    g_uart_head = 0U;
    g_uart_tail = 0U;
    g_tick_callback = 0;
    g_tick_context = 0;
    g_last_pwm_duty = 0U;
    assert(ball_balance_init() == ML_STATUS_OK);
}

static uint32_t make_vision_ready_at(float position_cm)
{
    ball_balance_status_t status;

    advance_frame(20U, 10U, position_cm, 1U);
    advance_frame(20U, 30U, position_cm, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.vision_ready);
    advance_frame(20U, 50U, position_cm, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
    return 50U;
}

static uint32_t make_vision_ready(void)
{
    return make_vision_ready_at(0.0f);
}

static void apply_sequence_overspeed_margin(
    uint32_t *capture_ms, float desired_margin_cm_per_s)
{
    ball_balance_status_t status;
    float direction;
    float dt_s = 0.020f;
    float position_velocity_coefficient =
        (BALL_OBSERVER_ALPHA * dt_s) / BALL_OBSERVER_BETA;
    float position_base;
    float velocity_denominator;
    float desired_velocity;
    float predicted_position;
    float residual;
    float measured_position;

    assert(capture_ms != 0);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert((status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) ||
        (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM));
    direction = (status.sequence_state ==
        BALL_SEQUENCE_TO_PLUS_5_CM) ? 1.0f : -1.0f;
    position_base = status.position_cm +
        (status.velocity_cm_per_s * dt_s) -
        (position_velocity_coefficient * status.velocity_cm_per_s);
    velocity_denominator = 1.0f + BALL_POSITION_KD +
        (BALL_POSITION_KP_PER_S * position_velocity_coefficient);
    desired_velocity =
        ((desired_margin_cm_per_s / direction) +
         (BALL_POSITION_KP_PER_S *
          (status.target_cm - position_base)) +
         (BALL_POSITION_KI_PER_S2 * status.integral_cm_s)) /
        velocity_denominator;
    predicted_position = status.position_cm +
        (status.velocity_cm_per_s * dt_s);
    residual = (desired_velocity - status.velocity_cm_per_s) *
        dt_s / BALL_OBSERVER_BETA;
    measured_position = predicted_position + residual;
    *capture_ms += 20U;
    queue_frame(*capture_ms, 0, 0, measured_position, 0.9f, 1U);
    ball_balance_process();
    advance_ms(BALL_CONTROL_PERIOD_MS);
}

static void advance_linear_measurement(uint32_t *capture_ms,
    float *position_cm, float velocity_cm_per_s)
{
    assert(capture_ms != 0);
    assert(position_cm != 0);
    *position_cm += velocity_cm_per_s * 0.020f;
    *capture_ms += 20U;
    advance_frame(10U, *capture_ms, *position_cm, 1U);
}

ml_status_t pwm_init(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t frequency_hz)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    assert(frequency_hz == 50U);
    return ML_STATUS_OK;
}

ml_status_t pwm_update(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint32_t duty)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    g_last_pwm_duty = duty;
    return ML_STATUS_OK;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART2);
    assert(baud == 115200UL);
    assert(priority == 1U);
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART2);
    assert(byte != 0);
    if (g_uart_head == g_uart_tail) {
        g_uart_head = 0U;
        g_uart_tail = 0U;
        return ML_STATUS_BUFFER_EMPTY;
    }
    *byte = g_uart_bytes[g_uart_head++];
    return ML_STATUS_OK;
}

uint32_t uart_get_rx_overflow_count(UART_Regs *uart)
{
    assert(uart == UART2);
    return 0U;
}

ml_status_t tim_interrupt_ms_init_ex(GPTIMER_Regs *timer, uint32_t time_ms,
    uint8_t priority, tim_callback_t callback, void *context)
{
    assert(timer == TIMG6);
    assert(time_ms == 1U);
    assert(priority == 2U);
    assert(callback != 0);
    g_tick_callback = callback;
    g_tick_context = context;
    return ML_STATUS_OK;
}

static void test_start_gate_timeout_and_abort(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    uint16_t timeout_hold_us;

    reset_controller();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    assert(ball_balance_set_target_cm(12.0f) == ML_STATUS_INVALID_ARGUMENT);
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    advance_frame(20U, capture_ms += 20U, 2.0f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.control_mode == BALL_CONTROL_CASCADE);
    assert(status.target_velocity_cm_per_s > 0.0f);
    assert(status.integral_cm_s == 0.0f);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.0f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(ball_balance_abort_sequence() == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(!status.enabled);
    assert(status.integral_cm_s == 0.0f);
    assert(status.servo_target_us == 1500U);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 49U; ++i) {
        advance_frame(100U, capture_ms += 100U,
            0.5f + (0.05f * (float) i), 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    timeout_hold_us = status.servo_current_us;
    advance_frame(100U, capture_ms += 100U, 2.95f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TIMEOUT);
    assert(status.sequence_elapsed_ms == 5000U);
    assert(!status.enabled);
    assert(!status.brake_active);
    assert(status.servo_target_us == timeout_hold_us);
    assert(status.servo_current_us == timeout_hold_us);
    advance_ms(100U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.servo_target_us == timeout_hold_us);
    assert(status.servo_current_us == timeout_hold_us);
    assert(ball_balance_enable(false) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TIMEOUT);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);
}

static void test_manual_range_speed_test_and_control_period(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;

    reset_controller();
    assert(ball_balance_set_manual_servo_offset_us(-100) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.servo_target_us == 1400U);
    assert(ball_balance_set_manual_servo_offset_us(-101) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(ball_balance_set_manual_servo_offset_us(100) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.servo_target_us == 1600U);
    assert(ball_balance_set_manual_servo_offset_us(101) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(ball_balance_enable_speed_test(true) == ML_STATUS_BUSY);

    capture_ms = make_vision_ready();
    assert(ball_balance_enable_speed_test(true) == ML_STATUS_OK);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s == 0.0f);
    assert(status.speed_error_cm_per_s == 0.0f);
    assert(status.control_output_us == 0.0f);
    assert(status.servo_target_us == 1500U);
    advance_frame(20U, capture_ms += 20U, 1.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.control_mode == BALL_CONTROL_SPEED_TEST);
    assert(status.target_velocity_cm_per_s == 0.0f);
    assert(status.speed_error_cm_per_s < 0.0f);
    assert(status.control_output_us < 0.0f);
    assert(status.servo_target_us > 1500U);
    assert(ball_balance_enable_speed_test(false) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.control_mode == BALL_CONTROL_DISABLED);
    assert(status.control_output_us == 0.0f);
    assert(status.servo_target_us == 1500U);

    assert(ball_balance_set_target_cm(5.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    advance_ms(9U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.control_output_us == 0.0f);
    advance_ms(1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s > 0.0f);
    if (status.speed_error_cm_per_s > 0.0f) {
        assert(status.control_output_us > 0.0f);
        assert(status.servo_target_us < 1500U);
    } else {
        assert(status.speed_error_cm_per_s < 0.0f);
        assert(status.control_output_us < 0.0f);
        assert(status.servo_target_us > 1500U);
    }
}

static void test_speed_derivative_direction_symmetry_and_decay(void)
{
    ball_balance_status_t positive;
    ball_balance_status_t positive_decay;
    ball_balance_status_t negative;
    uint32_t capture_ms;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_enable_speed_test(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, 0.2f, 1U);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&positive) == ML_STATUS_OK);
    assert(positive.velocity_cm_per_s > 0.0f);
    assert(positive.control_output_us < 0.0f);
    assert(positive.control_output_us >= -BALL_CONTROL_LIMIT_US);
    assert(positive.servo_target_us >= 1300U);
    assert(positive.servo_target_us <= 1700U);

    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&positive_decay) == ML_STATUS_OK);
    assert(positive_decay.control_output_us < 0.0f);
    assert(fabsf(positive_decay.control_output_us) <
        fabsf(positive.control_output_us));

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_enable_speed_test(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, -0.2f, 1U);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&negative) == ML_STATUS_OK);
    assert(negative.velocity_cm_per_s < 0.0f);
    assert(negative.control_output_us > 0.0f);
    assert(negative.control_output_us <= BALL_CONTROL_LIMIT_US);
    assert(negative.servo_target_us >= 1300U);
    assert(negative.servo_target_us <= 1700U);
    assert(fabsf(positive.control_output_us +
        negative.control_output_us) < 0.1f);
}

static void test_direct_centimeter_input_and_safety_gates(void)
{
    ball_balance_status_t status;

    reset_controller();
    advance_ms(20U);
    queue_frame(10U, 300, 112, 2.0f, 0.9f, 1U);
    ball_balance_process();
    advance_ms(20U);
    queue_frame(30U, 300, 112, 2.0f, 0.9f, 1U);
    ball_balance_process();
    advance_ms(20U);
    queue_frame(50U, 300, 112, 2.0f, 0.9f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
    assert(status.raw_center_x_px == 300);
    assert(status.position_cm > 1.9f);

    queue_frame(70U, 0, 0, 13.0f, 0.9f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
    assert(status.position_cm > 1.9f);
    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.vision_ready);

    reset_controller();
    queue_frame(10U, 0, 0, 0.0f, 0.19f, 1U);
    ball_balance_process();
    queue_frame(30U, 0, 0, 0.0f, 0.19f, 1U);
    ball_balance_process();
    queue_frame(50U, 0, 0, 0.0f, 0.19f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.vision_ready);

    reset_controller();
    queue_frame(10U, 0, 0, 0.0f, 0.20f, 1U);
    ball_balance_process();
    queue_frame(30U, 0, 0, 0.0f, 0.20f, 1U);
    ball_balance_process();
    queue_frame(50U, 0, 0, 0.0f, 0.20f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
}

static void test_vision_loss_and_reacquire(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t target_elapsed_ms;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, 0.0f, 0U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.state == BALL_BALANCE_ACTIVE);
    assert(status.enabled);
    assert(status.vision_ready);
    assert(status.raw_center_x_px == 0);
    assert(status.raw_center_y_px == 0);
    assert(status.raw_score == 0.0f);

    advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.enabled);

    advance_ms(20U);
    queue_frame(capture_ms += 20U, 0, 0, 0.0f, 0.19f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.enabled);
    advance_ms(20U);
    queue_frame(capture_ms += 20U, 0, 0, 13.0f, 0.9f, 1U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.enabled);

    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.state == BALL_BALANCE_WAITING_FOR_VISION);
    assert(!status.enabled);
    assert(!status.vision_ready);
    assert(status.target_cm == BALL_SEQUENCE_PLUS_CM);
    assert(status.control_mode == BALL_CONTROL_DISABLED);
    assert(status.integral_cm_s == 0.0f);
    assert(status.control_output_us == 0.0f);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);
    target_elapsed_ms = status.sequence_elapsed_ms;

    advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(!status.vision_ready);
    advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(status.state == BALL_BALANCE_ACTIVE);
    assert(status.enabled);
    assert(status.vision_ready);
    assert(status.target_cm == BALL_SEQUENCE_PLUS_CM);
    assert(status.sequence_elapsed_ms > target_elapsed_ms);
    assert(status.integral_cm_s == 0.0f);
    assert(status.control_output_us == 0.0f);

    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(ball_balance_abort_sequence() == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_ABORTED);

    reset_controller();
    (void) make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_ms(BALL_VISION_TIMEOUT_MS);
    advance_ms(BALL_SEQUENCE_TIMEOUT_MS - BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TIMEOUT);
    assert(status.sequence_elapsed_ms == BALL_SEQUENCE_TIMEOUT_MS);
    assert(!status.enabled);
}

static void test_invalid_frame_clears_sequence_settle(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U,
        BALL_SEQUENCE_PLUS_CM, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    advance_frame(20U, capture_ms += 20U, 0.0f, 0U);
    advance_ms(100U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    advance_frame(20U, capture_ms += 20U,
        BALL_SEQUENCE_PLUS_CM, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    for (i = 0U; i < 2U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(status.control_mode == BALL_CONTROL_CASCADE);
}

static void test_sequence_plus_position_confirm_and_final_low_speed(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float measured_position_cm;
    const float minus_approach[] = {3.0f, 1.0f, -1.0f, -3.5f};

    reset_controller();
    capture_ms = make_vision_ready_at(0.0f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s ==
        BALL_SEQUENCE_CRUISE_SPEED_CM_PER_S);
    assert(status.control_output_us > 0.0f);
    assert(status.servo_target_us < BALL_SERVO_CENTER_US);

    reset_controller();
    capture_ms = make_vision_ready_at(3.8f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    measured_position_cm = 3.8f;
    for (i = 0U; i < 45U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            1.01f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(fabsf(status.error_cm) <= BALL_SEQUENCE_PLUS_ERROR_CM);
    assert(status.velocity_cm_per_s >
        BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S);
    assert(status.target_velocity_cm_per_s == 0.0f);

    for (i = 0U; i < 100U; ++i) {
        advance_frame(20U, capture_ms += 20U, 5.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(fabsf(status.velocity_cm_per_s) <=
        BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S);

    for (i = 0U;
         i < (sizeof(minus_approach) / sizeof(minus_approach[0]));
         ++i) {
        advance_frame(10U, capture_ms += 20U, minus_approach[i], 1U);
    }
    measured_position_cm = -3.5f;
    for (i = 0U; i < 80U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            -1.01f);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(fabsf(status.error_cm) <= BALL_SEQUENCE_FINAL_ERROR_CM);
    assert(status.velocity_cm_per_s <
        -BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S);

    for (i = 0U; i < 20U; ++i) {
        advance_frame(10U, capture_ms += 20U, -3.5f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(fabsf(status.error_cm) > BALL_SEQUENCE_FINAL_ERROR_CM);

    measured_position_cm = -3.5f;
    for (i = 0U; i < 100U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            -0.99f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_COMPLETE) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(fabsf(status.velocity_cm_per_s) <=
        BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S);
}

static void test_sequence_minus_braking_capture_and_recovery(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float measured_position_cm;
    bool braking_seen = false;
    bool high_speed_band_seen = false;
    bool low_speed_band_seen = false;

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s ==
        -BALL_SEQUENCE_MINUS_CRUISE_SPEED_CM_PER_S);

    measured_position_cm = BALL_SEQUENCE_PLUS_CM;
    for (i = 0U; i < 100U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            -8.0f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if ((status.target_velocity_cm_per_s >
             -BALL_SEQUENCE_MINUS_CRUISE_SPEED_CM_PER_S) &&
            (fabsf(status.error_cm) > BALL_SEQUENCE_FINAL_ERROR_CM)) {
            braking_seen = true;
            assert(status.target_velocity_cm_per_s >=
                -BALL_SEQUENCE_MINUS_APPROACH_SPEED_LIMIT_CM_PER_S);
        }
        if ((fabsf(status.error_cm) <= BALL_SEQUENCE_FINAL_ERROR_CM) &&
            (fabsf(status.velocity_cm_per_s) >
             BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S)) {
            high_speed_band_seen = true;
            break;
        }
    }
    assert(braking_seen);
    assert(high_speed_band_seen);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s < 0.0f);
    assert(status.target_velocity_cm_per_s >=
        -BALL_SEQUENCE_MINUS_APPROACH_SPEED_LIMIT_CM_PER_S);

    for (i = 0U; i < 50U; ++i) {
        measured_position_cm = -4.5f;
        advance_frame(20U, capture_ms += 20U,
            measured_position_cm, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if ((fabsf(status.error_cm) <= BALL_SEQUENCE_FINAL_ERROR_CM) &&
            (fabsf(status.velocity_cm_per_s) <=
             BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S)) {
            low_speed_band_seen = true;
            break;
        }
    }
    assert(low_speed_band_seen);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s < 0.0f);
    assert(status.target_velocity_cm_per_s >=
        -BALL_SEQUENCE_MINUS_RECOVERY_SPEED_LIMIT_CM_PER_S);

    for (i = 0U; i < 10U; ++i) {
        measured_position_cm = -3.5f;
        advance_frame(20U, capture_ms += 20U,
            measured_position_cm, 1U);
    }
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.error_cm < -BALL_SEQUENCE_FINAL_ERROR_CM);
    assert(status.target_velocity_cm_per_s >=
        -BALL_SEQUENCE_MINUS_RECOVERY_SPEED_LIMIT_CM_PER_S);
    assert(!status.breakaway_active);

    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_MINUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (fabsf(status.error_cm) <=
            BALL_SEQUENCE_MINUS_CAPTURE_DEADBAND_CM) {
            break;
        }
    }
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(fabsf(status.error_cm) <=
        BALL_SEQUENCE_MINUS_CAPTURE_DEADBAND_CM);
    assert(status.target_velocity_cm_per_s == 0.0f);
    measured_position_cm = BALL_SEQUENCE_MINUS_CM;

    for (i = 0U; i < 30U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            -8.0f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.error_cm > BALL_SEQUENCE_FINAL_ERROR_CM) {
            break;
        }
    }
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.error_cm > BALL_SEQUENCE_FINAL_ERROR_CM);
    assert(status.target_velocity_cm_per_s > 0.0f);
    assert(status.target_velocity_cm_per_s <=
        BALL_SEQUENCE_MINUS_RECOVERY_SPEED_LIMIT_CM_PER_S);

    for (i = 0U; i < 40U; ++i) {
        advance_linear_measurement(&capture_ms, &measured_position_cm,
            8.0f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(!status.breakaway_active);
        if (status.error_cm < -BALL_SEQUENCE_FINAL_ERROR_CM) {
            break;
        }
    }
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.error_cm < -BALL_SEQUENCE_FINAL_ERROR_CM);
    assert(status.target_velocity_cm_per_s < 0.0f);
    assert(status.target_velocity_cm_per_s >=
        -BALL_SEQUENCE_MINUS_RECOVERY_SPEED_LIMIT_CM_PER_S);

    for (i = 0U; i < 80U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            measured_position_cm, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active) {
            break;
        }
    }
    assert(status.breakaway_active);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);
}

static void test_complete_sequence_and_hold(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t completed_elapsed_ms;
    uint32_t i;
    const float plus_path[] = {2.0f, 4.0f, 5.0f};
    const float minus_path[] = {3.0f, 1.0f, -1.0f, -3.0f, -5.0f};

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < (sizeof(plus_path) / sizeof(plus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, plus_path[i], 1U);
    }
    for (i = 0U; i < 80U; ++i) {
        advance_frame(20U, capture_ms += 20U, 5.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(fabsf(status.target_cm + 5.0f) < 0.01f);

    for (i = 0U; i < (sizeof(minus_path) / sizeof(minus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, minus_path[i], 1U);
    }
    for (i = 0U; i < 120U; ++i) {
        advance_frame(20U, capture_ms += 20U, -5.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_COMPLETE) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(status.enabled);
    assert(status.control_mode == BALL_CONTROL_CASCADE);
    assert(status.sequence_elapsed_ms < 5000U);
    assert(fabsf(status.target_cm + 5.0f) < 0.01f);
    assert(status.servo_target_us != 1500U ||
        fabsf(status.error_cm) < 0.05f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    assert(g_last_pwm_duty >= 3250U);
    assert(g_last_pwm_duty <= 4250U);
    completed_elapsed_ms = status.sequence_elapsed_ms;
    assert(ball_balance_enable(false) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(status.sequence_elapsed_ms == completed_elapsed_ms);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);
}

static void test_sequence_breakaway_threshold_direction_and_release(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float boost_before_reverse_jitter;
    uint16_t pre_breakaway_target_us;
    const float minus_path[] = {3.0f, 1.0f, -1.0f, -3.0f, -3.7f};

    reset_controller();
    capture_ms = make_vision_ready_at(4.11f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 25U; ++i) {
        advance_frame(20U, capture_ms += 20U, 4.11f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(0.0f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            0.02f * (float) (i + 1U), 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);

    reset_controller();
    capture_ms = make_vision_ready_at(-0.2f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 14U; ++i) {
        advance_frame(20U, capture_ms += 20U, -0.2f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    pre_breakaway_target_us = status.servo_target_us;
    assert(pre_breakaway_target_us < BALL_BREAKAWAY_SERVO_MINIMUM_US);
    advance_frame(20U, capture_ms += 20U, -0.2f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    assert(fabsf(status.breakaway_boost_us -
        BALL_BREAKAWAY_MAXIMUM_US) < 0.01f);
    assert(status.servo_target_us ==
        BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US);
    assert(status.servo_target_us <= pre_breakaway_target_us);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, capture_ms += 20U, -0.2f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.servo_target_us < pre_breakaway_target_us);
    assert(status.servo_target_us >=
        BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US);
    assert(fabsf(status.breakaway_boost_us -
        BALL_BREAKAWAY_MAXIMUM_US) < 0.01f);
    assert(status.integral_cm_s == 0.0f);
    assert(!status.breakaway_fault);

    boost_before_reverse_jitter = status.breakaway_boost_us;
    advance_frame(20U, capture_ms += 20U, -0.3f, 1U);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.velocity_cm_per_s <
        -BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S);
    assert(status.position_cm >
        (-0.2f - BALL_BREAKAWAY_AWAY_DISPLACEMENT_CM));
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us == boost_before_reverse_jitter);
    assert(status.integral_cm_s == 0.0f);

    advance_frame(20U, capture_ms += 20U, -0.6f, 1U);
    advance_ms(BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.velocity_cm_per_s <
        -BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S);
    assert(status.position_cm <=
        (-0.2f - BALL_BREAKAWAY_AWAY_DISPLACEMENT_CM));
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(3.7f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, 3.7f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us > 0.0f);
    assert(status.servo_target_us < BALL_SERVO_CENTER_US);
    assert(status.servo_target_us >=
        BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US);
    assert(status.integral_cm_s == 0.0f);

    advance_frame(20U, capture_ms += 20U, 4.0f, 1U);
    for (i = 0U; i < 6U; ++i) {
        advance_ms(BALL_CONTROL_PERIOD_MS);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    for (i = 0U; i < (sizeof(minus_path) / sizeof(minus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, minus_path[i], 1U);
    }
    for (i = 0U; i < 100U; ++i) {
        advance_frame(20U, capture_ms += 20U, -3.7f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(status.breakaway_active);
    assert(fabsf(status.breakaway_boost_us +
        BALL_BREAKAWAY_MAXIMUM_US) < 0.01f);
    assert(status.servo_target_us > BALL_SERVO_CENTER_US);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);
}

static void test_sequence_breakaway_final_tolerance_and_fault(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    bool reached_sequence_minimum = false;
    bool reached_sequence_stage1 = false;
    bool reached_sequence_maximum = false;
    bool passed_old_minimum = false;
    bool passed_1350_without_fault = false;
    const float minus_path[] = {3.0f, 1.0f, -1.0f, -3.0f, -4.05f};

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    for (i = 0U; i < (sizeof(minus_path) / sizeof(minus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, minus_path[i], 1U);
    }
    for (i = 0U; i < 100U; ++i) {
        advance_frame(20U, capture_ms += 20U, -4.05f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(!status.breakaway_active);
        if (status.sequence_state == BALL_SEQUENCE_COMPLETE) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U, -3.7f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(!status.breakaway_active);
    assert(!status.brake_active);

    reset_controller();
    capture_ms = make_vision_ready_at(1.6f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 200U; ++i) {
        advance_frame(20U, capture_ms += 20U, 1.6f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active &&
            (status.servo_current_us < BALL_BREAKAWAY_SERVO_MINIMUM_US)) {
            passed_old_minimum = true;
        }
        if (status.breakaway_active && !status.breakaway_fault &&
            (status.servo_current_us < 1350U)) {
            passed_1350_without_fault = true;
        }
        if (status.breakaway_active &&
            (status.servo_current_us ==
             BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US)) {
            reached_sequence_stage1 = true;
        }
        if (status.breakaway_active &&
            (status.servo_current_us ==
             BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US)) {
            reached_sequence_minimum = true;
        }
        if (status.breakaway_fault) {
            break;
        }
    }
    assert(passed_old_minimum);
    assert(passed_1350_without_fault);
    assert(reached_sequence_stage1);
    assert(reached_sequence_minimum);
    assert(status.breakaway_fault);
    assert(status.state == BALL_BALANCE_BREAKAWAY_FAULT);
    assert(status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(status.sequence_elapsed_ms < BALL_SEQUENCE_TIMEOUT_MS);
    assert(!status.enabled);
    assert(!status.breakaway_active);
    assert(!status.brake_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);

    reset_controller();
    capture_ms = make_vision_ready_at(1.6f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 100U; ++i) {
        advance_frame(20U, capture_ms += 20U, 1.6f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active &&
            (status.servo_current_us ==
             BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US)) {
            break;
        }
    }
    assert(status.breakaway_active);
    assert(!status.breakaway_fault);
    for (i = 0U; i < 8U; ++i) {
        advance_frame(20U, capture_ms += 20U, 1.6f, 1U);
    }
    advance_frame(20U, capture_ms += 20U, 1.85f, 1U);
    advance_frame(20U, capture_ms += 20U, 2.10f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(!status.breakaway_fault);

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    for (i = 0U; i < 60U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active &&
            (status.servo_current_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US)) {
            reached_sequence_maximum = true;
        }
        if (status.breakaway_fault) {
            break;
        }
    }
    assert(status.breakaway_fault);
    assert(status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(status.sequence_elapsed_ms < BALL_SEQUENCE_TIMEOUT_MS);
    assert(reached_sequence_maximum);
    assert(!status.brake_active);
}

static void test_sequence_overspeed_brake_hysteresis_and_resets(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float integral_before_brake;
    float overspeed_margin;

#if !BALL_SEQUENCE_OVERSPEED_BRAKE_ENABLED
    reset_controller();
    capture_ms = make_vision_ready_at(0.0f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    apply_sequence_overspeed_margin(&capture_ms, 0.51f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    overspeed_margin = status.velocity_cm_per_s -
        status.target_velocity_cm_per_s;
    assert(overspeed_margin >
        BALL_SEQUENCE_BRAKE_ENTER_MARGIN_CM_PER_S);
    assert(!status.brake_active);
    assert(status.control_output_us < 0.0f);
    assert(status.control_output_us >= -BALL_CONTROL_LIMIT_US);
    assert(status.servo_target_us > BALL_SERVO_CENTER_US);
    assert(status.servo_target_us <= BALL_SERVO_MAXIMUM_US);

    reset_controller();
    capture_ms = make_vision_ready_at(2.5f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 8U; ++i) {
        apply_sequence_overspeed_margin(&capture_ms, 0.0f);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    integral_before_brake = status.integral_cm_s;
    assert(integral_before_brake > 0.0f);
    apply_sequence_overspeed_margin(&capture_ms, 0.60f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.brake_active);
    assert(status.integral_cm_s > integral_before_brake);

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    apply_sequence_overspeed_margin(&capture_ms, 0.60f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.brake_active);
    assert(status.control_output_us > 0.0f);
    assert(status.control_output_us <= BALL_CONTROL_LIMIT_US);
    assert(status.servo_target_us < BALL_SERVO_CENTER_US);
    assert(status.servo_target_us >=
        BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US);
    return;
#endif

    reset_controller();
    capture_ms = make_vision_ready_at(0.0f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    apply_sequence_overspeed_margin(&capture_ms, 0.49f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    overspeed_margin = status.velocity_cm_per_s -
        status.target_velocity_cm_per_s;
    assert(overspeed_margin <
        BALL_SEQUENCE_BRAKE_ENTER_MARGIN_CM_PER_S);
    assert(!status.brake_active);

    apply_sequence_overspeed_margin(&capture_ms, 0.51f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    overspeed_margin = status.velocity_cm_per_s -
        status.target_velocity_cm_per_s;
    assert(overspeed_margin >
        BALL_SEQUENCE_BRAKE_ENTER_MARGIN_CM_PER_S);
    assert(status.brake_active);
    assert(status.control_output_us == -150.0f);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);

    apply_sequence_overspeed_margin(&capture_ms, 0.30f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    overspeed_margin = status.velocity_cm_per_s -
        status.target_velocity_cm_per_s;
    assert(overspeed_margin >
        BALL_SEQUENCE_BRAKE_EXIT_MARGIN_CM_PER_S);
    assert(overspeed_margin <
        BALL_SEQUENCE_BRAKE_ENTER_MARGIN_CM_PER_S);
    assert(status.brake_active);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);

    apply_sequence_overspeed_margin(&capture_ms, 0.0f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert((status.velocity_cm_per_s -
            status.target_velocity_cm_per_s) <=
        BALL_SEQUENCE_BRAKE_EXIT_MARGIN_CM_PER_S);
    assert(!status.brake_active);

    reset_controller();
    capture_ms = make_vision_ready_at(2.5f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 8U; ++i) {
        apply_sequence_overspeed_margin(&capture_ms, 0.0f);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.brake_active);
    assert(status.integral_cm_s > 0.0f);
    integral_before_brake = status.integral_cm_s;
    apply_sequence_overspeed_margin(&capture_ms, 0.60f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.brake_active);
    assert(fabsf(status.integral_cm_s - integral_before_brake) < 0.001f);
    advance_ms(3U * BALL_CONTROL_PERIOD_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.brake_active);
    assert(fabsf(status.integral_cm_s - integral_before_brake) < 0.001f);
    assert(ball_balance_abort_sequence() == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.brake_active);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);

    reset_controller();
    capture_ms = make_vision_ready_at(BALL_SEQUENCE_PLUS_CM);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, capture_ms += 20U,
            BALL_SEQUENCE_PLUS_CM, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(!status.brake_active);
    apply_sequence_overspeed_margin(&capture_ms, 0.60f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.brake_active);
    assert(status.control_output_us == BALL_CONTROL_LIMIT_US);
    assert(status.servo_target_us ==
        BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US);
    assert(ball_balance_abort_sequence() == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.brake_active);

    reset_controller();
    capture_ms = make_vision_ready_at(0.0f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    apply_sequence_overspeed_margin(&capture_ms, 0.60f);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.brake_active);
    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(!status.brake_active);
    assert(status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
}

static void test_zero_center_outer_loop_direction(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, 3.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.control_mode == BALL_CONTROL_CASCADE);
    assert(status.target_cm == 0.0f);
    assert(status.target_velocity_cm_per_s < 0.0f);
    assert(status.target_velocity_cm_per_s >= -10.0f);
    assert(status.control_output_us < 0.0f);
    assert(status.control_output_us >= -200.0f);
    assert(status.servo_target_us > 1500U);
    assert(status.integral_cm_s == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, -3.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.target_velocity_cm_per_s > 0.0f);
    assert(status.target_velocity_cm_per_s <= 10.0f);
    assert(status.control_output_us > 0.0f);
    assert(status.control_output_us <= 200.0f);
    assert(status.servo_target_us < 1500U);
    assert(status.integral_cm_s == 0.0f);
}

static void test_position_derivative_braking_symmetry(void)
{
    ball_balance_status_t positive_motion;
    ball_balance_status_t negative_motion;
    uint32_t capture_ms;
    float projected_braking_us;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, 1.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&positive_motion) == ML_STATUS_OK);
    assert(positive_motion.velocity_cm_per_s > 0.0f);
    assert(positive_motion.target_velocity_cm_per_s < 0.0f);
    assert(positive_motion.control_output_us < 0.0f);
    assert(positive_motion.control_output_us >= -BALL_CONTROL_LIMIT_US);
    assert(positive_motion.servo_target_us > BALL_SERVO_CENTER_US);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    advance_frame(20U, capture_ms += 20U, -1.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&negative_motion) == ML_STATUS_OK);
    assert(negative_motion.velocity_cm_per_s < 0.0f);
    assert(negative_motion.target_velocity_cm_per_s > 0.0f);
    assert(negative_motion.control_output_us > 0.0f);
    assert(negative_motion.control_output_us <= BALL_CONTROL_LIMIT_US);
    assert(negative_motion.servo_target_us < BALL_SERVO_CENTER_US);

    assert(fabsf(positive_motion.velocity_cm_per_s +
        negative_motion.velocity_cm_per_s) < 0.01f);
    assert(fabsf(positive_motion.target_velocity_cm_per_s +
        negative_motion.target_velocity_cm_per_s) < 0.01f);
    assert(fabsf(positive_motion.control_output_us +
        negative_motion.control_output_us) < 0.1f);
    projected_braking_us = BALL_SPEED_KP_US_PER_CM_PER_S *
        BALL_POSITION_KD * 5.7f;
    assert(projected_braking_us >= 61.5f);
    assert(projected_braking_us <= 61.7f);
}

static void test_position_integral_separation_limit_and_resets(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float integral_velocity_contribution;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(2.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 350U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s >= 11.99f);
    assert(status.integral_cm_s <=
        BALL_POSITION_INTEGRAL_LIMIT_CM_S);
    integral_velocity_contribution =
        BALL_POSITION_KI_PER_S2 * status.integral_cm_s;
    assert(fabsf(integral_velocity_contribution) >= 3.59f);
    assert(fabsf(integral_velocity_contribution) <= 3.61f);
    assert(fabsf(BALL_SPEED_KP_US_PER_CM_PER_S *
        integral_velocity_contribution) >= 97.1f);
    assert(fabsf(BALL_SPEED_KP_US_PER_CM_PER_S *
        integral_velocity_contribution) <= 97.3f);
    assert(BALL_SPEED_KP_US_PER_CM_PER_S *
        ((BALL_POSITION_KP_PER_S *
          BALL_POSITION_INTEGRAL_SEPARATION_CM) +
         (BALL_POSITION_KI_PER_S2 *
          BALL_POSITION_INTEGRAL_LIMIT_CM_S)) >= 161.9f);
    assert(BALL_SPEED_KP_US_PER_CM_PER_S *
        ((BALL_POSITION_KP_PER_S *
          BALL_POSITION_INTEGRAL_SEPARATION_CM) +
         (BALL_POSITION_KI_PER_S2 *
          BALL_POSITION_INTEGRAL_LIMIT_CM_S)) <= 162.1f);

    advance_frame(20U, capture_ms += 20U, -4.0f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(fabsf(status.error_cm) >
        BALL_POSITION_INTEGRAL_SEPARATION_CM);
    assert(status.integral_cm_s == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(2.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 50U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s > 0.5f);
    assert(ball_balance_enable(false) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(2.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 50U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s > 0.5f);
    advance_frame(20U, capture_ms += 20U, 0.0f, 0U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.enabled);
    assert(status.integral_cm_s > 0.5f);
    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(status.integral_cm_s == 0.0f);
}

static void test_position_integral_freezes_on_inner_saturation(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float integral_before_step;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_set_target_cm(2.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 50U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.0f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s > 0.25f);
    integral_before_step = status.integral_cm_s;

    capture_ms += 20U;
    queue_frame(capture_ms, 0, 0, -1.0f, 0.9f, 1U);
    ball_balance_process();
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(fabsf(status.error_cm) <=
        BALL_POSITION_INTEGRAL_SEPARATION_CM);
    assert(status.control_output_us >= 199.0f);
    assert(fabsf(status.integral_cm_s - integral_before_step) < 0.001f);
}

static void test_breakaway_arming_ramp_release_and_retrigger(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    bool retriggered = false;

    reset_controller();
    capture_ms = make_vision_ready_at(2.4f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 14U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.integral_cm_s == 0.0f);

    advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us < 0.0f);
    assert(fabsf(status.breakaway_boost_us + 0.5f) < 0.01f);
    assert(status.integral_cm_s == 0.0f);

    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_boost_us <= -10.4f);
    assert(status.breakaway_boost_us >= -10.6f);
    assert(status.servo_target_us > BALL_SERVO_CENTER_US);
    assert(status.servo_target_us <= BALL_BREAKAWAY_SERVO_MAXIMUM_US);

    advance_frame_with_x(20U, capture_ms += 20U, 134, 2.3f);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.velocity_cm_per_s <=
        -BALL_BREAKAWAY_RELEASE_SPEED_CM_PER_S);
    assert(status.position_cm >
        (2.4f - BALL_BREAKAWAY_RELEASE_DISPLACEMENT_CM));
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us < -10.5f);
    assert(status.integral_cm_s == 0.0f);

    advance_frame(20U, capture_ms += 20U, 2.0f, 1U);
    for (i = 0U; i < 5U; ++i) {
        advance_ms(10U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(status.velocity_cm_per_s <=
            -BALL_BREAKAWAY_RELEASE_SPEED_CM_PER_S);
        assert(status.position_cm <=
            (2.4f - BALL_BREAKAWAY_RELEASE_DISPLACEMENT_CM));
        assert(status.breakaway_active);
        assert(status.integral_cm_s == 0.0f);
    }

    capture_ms += 20U;
    queue_frame(capture_ms, 0, 0, 2.412f, 0.9f, 1U);
    ball_balance_process();
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.position_cm >
        (2.4f - BALL_BREAKAWAY_RELEASE_DISPLACEMENT_CM));
    assert(status.breakaway_active);
    assert(status.integral_cm_s == 0.0f);

    capture_ms += 20U;
    queue_frame(capture_ms, 0, 0, 1.9f, 0.9f, 1U);
    ball_balance_process();
    for (i = 0U; i < 5U; ++i) {
        advance_ms(10U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(status.breakaway_active);
        assert(status.integral_cm_s == 0.0f);
    }
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.integral_cm_s < 0.0f);

    for (i = 0U; i < 100U; ++i) {
        advance_frame(20U, capture_ms += 20U, 1.8f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.breakaway_active) {
            retriggered = true;
            break;
        }
    }
    assert(retriggered);
    assert(!status.breakaway_fault);

    reset_controller();
    capture_ms = make_vision_ready_at(0.89f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 30U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.89f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.integral_cm_s < 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(-0.89f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 30U; ++i) {
        advance_frame(20U, capture_ms += 20U, -0.89f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.integral_cm_s > 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(0.91f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, 0.91f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us < 0.0f);
    assert(status.integral_cm_s == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(-0.91f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, -0.91f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us > 0.0f);
    assert(status.integral_cm_s == 0.0f);
}

static void test_breakaway_release_qualification_and_pixel_jitter(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float minimum_velocity = 0.0f;

    reset_controller();
    capture_ms = make_vision_ready_at(2.4f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);

    advance_frame(100U, capture_ms += 100U, 2.1f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.position_cm <=
        (2.4f - BALL_BREAKAWAY_RELEASE_DISPLACEMENT_CM));
    assert(status.velocity_cm_per_s <
        -BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S);
    assert(status.velocity_cm_per_s >
        -BALL_BREAKAWAY_RELEASE_SPEED_CM_PER_S);
    assert(status.breakaway_active);
    assert(status.integral_cm_s == 0.0f);

    reset_controller();
    capture_ms = make_vision_ready_at(2.475f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.475f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    for (i = 0U; i < 20U; ++i) {
        bool low_pixel = (i & 1U) == 0U;

        advance_frame_with_x(20U, capture_ms += 20U,
            low_pixel ? 134 : 133,
            low_pixel ? 2.3833f : 2.475f);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.velocity_cm_per_s < minimum_velocity) {
            minimum_velocity = status.velocity_cm_per_s;
        }
        assert(status.breakaway_active);
        assert(status.integral_cm_s == 0.0f);
        assert(!status.breakaway_fault);
    }
    assert(minimum_velocity <=
        -BALL_BREAKAWAY_RELEASE_SPEED_CM_PER_S);
    assert(status.breakaway_boost_us < -0.5f);

    reset_controller();
    capture_ms = make_vision_ready_at(2.4f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    advance_frame(20U, capture_ms += 20U, 2.5f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.velocity_cm_per_s >
        BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S);
    assert(status.position_cm <
        (2.4f + BALL_BREAKAWAY_AWAY_DISPLACEMENT_CM));
    assert(status.breakaway_active);
    assert(status.breakaway_boost_us < 0.0f);
    assert(status.integral_cm_s == 0.0f);

    advance_frame(20U, capture_ms += 20U, 2.8f, 1U);
    advance_ms(10U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.velocity_cm_per_s >
        BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S);
    assert(status.position_cm >=
        (2.4f + BALL_BREAKAWAY_AWAY_DISPLACEMENT_CM));
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
}

static void test_breakaway_limits_fault_and_safety_resets(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    bool passed_old_boost_limit_before_servo_limit = false;

    reset_controller();
    capture_ms = make_vision_ready_at(1.0f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 400U; ++i) {
        advance_frame(10U, capture_ms += 10U, 1.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if ((status.breakaway_boost_us <= -95.0f) &&
            (status.servo_current_us <
             BALL_BREAKAWAY_SERVO_MAXIMUM_US)) {
            passed_old_boost_limit_before_servo_limit = true;
            assert(!status.breakaway_fault);
        }
        if (status.servo_current_us ==
            BALL_BREAKAWAY_SERVO_MAXIMUM_US) {
            break;
        }
    }
    assert(passed_old_boost_limit_before_servo_limit);
    assert(status.breakaway_active);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);
    assert(status.servo_current_us == BALL_BREAKAWAY_SERVO_MAXIMUM_US);
    assert(status.integral_cm_s == 0.0f);
    assert(!status.breakaway_fault);
    for (i = 0U; i < 49U; ++i) {
        advance_frame(10U, capture_ms += 10U, 1.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(!status.breakaway_fault);
    }
    assert(status.breakaway_boost_us <= -149.9f);
    advance_frame(10U, capture_ms += 10U, 1.0f, 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_fault);
    assert(status.state == BALL_BALANCE_BREAKAWAY_FAULT);
    assert(!status.enabled);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);
    assert(ball_balance_enable(true) == ML_STATUS_BUSY);
    assert(ball_balance_enable_speed_test(true) == ML_STATUS_BUSY);
    assert(ball_balance_set_manual_servo_offset_us(1) == ML_STATUS_BUSY);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    advance_frame(20U, capture_ms += 20U, 1.0f, 0U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_fault);
    assert(status.state == BALL_BALANCE_BREAKAWAY_FAULT);

    reset_controller();
    capture_ms = make_vision_ready_at(-1.0f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 200U; ++i) {
        advance_frame(20U, capture_ms += 20U, -1.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.servo_current_us ==
            BALL_BREAKAWAY_SERVO_MINIMUM_US) {
            break;
        }
    }
    assert(status.breakaway_active);
    assert(status.servo_target_us == BALL_BREAKAWAY_SERVO_MINIMUM_US);
    assert(status.servo_current_us == BALL_BREAKAWAY_SERVO_MINIMUM_US);
    assert(!status.breakaway_fault);
    for (i = 0U; i < 15U; ++i) {
        advance_frame(20U, capture_ms += 20U, -1.0f, 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        assert(!status.breakaway_fault);
    }
    assert(status.breakaway_boost_us > 0.0f);
    assert(status.breakaway_boost_us <= BALL_BREAKAWAY_MAXIMUM_US);

    assert(ball_balance_enable(false) == ML_STATUS_OK);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(!status.breakaway_fault);
    assert(status.servo_target_us == BALL_SERVO_CENTER_US);

    reset_controller();
    capture_ms = make_vision_ready_at(2.4f);
    assert(ball_balance_set_target_cm(0.0f) == ML_STATUS_OK);
    assert(ball_balance_enable(true) == ML_STATUS_OK);
    for (i = 0U; i < 25U; ++i) {
        advance_frame(20U, capture_ms += 20U, 2.4f, 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.breakaway_active);
    advance_frame(20U, capture_ms += 20U, 2.4f, 0U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.enabled);
    assert(status.breakaway_active);
    advance_ms(BALL_VISION_TIMEOUT_MS);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.enabled);
    assert(!status.breakaway_active);
    assert(status.breakaway_boost_us == 0.0f);
    assert(!status.breakaway_fault);
}

int main(void)
{
    assert(BALL_SEQUENCE_BREAKAWAY_ENABLED == 1);
    assert(BALL_POSITION_KP_PER_S == 0.8f);
    assert(BALL_POSITION_KD == 0.40f);
    assert(BALL_POSITION_KI_PER_S2 == 0.30f);
    assert(BALL_POSITION_INTEGRAL_LIMIT_CM_S == 12.0f);
    assert(BALL_POSITION_INTEGRAL_SEPARATION_CM == 3.0f);
    assert(BALL_SPEED_KP_US_PER_CM_PER_S == 27.0f);
    assert(BALL_SPEED_KD_US_PER_CM_PER_S2 == 3.0f);
    assert(BALL_SEQUENCE_SPEED_KD_US_PER_CM_PER_S2 == 0.0f);
    assert(BALL_SEQUENCE_CRUISE_SPEED_CM_PER_S == 6.0f);
    assert(BALL_SEQUENCE_BRAKE_ACCEL_CM_PER_S2 == 45.0f);
    assert(BALL_SEQUENCE_BRAKE_MARGIN_CM == 0.15f);
    assert(BALL_SEQUENCE_APPROACH_KP_PER_S == 4.0f);
    assert(BALL_SEQUENCE_MINUS_CRUISE_SPEED_CM_PER_S == 5.0f);
    assert(BALL_SEQUENCE_MINUS_BRAKE_ACCEL_CM_PER_S2 == 8.0f);
    assert(BALL_SEQUENCE_MINUS_BRAKE_MARGIN_CM == 0.30f);
    assert(BALL_SEQUENCE_MINUS_APPROACH_KP_PER_S == 2.0f);
    assert(BALL_SEQUENCE_MINUS_APPROACH_SPEED_LIMIT_CM_PER_S == 4.0f);
    assert(BALL_SEQUENCE_MINUS_CAPTURE_ERROR_CM == 0.80f);
    assert(BALL_SEQUENCE_MINUS_CAPTURE_KP_PER_S == 4.0f);
    assert(BALL_SEQUENCE_MINUS_RECOVERY_SPEED_LIMIT_CM_PER_S == 3.0f);
    assert(BALL_SEQUENCE_MINUS_CAPTURE_DEADBAND_CM == 0.15f);
    assert(BALL_SEQUENCE_MINUS_BRAKE_SPEED_KD_US_PER_CM_PER_S2 == 3.0f);
    assert(BALL_BREAKAWAY_ERROR_MINIMUM_CM == 0.9f);
    assert(BALL_BREAKAWAY_STATIONARY_SPEED_MAX_CM_PER_S == 0.3f);
    assert(BALL_SEQUENCE_MINUS_BREAKAWAY_ARM_SPEED_MAX_CM_PER_S == 1.0f);
    assert(BALL_BREAKAWAY_RELEASE_SPEED_CM_PER_S == 0.5f);
    assert(BALL_BREAKAWAY_RELEASE_DISPLACEMENT_CM == 0.15f);
    assert(BALL_BREAKAWAY_AWAY_DISPLACEMENT_CM == 0.15f);
    assert(BALL_BREAKAWAY_RELEASE_CONFIRM_MS == 60U);
    assert(BALL_SEQUENCE_BREAKAWAY_RELEASE_CONFIRM_MS == 30U);
    assert(BALL_BREAKAWAY_ARM_MS == 300U);
    assert(BALL_SEQUENCE_MINUS_BREAKAWAY_ARM_MS == 100U);
    assert(BALL_SEQUENCE_BREAKAWAY_ARM_PROGRESS_CM == 0.15f);
    assert(BALL_BREAKAWAY_RAMP_US_PER_S == 50.0f);
    assert(BALL_SEQUENCE_BREAKAWAY_RAMP_US_PER_S == 100.0f);
    assert(BALL_SEQUENCE_BREAKAWAY_IMMEDIATE_MAXIMUM == 1);
    assert(BALL_SEQUENCE_OVERSPEED_BRAKE_ENABLED == 0);
    assert(BALL_SEQUENCE_BRAKE_ENTER_MARGIN_CM_PER_S == 0.5f);
    assert(BALL_SEQUENCE_BRAKE_EXIT_MARGIN_CM_PER_S == 0.1f);
    assert(BALL_BREAKAWAY_MAXIMUM_US == 150.0f);
    assert(BALL_BREAKAWAY_MAXIMUM_HOLD_MS == 500U);
    assert(BALL_BREAKAWAY_SERVO_MINIMUM_US == 1400U);
    assert(BALL_SERVO_MINIMUM_US == 1200U);
    assert(BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US == 1300U);
    assert(BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US == 1200U);
    assert(BALL_SEQUENCE_BREAKAWAY_STAGE1_HOLD_MS == 250U);
    assert(BALL_SEQUENCE_BREAKAWAY_STAGE2_HOLD_MS == 250U);
    assert(BALL_BREAKAWAY_SERVO_MAXIMUM_US == 1650U);
    assert(BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US >=
        BALL_SERVO_MINIMUM_US);
    assert(((float) BALL_SERVO_CENTER_US - BALL_CONTROL_LIMIT_US) >=
        (float) BALL_SEQUENCE_BREAKAWAY_STAGE1_SERVO_MINIMUM_US);
    assert(BALL_BREAKAWAY_SERVO_MAXIMUM_US <=
        BALL_SERVO_MAXIMUM_US);
    assert(BALL_SEQUENCE_PLUS_ERROR_CM == 1.0f);
    assert(BALL_SEQUENCE_PLUS_CONFIRM_MS == 30U);
    assert(BALL_SEQUENCE_FINAL_ERROR_CM == 1.0f);
    assert(BALL_SEQUENCE_SETTLE_SPEED_MAX_CM_PER_S == 1.0f);
    assert(BALL_SEQUENCE_FINAL_SETTLE_MS == 500U);
    assert(BALL_SEQUENCE_TIMEOUT_MS == 5000U);
    test_start_gate_timeout_and_abort();
    test_manual_range_speed_test_and_control_period();
    test_speed_derivative_direction_symmetry_and_decay();
    test_direct_centimeter_input_and_safety_gates();
    test_vision_loss_and_reacquire();
    test_invalid_frame_clears_sequence_settle();
    test_sequence_plus_position_confirm_and_final_low_speed();
    test_sequence_minus_braking_capture_and_recovery();
    test_complete_sequence_and_hold();
    test_sequence_breakaway_threshold_direction_and_release();
    test_sequence_breakaway_final_tolerance_and_fault();
    test_sequence_overspeed_brake_hysteresis_and_resets();
    test_zero_center_outer_loop_direction();
    test_position_derivative_braking_symmetry();
    test_position_integral_separation_limit_and_resets();
    test_position_integral_freezes_on_inner_saturation();
    test_breakaway_arming_ramp_release_and_retrigger();
    test_breakaway_release_qualification_and_pixel_jitter();
    test_breakaway_limits_fault_and_safety_resets();
    printf("ball balance tests: PASS\n");
    return 0;
}
