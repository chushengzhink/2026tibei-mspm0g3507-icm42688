#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_balance.h"
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
    float score, uint8_t valid)
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
    memcpy(&frame[18], &score, sizeof(score));
    frame[22] = valid;
    crc = maix_crc16_ibm(frame, 26U);
    write_le16(&frame[26], crc);

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
    }
}

static void advance_frame(uint32_t elapsed_ms, uint32_t capture_ms,
    int16_t x, uint8_t valid)
{
    advance_ms(elapsed_ms);
    queue_frame(capture_ms, x, 112, valid ? 0.9f : 0.0f, valid);
    ball_balance_process();
}

static int16_t x_for_cm(float cm)
{
    float x = 20.0f + (((cm + 12.0f) / 24.0f) * 280.0f);

    return (int16_t) (x + 0.5f);
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

static uint32_t make_vision_ready(void)
{
    ball_balance_status_t status;

    advance_frame(20U, 10U, x_for_cm(0.0f), 1U);
    advance_frame(20U, 30U, x_for_cm(0.0f), 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.vision_ready);
    advance_frame(20U, 50U, x_for_cm(0.0f), 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
    return 50U;
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

    reset_controller();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    assert(ball_balance_set_target_cm(12.0f) == ML_STATUS_INVALID_ARGUMENT);
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    advance_frame(20U, capture_ms += 20U, x_for_cm(2.0f), 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.integral_cm_s > 0.0f);
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
    for (i = 0U; i < 50U; ++i) {
        advance_frame(100U, capture_ms += 100U, x_for_cm(0.0f), 1U);
    }
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_TIMEOUT);
    assert(status.sequence_elapsed_ms == 5000U);
    assert(!status.enabled);
    assert(status.servo_target_us == 1500U);
}

static void test_vision_loss_and_reacquire(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_ms(151U);
    ball_balance_process();
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_VISION_LOST);
    assert(status.state == BALL_BALANCE_VISION_LOST);
    assert(!status.enabled);
    assert(status.integral_cm_s == 0.0f);
    assert(status.servo_target_us == 1500U);

    advance_frame(20U, capture_ms += 20U, x_for_cm(0.0f), 1U);
    advance_frame(20U, capture_ms += 20U, x_for_cm(0.0f), 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(!status.vision_ready);
    advance_frame(20U, capture_ms += 20U, x_for_cm(0.0f), 1U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_ready);
    assert(!status.enabled);

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    advance_frame(20U, capture_ms + 20U, 0, 0U);
    assert(ball_balance_get_status(&status) == ML_STATUS_OK);
    assert(status.sequence_state == BALL_SEQUENCE_VISION_LOST);
    assert(!status.enabled);
}

static void test_complete_sequence_and_hold(void)
{
    ball_balance_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    const float plus_path[] = {2.0f, 4.0f, 5.0f};
    const float minus_path[] = {3.0f, 1.0f, -1.0f, -3.0f, -5.0f};

    reset_controller();
    capture_ms = make_vision_ready();
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_OK);
    for (i = 0U; i < (sizeof(plus_path) / sizeof(plus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, x_for_cm(plus_path[i]), 1U);
    }
    for (i = 0U; i < 80U; ++i) {
        advance_frame(20U, capture_ms += 20U, x_for_cm(5.0f), 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM);
    assert(fabsf(status.target_cm + 5.0f) < 0.01f);

    for (i = 0U; i < (sizeof(minus_path) / sizeof(minus_path[0])); ++i) {
        advance_frame(20U, capture_ms += 20U, x_for_cm(minus_path[i]), 1U);
    }
    for (i = 0U; i < 120U; ++i) {
        advance_frame(20U, capture_ms += 20U, x_for_cm(-5.0f), 1U);
        assert(ball_balance_get_status(&status) == ML_STATUS_OK);
        if (status.sequence_state == BALL_SEQUENCE_COMPLETE) {
            break;
        }
    }
    assert(status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(status.enabled);
    assert(status.sequence_elapsed_ms < 5000U);
    assert(fabsf(status.target_cm + 5.0f) < 0.01f);
    assert(status.servo_target_us != 1500U ||
        fabsf(status.error_cm) < 0.05f);
    assert(ball_balance_start_pm5_sequence() == ML_STATUS_BUSY);
    assert(g_last_pwm_duty >= 3250U);
    assert(g_last_pwm_duty <= 4250U);
}

int main(void)
{
    test_start_gate_timeout_and_abort();
    test_vision_loss_and_reacquire();
    test_complete_sequence_and_hold();
    printf("ball balance tests: PASS\n");
    return 0;
}
