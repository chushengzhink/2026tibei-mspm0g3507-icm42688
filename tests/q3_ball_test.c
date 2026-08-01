#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "maix_ball_protocol.h"
#include "ml_pwm.h"
#include "ml_tim.h"
#include "ml_uart.h"
#include "q3_ball.h"
#include "q3_ball_config.h"

UART_Regs g_test_uart2;
GPTIMER_Regs g_test_tima1;
GPTIMER_Regs g_test_timg6;

static uint8_t g_uart_bytes[65536];
static uint32_t g_uart_head;
static uint32_t g_uart_tail;
static tim_callback_t g_tick_callback;
static void *g_tick_context;
static ml_status_t g_uart_init_status = ML_STATUS_OK;
static ml_status_t g_pwm_init_status = ML_STATUS_OK;
static ml_status_t g_pwm_update_status = ML_STATUS_OK;
static ml_status_t g_tim_init_status = ML_STATUS_OK;
static q3_core_init_stage_t g_progress_stages[8];
static uint32_t g_progress_count;

static void record_init_progress(q3_core_init_stage_t stage, void *context)
{
    (void) context;
    assert(g_progress_count <
        (sizeof(g_progress_stages) / sizeof(g_progress_stages[0])));
    g_progress_stages[g_progress_count++] = stage;
}

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

static void queue_frame_ex(uint32_t capture_ms, float position_cm,
    uint8_t valid, float score, bool corrupt_crc)
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
    memcpy(&frame[18], &position_cm, sizeof(position_cm));
    memcpy(&frame[22], &score, sizeof(score));
    frame[26] = valid;
    crc = maix_crc16_ibm(frame, 30U);
    if (corrupt_crc) {
        crc = (uint16_t) (crc ^ 0x0001U);
    }
    write_le16(&frame[30], crc);
    assert((g_uart_tail + sizeof(frame)) < sizeof(g_uart_bytes));
    for (i = 0U; i < sizeof(frame); ++i) {
        g_uart_bytes[g_uart_tail++] = frame[i];
    }
}

static void queue_frame(uint32_t capture_ms, float position_cm,
    uint8_t valid)
{
    queue_frame_ex(capture_ms, position_cm, valid,
        valid ? 0.9f : 0.0f, false);
}

static void advance_ms(uint32_t elapsed_ms)
{
    uint32_t i;

    assert(g_tick_callback != 0);
    for (i = 0U; i < elapsed_ms; ++i) {
        g_tick_callback(g_tick_context);
        q3_ball_process();
    }
}

static void advance_frame(uint32_t elapsed_ms, uint32_t *capture_ms,
    float position_cm)
{
    advance_ms(elapsed_ms);
    *capture_ms += elapsed_ms;
    queue_frame(*capture_ms, position_cm, 1U);
    q3_ball_process();
}

static void advance_frame_ex(uint32_t elapsed_ms, uint32_t *capture_ms,
    float position_cm, uint8_t valid, float score, bool corrupt_crc)
{
    advance_ms(elapsed_ms);
    *capture_ms += elapsed_ms;
    queue_frame_ex(*capture_ms, position_cm, valid, score, corrupt_crc);
    q3_ball_process();
}

ml_status_t pwm_init(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint16_t frequency_hz)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    assert(frequency_hz == Q3_SERVO_FREQUENCY_HZ);
    return g_pwm_init_status;
}

ml_status_t pwm_update(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint32_t duty)
{
    (void) duty;
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    return g_pwm_update_status;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART2);
    assert(baud == Q3_VISION_UART_BAUD);
    assert(priority == Q3_VISION_UART_PRIORITY);
    return g_uart_init_status;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART2);
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

ml_status_t tim_interrupt_ms_init_ex(GPTIMER_Regs *timer,
    uint32_t time_ms, uint8_t priority, tim_callback_t callback,
    void *context)
{
    assert(timer == TIMG6);
    assert(time_ms == 1U);
    assert(priority == Q3_TIMEBASE_PRIORITY);
    if (g_tim_init_status != ML_STATUS_OK) {
        return g_tim_init_status;
    }
    g_tick_callback = callback;
    g_tick_context = context;
    return ML_STATUS_OK;
}

static void reset_q3(void)
{
    g_uart_head = 0U;
    g_uart_tail = 0U;
    g_tick_callback = 0;
    g_tick_context = 0;
    g_uart_init_status = ML_STATUS_OK;
    g_pwm_init_status = ML_STATUS_OK;
    g_pwm_update_status = ML_STATUS_OK;
    g_tim_init_status = ML_STATUS_OK;
    assert(q3_ball_init() == ML_STATUS_OK);
}

static void test_core_init_failure_stages(void)
{
    g_uart_head = 0U;
    g_uart_tail = 0U;
    g_tick_callback = 0;
    g_tick_context = 0;
    g_progress_count = 0U;
    g_uart_init_status = ML_STATUS_BUSY;
    g_pwm_init_status = ML_STATUS_OK;
    g_pwm_update_status = ML_STATUS_OK;
    g_tim_init_status = ML_STATUS_OK;
    assert(q3_ball_init_with_progress(record_init_progress, 0) ==
        ML_STATUS_BUSY);
    assert(q3_ball_get_init_stage() == Q3_CORE_INIT_UART2);
    assert(g_progress_count == 2U);
    assert(g_progress_stages[0] == Q3_CORE_INIT_START);
    assert(g_progress_stages[1] == Q3_CORE_INIT_UART2);

    g_uart_init_status = ML_STATUS_OK;
    g_pwm_init_status = ML_STATUS_BUSY;
    g_progress_count = 0U;
    assert(q3_ball_init_with_progress(record_init_progress, 0) ==
        ML_STATUS_BUSY);
    assert(q3_ball_get_init_stage() == Q3_CORE_INIT_SERVO);

    g_pwm_init_status = ML_STATUS_OK;
    g_pwm_update_status = ML_STATUS_BUSY;
    g_progress_count = 0U;
    assert(q3_ball_init_with_progress(record_init_progress, 0) ==
        ML_STATUS_BUSY);
    assert(q3_ball_get_init_stage() == Q3_CORE_INIT_SERVO);

    g_pwm_update_status = ML_STATUS_OK;
    g_tim_init_status = ML_STATUS_TIMEOUT;
    g_progress_count = 0U;
    assert(q3_ball_init_with_progress(record_init_progress, 0) ==
        ML_STATUS_TIMEOUT);
    assert(q3_ball_get_init_stage() == Q3_CORE_INIT_TIMG6);

    g_tim_init_status = ML_STATUS_OK;
    g_progress_count = 0U;
    assert(q3_ball_init_with_progress(record_init_progress, 0) ==
        ML_STATUS_OK);
    assert(q3_ball_get_init_stage() == Q3_CORE_INIT_COMPLETE);
    assert(g_progress_count == 6U);
    assert(g_progress_stages[0] == Q3_CORE_INIT_START);
    assert(g_progress_stages[1] == Q3_CORE_INIT_UART2);
    assert(g_progress_stages[2] == Q3_CORE_INIT_SERVO);
    assert(g_progress_stages[3] == Q3_CORE_INIT_TIMG6);
    assert(g_progress_stages[4] == Q3_CORE_INIT_SAFE);
    assert(g_progress_stages[5] == Q3_CORE_INIT_COMPLETE);
}

static void test_wait_vision_valid_streak_and_boot_entry(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;

    reset_q3();
    advance_frame(20U, &capture_ms, 0.0f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.vision_valid_streak == 1U);
    assert(!status.vision_ready);

    advance_frame(20U, &capture_ms, 0.0f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.vision_valid_streak == 2U);
    assert(!status.vision_ready);

    advance_frame(20U, &capture_ms, 0.0f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_valid_streak == 3U);
    assert(status.vision_ready);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_BOOT_SETTLE);
}

static void test_wait_vision_rejects_invalid_low_score_and_bad_crc(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;
    uint32_t i;

    reset_q3();
    for (i = 0U; i < 5U; ++i) {
        advance_frame_ex(20U, &capture_ms, 0.0f, 0U, 0.0f, false);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.valid_frames == 5U);
    assert(status.vision_valid_streak == 0U);
    assert(!status.vision_ready);

    reset_q3();
    capture_ms = 0U;
    for (i = 0U; i < 5U; ++i) {
        advance_frame_ex(20U, &capture_ms, 0.0f, 1U, 0.10f, false);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.valid_frames == 5U);
    assert(status.vision_valid_streak == 0U);
    assert(status.raw_score > 0.09f && status.raw_score < 0.11f);

    reset_q3();
    capture_ms = 0U;
    for (i = 0U; i < 5U; ++i) {
        advance_frame_ex(20U, &capture_ms,
            Q3_MEASUREMENT_MAXIMUM_CM + 1.0f, 1U, 0.90f, false);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.valid_frames == 5U);
    assert(status.vision_valid_streak == 0U);

    reset_q3();
    capture_ms = 0U;
    for (i = 0U; i < 5U; ++i) {
        advance_frame_ex(20U, &capture_ms, 0.0f, 1U, 0.90f, true);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.valid_frames == 0U);
    assert(status.crc_errors == 5U);
    assert(status.vision_valid_streak == 0U);
}

static void test_wait_vision_requires_near_origin(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;
    uint32_t i;

    reset_q3();
    for (i = 0U; i < 5U; ++i) {
        advance_frame(20U, &capture_ms, 1.20f);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.vision_ready);
    assert(status.vision_valid_streak == 3U);
    assert(status.position_cm > (Q3_READY_ERROR_CM + 0.30f));
}

static void test_wait_vision_accepts_slow_valid_frames(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;

    reset_q3();
    advance_frame(300U, &capture_ms, 0.0f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.vision_valid_streak == 1U);

    advance_frame(300U, &capture_ms, 0.0f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_WAIT_VISION);
    assert(status.vision_valid_streak == 2U);
    assert(status.vision_diag == Q3_VISION_DIAG_SLOW_FRAME);
    assert(status.vision_last_diag_interval_ms == 300U);

    advance_frame(300U, &capture_ms, 0.0f);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.vision_valid_streak == 3U);
    assert(status.vision_ready);
    assert(status.state == Q3_STATE_BOOT_SETTLE);
}

static void test_boot_still_faults_on_slow_vision_timeout(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;

    reset_q3();
    advance_frame(20U, &capture_ms, 0.0f);
    advance_frame(20U, &capture_ms, 0.0f);
    advance_frame(20U, &capture_ms, 0.0f);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_BOOT_SETTLE);

    advance_ms(Q3_VISION_TIMEOUT_MS + 10U);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_VISION_FAULT);
}

static uint32_t make_ready(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;
    float position = 0.0f;
    uint32_t elapsed;

    for (elapsed = 0U; elapsed < 6000U; elapsed += 20U) {
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        switch (status.state) {
            case Q3_STATE_BOOT_PROBE_PLUS:
                position += 0.022f;
                break;
            case Q3_STATE_BOOT_RETURN_PLUS:
                position *= 0.70f;
                break;
            case Q3_STATE_BOOT_PROBE_MINUS:
                position -= 0.022f;
                break;
            case Q3_STATE_BOOT_RECENTER:
                position *= 0.55f;
                if (fabsf(position) < 0.005f) {
                    position = 0.0f;
                }
                break;
            default:
                break;
        }
        advance_frame(20U, &capture_ms, position);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_READY) {
            assert(status.profile_valid);
            assert(status.response_scale >=
                Q3_BOOT_RESPONSE_SCALE_MINIMUM - 0.001f);
            assert(status.response_scale <=
                Q3_BOOT_RESPONSE_SCALE_MAXIMUM + 0.001f);
            return capture_ms;
        }
    }
    assert(!"Q3 did not reach READY");
    return capture_ms;
}

static uint32_t start_boot_calibration(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = 0U;
    uint32_t i;

    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, 0.0f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_BOOT_SETTLE) {
            return capture_ms;
        }
    }
    assert(!"Q3 did not start boot calibration");
    return capture_ms;
}

static void advance_to_boot_state(q3_state_t target, uint32_t *capture_ms,
    float position_cm)
{
    q3_ball_status_t status;
    uint32_t i;

    for (i = 0U; i < 300U; ++i) {
        advance_frame(20U, capture_ms, position_cm);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == target) {
            return;
        }
        assert(status.state != Q3_STATE_CALIBRATION_FAULT);
    }
    assert(!"Q3 did not reach requested boot state");
}

static void test_boot_cal_fault_position_limit(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;

    reset_q3();
    capture_ms = start_boot_calibration();
    advance_frame(20U, &capture_ms, 2.0f);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_CALIBRATION_FAULT);
    assert(status.cal_fault_reason == Q3_CAL_FAULT_POSITION_LIMIT);
    assert(status.cal_fault_state == Q3_STATE_BOOT_SETTLE);
}

static void test_boot_cal_fault_plus_direction(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    uint32_t i;

    reset_q3();
    capture_ms = start_boot_calibration();
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_PLUS, &capture_ms, 0.0f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, -0.20f);
    }
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_PLUS, &capture_ms, -0.20f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, -0.45f);
        advance_ms(Q3_CONTROL_PERIOD_MS);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_CALIBRATION_FAULT) {
            break;
        }
    }
    assert(status.state == Q3_STATE_CALIBRATION_FAULT);
    assert(status.cal_fault_reason == Q3_CAL_FAULT_PLUS_DIRECTION);
    assert(status.cal_fault_state == Q3_STATE_BOOT_PROBE_PLUS);
}

static void test_boot_cal_fault_minus_direction(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    uint32_t i;

    reset_q3();
    capture_ms = start_boot_calibration();
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_PLUS, &capture_ms, 0.0f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, 0.20f);
    }
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_MINUS, &capture_ms, 0.0f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, 0.25f);
        advance_ms(Q3_CONTROL_PERIOD_MS);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_CALIBRATION_FAULT) {
            break;
        }
    }
    assert(status.state == Q3_STATE_CALIBRATION_FAULT);
    assert(status.cal_fault_reason == Q3_CAL_FAULT_MINUS_DIRECTION);
    assert(status.cal_fault_state == Q3_STATE_BOOT_PROBE_MINUS);
}

static void test_boot_cal_fault_recenter_timeout(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    uint32_t i;

    reset_q3();
    capture_ms = start_boot_calibration();
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_PLUS, &capture_ms, 0.0f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, 0.20f);
    }
    advance_to_boot_state(Q3_STATE_BOOT_PROBE_MINUS, &capture_ms, 0.0f);
    for (i = 0U; i < 10U; ++i) {
        advance_frame(20U, &capture_ms, -0.20f);
    }
    advance_to_boot_state(Q3_STATE_BOOT_RECENTER, &capture_ms, 0.70f);
    for (i = 0U; i < 260U; ++i) {
        advance_frame(20U, &capture_ms, 0.70f);
        advance_ms(Q3_CONTROL_PERIOD_MS);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_CALIBRATION_FAULT) {
            break;
        }
    }
    assert(status.state == Q3_STATE_CALIBRATION_FAULT);
    assert(status.cal_fault_reason == Q3_CAL_FAULT_RECENTER_TIMEOUT);
    assert(status.cal_fault_state == Q3_STATE_BOOT_RECENTER);
}

static void test_formal_sequence_and_bounds(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    float position;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (position = 0.0f; position < 4.25f; position += 0.20f) {
        advance_frame(20U, &capture_ms, position);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        assert(!status.plus_captured);
        assert(status.servo_target_us >= Q3_SERVO_MINIMUM_US);
        assert(status.servo_target_us <= Q3_SERVO_MAXIMUM_US);
    }
    for (position = 4.30f; position <= 5.20f; position += 0.10f) {
        advance_frame(20U, &capture_ms, position);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.plus_captured) {
            break;
        }
    }
    assert(status.plus_captured);
    assert(status.state == Q3_STATE_REVERSAL ||
        status.state == Q3_STATE_MINUS_DRIVE);

    for (position = 4.1f; position > -4.3f; position -= 0.20f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (i = 0U; i < 60U; ++i) {
        advance_frame(20U, &capture_ms, -4.75f);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_COMPLETE);
    assert(status.sequence_completed);
    assert(status.final_captured);
    assert(status.sequence_elapsed_ms <= Q3_SEQUENCE_TIMEOUT_MS);
}

static void test_stall_uses_rock_and_safe_bounds(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    bool saw_rock = false;
    bool saw_burst = false;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (i = 0U; i < 45U; ++i) {
        advance_frame(20U, &capture_ms, 0.0f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        saw_rock = saw_rock || status.rescue_stage == Q3_RESCUE_ROCK;
        saw_burst = saw_burst || status.rescue_stage == Q3_RESCUE_BURST;
        assert(status.servo_target_us >= Q3_SERVO_MINIMUM_US);
        assert(status.servo_target_us <= Q3_SERVO_MAXIMUM_US);
    }
    assert(saw_rock);
    assert(saw_burst);
    assert(status.rescue_attempts <= Q3_RESCUE_MAXIMUM_ATTEMPTS);
}

static void test_stall_ignores_pixel_velocity_jitter(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    uint32_t i;
    float position;
    bool saw_rescue = false;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (i = 0U; i < 28U; ++i) {
        position = (i & 1U) ? 0.06f : 0.0f;
        advance_frame(20U, &capture_ms, position);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        assert(status.position_cm < Q3_STALL_PROGRESS_CM);
        assert(status.servo_target_us >= Q3_SERVO_MINIMUM_US);
        assert(status.servo_target_us <= Q3_SERVO_MAXIMUM_US);
        saw_rescue = saw_rescue ||
            (status.rescue_stage != Q3_RESCUE_NONE);
    }
    assert(saw_rescue);
    assert(status.rescue_attempts > 0U);
}

static void test_plus_urgent_rescue_holds_boundary_and_releases(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    float position;
    uint32_t i;
    bool saw_hold = false;
    bool released = false;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (position = 0.0f; position < 2.25f; position += 0.10f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (i = 0U; i < 45U; ++i) {
        advance_frame(20U, &capture_ms, 2.25f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.rescue_stage == Q3_RESCUE_HOLD) {
            saw_hold = true;
            assert(status.servo_target_us == Q3_SERVO_MINIMUM_US);
            break;
        }
        assert(status.rescue_stage != Q3_RESCUE_ROCK);
    }
    assert(saw_hold);

    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, &capture_ms, 2.45f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.rescue_stage == Q3_RESCUE_NONE) {
            released = true;
            assert(status.servo_target_us > Q3_SERVO_MINIMUM_US);
            break;
        }
    }
    assert(released);
}

static void test_map_rescue_keeps_staged_sequence(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    bool saw_rock = false;
    bool saw_burst = false;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_arm_map_calibration() == ML_STATUS_OK);
    assert(q3_ball_start_map_calibration() == ML_STATUS_OK);
    for (i = 0U; i < 45U; ++i) {
        advance_frame(20U, &capture_ms, 0.0f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        saw_rock = saw_rock || status.rescue_stage == Q3_RESCUE_ROCK;
        saw_burst = saw_burst || status.rescue_stage == Q3_RESCUE_BURST;
        if (status.rescue_stage != Q3_RESCUE_NONE) {
            assert(status.rescue_stage != Q3_RESCUE_HOLD);
        }
    }
    assert(saw_rock);
    assert(saw_burst);
}

static void test_minus_rescue_keeps_staged_sequence(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    float position;
    bool saw_rock = false;
    bool saw_burst = false;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (position = 0.0f; position <= 4.35f; position += 0.20f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, &capture_ms, 4.35f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_MINUS_DRIVE) {
            break;
        }
    }
    assert(status.state == Q3_STATE_MINUS_DRIVE);

    for (i = 0U; i < 45U; ++i) {
        advance_frame(20U, &capture_ms, 4.35f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        saw_rock = saw_rock || status.rescue_stage == Q3_RESCUE_ROCK;
        saw_burst = saw_burst || status.rescue_stage == Q3_RESCUE_BURST;
        if (status.rescue_stage != Q3_RESCUE_NONE) {
            assert(status.rescue_stage != Q3_RESCUE_HOLD);
        }
    }
    assert(saw_rock);
    assert(saw_burst);
}

static uint32_t start_and_reach_minus_drive(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    float position;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (position = 0.0f; position <= 4.35f; position += 0.20f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (i = 0U; i < 20U; ++i) {
        advance_frame(20U, &capture_ms, 4.35f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_MINUS_DRIVE) {
            return capture_ms;
        }
    }
    assert(!"Q3 did not reach minus drive");
    return capture_ms;
}

static uint32_t start_and_reach_final_capture(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_minus_drive();
    uint32_t i;

    advance_frame(40U, &capture_ms, 2.00f);
    advance_frame(40U, &capture_ms, -0.50f);
    advance_frame(40U, &capture_ms, -2.80f);
    for (i = 0U; i < 160U; ++i) {
        advance_frame(20U, &capture_ms, -4.90f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_FINAL_CAPTURE) {
            return capture_ms;
        }
        assert(status.state != Q3_STATE_TIMEOUT);
    }
    assert(!"Q3 did not reach final capture");
    return capture_ms;
}

static void test_fast_final_entry_latches_final_capture(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_minus_drive();
    float position;
    bool saw_fast_band_entry = false;

    for (position = 3.5f; position > -4.8f; position -= 0.90f) {
        advance_frame(20U, &capture_ms, position);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if ((status.position_cm <= Q3_MINUS_VALID_MAXIMUM_CM) &&
            (status.position_cm >= Q3_MINUS_VALID_MINIMUM_CM) &&
            ((status.velocity_cm_per_s > Q3_FINAL_CAPTURE_ENTRY_SPEED_CM_S) ||
             (status.velocity_cm_per_s < -Q3_FINAL_CAPTURE_ENTRY_SPEED_CM_S))) {
            saw_fast_band_entry = true;
            assert(status.state == Q3_STATE_FINAL_CAPTURE);
            assert(status.final_capture_latched);
            assert(status.target_cm == Q3_FINAL_HOLD_TARGET_CM);
            assert(status.velocity_cm_per_s < 0.0f);
            assert(status.control_output_us > 0.0f);
            break;
        }
    }
    assert(saw_fast_band_entry);
}

static void test_final_capture_brakes_negative_crossing_in_place(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_final_capture();

    advance_frame(20U, &capture_ms, -5.45f);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    if (status.state != Q3_STATE_FINAL_CAPTURE) {
        fprintf(stderr, "negative rebrake state=%u x=%.3f v=%.3f u=%.1f\n",
            (unsigned) status.state, status.position_cm,
            status.velocity_cm_per_s, status.control_output_us);
    }
    assert(status.state == Q3_STATE_FINAL_CAPTURE);
    assert(status.final_capture_latched);
    assert(status.brake_active);
    assert(status.target_cm == Q3_FINAL_HOLD_TARGET_CM);
    assert(status.velocity_cm_per_s < -Q3_FINAL_REBRAKE_SPEED_CM_S);
    assert(status.control_output_us > 80.0f);
    assert(status.servo_target_us < status.neutral_us);
}

static void test_final_capture_brakes_upper_edge_bounce_in_place(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_final_capture();
    uint32_t i;
    bool saw_upper_bounce = false;

    for (i = 0U; i < 24U; ++i) {
        advance_frame(20U, &capture_ms, -5.20f + 0.08f * (float) i);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        assert(status.state == Q3_STATE_FINAL_CAPTURE);
        if ((status.position_cm > (Q3_MINUS_VALID_MAXIMUM_CM -
                Q3_FINAL_EDGE_MARGIN_CM)) &&
            (status.velocity_cm_per_s > Q3_FINAL_REBRAKE_SPEED_CM_S)) {
            saw_upper_bounce = true;
            break;
        }
    }
    assert(saw_upper_bounce);
    advance_ms(Q3_CONTROL_PERIOD_MS);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_FINAL_CAPTURE);
    assert(status.final_capture_latched);
    assert(status.brake_active);
    assert(status.target_cm == Q3_FINAL_HOLD_TARGET_CM);
    assert(status.velocity_cm_per_s > Q3_FINAL_REBRAKE_SPEED_CM_S);
    assert(status.control_output_us <= -80.0f);
    assert(status.servo_target_us > status.neutral_us);
}

static void test_final_funnel_does_not_return_to_minus_drive(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_minus_drive();
    uint32_t i;

    advance_frame(20U, &capture_ms, 0.40f);
    advance_frame(20U, &capture_ms, -1.20f);
    advance_frame(20U, &capture_ms, -2.40f);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.final_capture_latched);
    assert(status.state == Q3_STATE_FINAL_CAPTURE);
    assert(status.target_cm == Q3_FINAL_HOLD_TARGET_CM);

    for (i = 0U; i < 40U; ++i) {
        advance_frame(20U, &capture_ms, -3.60f + 0.02f * (float) i);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        assert(status.state != Q3_STATE_MINUS_DRIVE);
        assert(status.state == Q3_STATE_FINAL_CAPTURE);
        assert(status.target_cm == Q3_FINAL_HOLD_TARGET_CM);
        assert(status.final_capture_latched);
    }
}

static void test_low_speed_final_capture_still_completes(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms = start_and_reach_minus_drive();
    uint32_t i;

    advance_frame(40U, &capture_ms, 2.00f);
    advance_frame(40U, &capture_ms, -0.50f);
    advance_frame(40U, &capture_ms, -2.80f);
    advance_frame(40U, &capture_ms, -4.70f);
    for (i = 0U; i < 120U; ++i) {
        advance_frame(20U, &capture_ms, -4.70f);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_COMPLETE) {
            break;
        }
    }
    assert(status.state == Q3_STATE_COMPLETE);
    assert(status.sequence_completed);
    assert(status.final_captured);
}

static void test_map_calibration_and_abort(void)
{
    q3_ball_status_t status;
    q3_calibration_point_t point;
    uint32_t capture_ms;
    float position;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_arm_map_calibration() == ML_STATUS_OK);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_MAP_ARMED);
    assert(q3_ball_start_map_calibration() == ML_STATUS_OK);
    for (position = 0.0f; position < 5.85f; position += 0.10f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (position = 5.8f; position > -5.85f; position -= 0.10f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (position = -5.8f; position < 0.0f; position += 0.10f) {
        advance_frame(20U, &capture_ms, position);
    }
    for (i = 0U; i < 30U; ++i) {
        advance_frame(20U, &capture_ms, 0.0f);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_MAP_COMPLETE);
    assert(q3_ball_calibration_count() == Q3_PROFILE_POINT_COUNT);
    for (i = 0U; i < Q3_PROFILE_POINT_COUNT; ++i) {
        assert(q3_ball_get_calibration_point((uint8_t) i, &point) ==
            ML_STATUS_OK);
        assert(point.position_cm == -6.0f + (float) i);
        if ((point.valid_mask & 0x03U) != 0x03U) {
            fprintf(stderr, "map index=%lu mask=%u\n",
                (unsigned long) i, (unsigned) point.valid_mask);
        }
        assert((point.valid_mask & 0x03U) == 0x03U);
    }

    assert(q3_ball_abort() == ML_STATUS_OK);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_ABORTED);
    assert(status.servo_target_us >= Q3_SERVO_MINIMUM_US);
    assert(status.servo_target_us <= Q3_SERVO_MAXIMUM_US);
}

static void test_vision_timeout_and_sequence_timeout(void)
{
    q3_ball_status_t status;
    uint32_t capture_ms;
    uint32_t i;

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    advance_ms(Q3_VISION_TIMEOUT_MS + 20U);
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_VISION_FAULT);

    reset_q3();
    capture_ms = make_ready();
    assert(q3_ball_start() == ML_STATUS_OK);
    for (i = 0U; i < 260U; ++i) {
        advance_frame(20U, &capture_ms, 0.0f);
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    assert(status.state == Q3_STATE_TIMEOUT);
}

typedef struct {
    float position_cm;
    float velocity_cm_s;
    uint32_t capture_ms;
    uint32_t next_frame_ms;
    uint32_t formal_start_ms;
    bool formal_started;
    bool saw_rescue;
} q3_test_plant_t;

static float plant_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float plant_sign(float value)
{
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

static void plant_step(q3_test_plant_t *plant, bool jitter_and_drop)
{
    q3_ball_status_t status;
    float physical_command;
    float terrain_bias = 0.0f;
    float effective;
    float static_threshold;
    float rolling_threshold;
    float acceleration;
    float direction;
    bool local_trap = false;
    bool send_frame = false;
    uint32_t relative_ms = 0U;

    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    physical_command = ((float) status.servo_current_us -
        status.neutral_us) / (float) status.axis_sign;
    if ((plant->position_cm > 1.55f) &&
        (plant->position_cm < 1.95f)) {
        terrain_bias = 8.0f;
    } else if ((plant->position_cm < -1.65f) &&
               (plant->position_cm > -2.10f)) {
        terrain_bias = -7.0f;
    }
    effective = physical_command - terrain_bias;
    direction = (plant_abs(effective) > 1.0f) ?
        plant_sign(effective) : plant_sign(plant->velocity_cm_s);
    static_threshold = direction >= 0.0f ? 52.0f : 58.0f;
    rolling_threshold = direction >= 0.0f ? 25.0f : 30.0f;
    if ((direction > 0.0f) && (plant->position_cm > 1.55f) &&
        (plant->position_cm < 1.95f)) {
        static_threshold = 145.0f;
        local_trap = true;
    }
    if (local_trap && (plant_abs(effective) < static_threshold)) {
        plant->velocity_cm_s *= 0.92f;
        if (plant_abs(plant->velocity_cm_s) < 0.005f) {
            plant->velocity_cm_s = 0.0f;
        }
    } else if ((plant_abs(plant->velocity_cm_s) < 0.12f) &&
               (plant_abs(effective) < static_threshold)) {
        plant->velocity_cm_s *= 0.80f;
        if (plant_abs(plant->velocity_cm_s) < 0.005f) {
            plant->velocity_cm_s = 0.0f;
        }
    } else {
        acceleration = 0.65f * (effective -
            plant_sign(plant->velocity_cm_s == 0.0f ? effective :
                plant->velocity_cm_s) * rolling_threshold) -
            4.5f * plant->velocity_cm_s;
        plant->velocity_cm_s += acceleration * 0.001f;
    }
    plant->position_cm += plant->velocity_cm_s * 0.001f;
    if (plant->position_cm > 12.0f) {
        plant->position_cm = 12.0f;
        plant->velocity_cm_s = 0.0f;
    } else if (plant->position_cm < -12.0f) {
        plant->position_cm = -12.0f;
        plant->velocity_cm_s = 0.0f;
    }

    if (plant->formal_started) {
        relative_ms = status.uptime_ms - plant->formal_start_ms;
    }
    if (status.uptime_ms >= plant->next_frame_ms) {
        send_frame = true;
        plant->next_frame_ms = status.uptime_ms +
            (jitter_and_drop ? ((relative_ms / 20U) % 3U == 0U ?
                30U : 20U) : 20U);
        if (jitter_and_drop && plant->formal_started &&
            (relative_ms >= 1100U) && (relative_ms < 1180U)) {
            send_frame = false;
        }
    }
    g_tick_callback(g_tick_context);
    q3_ball_process();
    if (send_frame) {
        float noise = ((plant->capture_ms / 20U) & 1U) ? 0.025f : -0.025f;

        plant->capture_ms += jitter_and_drop ?
            (plant->capture_ms % 3U == 0U ? 30U : 20U) : 20U;
        queue_frame(plant->capture_ms, plant->position_cm + noise, 1U);
        q3_ball_process();
    }
    assert(q3_ball_get_status(&status) == ML_STATUS_OK);
    if (status.rescue_stage != Q3_RESCUE_NONE) {
        plant->saw_rescue = true;
    }
    assert(status.servo_target_us >= Q3_SERVO_MINIMUM_US);
    assert(status.servo_target_us <= Q3_SERVO_MAXIMUM_US);
}

static void test_closed_loop_bad_mechanics(void)
{
    q3_test_plant_t plant;
    q3_ball_status_t status;
    uint32_t elapsed;

    reset_q3();
    memset(&plant, 0, sizeof(plant));
    plant.next_frame_ms = 1U;
    for (elapsed = 0U; elapsed < 8000U; ++elapsed) {
        plant_step(&plant, true);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_READY) {
            break;
        }
    }
    assert(status.state == Q3_STATE_READY);
    assert(plant_abs(plant.position_cm) <= 0.60f);
    assert(q3_ball_start() == ML_STATUS_OK);
    plant.formal_started = true;
    plant.formal_start_ms = status.uptime_ms;
    for (elapsed = 0U; elapsed < 5100U; ++elapsed) {
        plant_step(&plant, true);
        assert(q3_ball_get_status(&status) == ML_STATUS_OK);
        if (status.state == Q3_STATE_COMPLETE) {
            break;
        }
    }
    if (status.state != Q3_STATE_COMPLETE) {
        fprintf(stderr,
            "closed-loop state=%u x=%.3f v=%.3f t=%lu rescue=%u/%u plus=%u\n",
            (unsigned) status.state, status.position_cm,
            status.velocity_cm_per_s,
            (unsigned long) status.sequence_elapsed_ms,
            (unsigned) status.rescue_stage,
            (unsigned) status.rescue_attempts,
            status.plus_captured ? 1U : 0U);
    }
    assert(status.plus_captured);
    assert(status.state == Q3_STATE_COMPLETE);
    assert(status.sequence_completed);
    assert(status.sequence_elapsed_ms <= Q3_SEQUENCE_TIMEOUT_MS);
    assert(status.position_cm >= Q3_MINUS_VALID_MINIMUM_CM);
    assert(status.position_cm <= Q3_MINUS_VALID_MAXIMUM_CM);
    assert(plant.saw_rescue);
}

int main(void)
{
    test_core_init_failure_stages();
    test_wait_vision_valid_streak_and_boot_entry();
    test_wait_vision_rejects_invalid_low_score_and_bad_crc();
    test_wait_vision_requires_near_origin();
    test_wait_vision_accepts_slow_valid_frames();
    test_boot_still_faults_on_slow_vision_timeout();
    test_boot_cal_fault_position_limit();
    test_boot_cal_fault_plus_direction();
    test_boot_cal_fault_minus_direction();
    test_boot_cal_fault_recenter_timeout();
    test_formal_sequence_and_bounds();
    test_stall_uses_rock_and_safe_bounds();
    test_stall_ignores_pixel_velocity_jitter();
    test_plus_urgent_rescue_holds_boundary_and_releases();
    test_map_rescue_keeps_staged_sequence();
    test_minus_rescue_keeps_staged_sequence();
    test_fast_final_entry_latches_final_capture();
    test_final_capture_brakes_negative_crossing_in_place();
    test_final_capture_brakes_upper_edge_bounce_in_place();
    test_final_funnel_does_not_return_to_minus_drive();
    test_low_speed_final_capture_still_completes();
    test_map_calibration_and_abort();
    test_vision_timeout_and_sequence_timeout();
    test_closed_loop_bad_mechanics();
    return 0;
}
