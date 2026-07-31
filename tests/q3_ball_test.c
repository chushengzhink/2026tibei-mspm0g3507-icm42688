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

static void queue_frame(uint32_t capture_ms, float position_cm,
    uint8_t valid)
{
    uint8_t frame[MAIX_BALL_FRAME_SIZE];
    float score = valid ? 0.9f : 0.0f;
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
    write_le16(&frame[30], crc);
    assert((g_uart_tail + sizeof(frame)) < sizeof(g_uart_bytes));
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

ml_status_t pwm_init(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint16_t frequency_hz)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    assert(frequency_hz == Q3_SERVO_FREQUENCY_HZ);
    return ML_STATUS_OK;
}

ml_status_t pwm_update(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint32_t duty)
{
    (void) duty;
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    return ML_STATUS_OK;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART2);
    assert(baud == Q3_VISION_UART_BAUD);
    assert(priority == Q3_VISION_UART_PRIORITY);
    return ML_STATUS_OK;
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
    assert(q3_ball_init() == ML_STATUS_OK);
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
    test_formal_sequence_and_bounds();
    test_stall_uses_rock_and_safe_bounds();
    test_map_calibration_and_abort();
    test_vision_timeout_and_sequence_timeout();
    test_closed_loop_bad_mechanics();
    return 0;
}
