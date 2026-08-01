#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"
#include "q3_ball.h"
#include "q3_ball_app.h"
#include "q3_ball_config.h"
#include "q3_ball_telemetry.h"

#define UART_CAPTURE_SIZE (65536U)
#define UART_INPUT_SIZE   (128U)
#define OLED_HISTORY_SIZE (32U)

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;
UART_Regs g_test_uart0;

static q3_ball_status_t g_status;
static q3_calibration_point_t g_map[Q3_PROFILE_POINT_COUNT];
static char g_uart_capture[UART_CAPTURE_SIZE];
static uint8_t g_uart_input[UART_INPUT_SIZE];
static uint32_t g_uart_bytes;
static uint32_t g_uart_input_head;
static uint32_t g_uart_input_tail;
static uint32_t g_uart_send_fail_after;
static uint32_t g_delay_total_ms;
static uint32_t g_oled_init_calls;
static uint32_t g_oled_failures;
static uint32_t g_oled_show_calls;
static uint32_t g_oled_show_failure_call;
static char g_oled_lines[OLED_TEXT_LINE_COUNT]
    [OLED_TEXT_COLUMN_COUNT + 1U];
static char g_oled_line4_history[OLED_HISTORY_SIZE]
    [OLED_TEXT_COLUMN_COUNT + 1U];
static uint32_t g_oled_line4_history_count;
static bool g_center_pressed;
static uint32_t g_start_calls;
static uint32_t g_abort_calls;
static uint32_t g_arm_map_calls;
static uint32_t g_start_map_calls;
static bool g_telemetry_active;
static uint16_t g_telemetry_count;
static uint32_t g_telemetry_start_calls;
static uint32_t g_telemetry_finish_calls;
static uint32_t g_telemetry_storage_init_calls;
static uint32_t g_uart0_init_calls;
static uint32_t g_process_step_ms = 10U;
static uint32_t g_uart_try_read_calls;
static bool g_last_export_allowed;
static bool g_core_init_attempted;
static bool g_core_init_finished;
static bool g_uart0_init_after_core;
static ml_status_t g_telemetry_init_status;
static ml_status_t g_core_init_status;
static q3_core_init_stage_t g_core_init_stage;

static void reset_test_state(void)
{
    uint8_t index;

    memset(&g_status, 0, sizeof(g_status));
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    memset(g_uart_input, 0, sizeof(g_uart_input));
    memset(g_oled_lines, 0, sizeof(g_oled_lines));
    memset(g_oled_line4_history, 0, sizeof(g_oled_line4_history));
    memset(g_map, 0, sizeof(g_map));
    g_uart_bytes = 0U;
    g_uart_input_head = 0U;
    g_uart_input_tail = 0U;
    g_uart_send_fail_after = UINT32_MAX;
    g_delay_total_ms = 0U;
    g_oled_init_calls = 0U;
    g_oled_failures = 0U;
    g_oled_show_calls = 0U;
    g_oled_show_failure_call = 0U;
    g_oled_line4_history_count = 0U;
    g_center_pressed = false;
    g_start_calls = 0U;
    g_abort_calls = 0U;
    g_arm_map_calls = 0U;
    g_start_map_calls = 0U;
    g_telemetry_active = false;
    g_telemetry_count = 0U;
    g_telemetry_start_calls = 0U;
    g_telemetry_finish_calls = 0U;
    g_telemetry_storage_init_calls = 0U;
    g_uart0_init_calls = 0U;
    g_process_step_ms = 10U;
    g_uart_try_read_calls = 0U;
    g_last_export_allowed = false;
    g_core_init_attempted = false;
    g_core_init_finished = false;
    g_uart0_init_after_core = false;
    g_telemetry_init_status = ML_STATUS_OK;
    g_core_init_status = ML_STATUS_OK;
    g_core_init_stage = Q3_CORE_INIT_COMPLETE;
    for (index = 0U; index < Q3_PROFILE_POINT_COUNT; ++index) {
        g_map[index].position_cm = -6.0f + (float) index;
        g_map[index].rolling_plus_us = 40.0f + index;
        g_map[index].rolling_minus_us = 42.0f + index;
        g_map[index].breakaway_plus_us = 100.0f + index;
        g_map[index].breakaway_minus_us = 105.0f + index;
        g_map[index].acceleration_plus_cm_s2 = 7.0f;
        g_map[index].acceleration_minus_cm_s2 = 7.5f;
        g_map[index].valid_mask = 63U;
    }
}

static void queue_uart_byte(uint8_t byte)
{
    assert(g_uart_input_tail < UART_INPUT_SIZE);
    g_uart_input[g_uart_input_tail++] = byte;
}

static void advance_polls(uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; ++index) {
        q3_ball_app_poll();
    }
}

static void key_press(void)
{
    g_center_pressed = true;
    advance_polls(5U);
}

static void key_release(void)
{
    g_center_pressed = false;
    advance_polls(5U);
}

static void assert_uart_contains(const char *text)
{
    assert(strstr(g_uart_capture, text) != 0);
}

static bool oled_line4_history_contains(const char *text)
{
    uint32_t index;

    for (index = 0U; index < g_oled_line4_history_count; ++index) {
        if (strncmp(g_oled_line4_history[index], text,
                strlen(text)) == 0) {
            return true;
        }
    }
    return false;
}

ml_status_t board_led_init(void)
{
    return ML_STATUS_OK;
}

void board_led_off(void)
{
}

void board_led_toggle(void)
{
}

ml_status_t board_resource_claim(ml_board_resource_t resource,
    ml_board_owner_t owner)
{
    assert(resource == ML_BOARD_RESOURCE_PB24);
    assert(owner == ML_BOARD_OWNER_KEY);
    return ML_STATUS_OK;
}

void board_resource_release(ml_board_resource_t resource,
    ml_board_owner_t owner)
{
    (void) resource;
    (void) owner;
}

void delay_ms(uint32_t time_ms)
{
    g_delay_total_ms += time_ms;
}

ml_status_t gpio_init(GPIO_Regs *gpio, uint32_t pins,
    GPIOn_enum gpion, GPIO_Mode_enum mode)
{
    assert(gpio == ML_KEY_CENTER_PORT);
    assert(pins == ML_KEY_CENTER_PIN);
    assert(gpion == ML_KEY_CENTER_IOMUX);
    assert(mode == IN_UP);
    return ML_STATUS_OK;
}

uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins)
{
    assert(gpio == ML_KEY_CENTER_PORT);
    assert(pins == ML_KEY_CENTER_PIN);
    return g_center_pressed ? 0U : 1U;
}

ml_status_t OLED_Init(void)
{
    ++g_oled_init_calls;
    if (g_oled_failures > 0U) {
        --g_oled_failures;
        return ML_STATUS_NO_ACK;
    }
    return ML_STATUS_OK;
}

ml_status_t OLED_ShowLine(uint8_t line, const char *text)
{
    uint8_t index;

    assert(line >= 1U && line <= OLED_TEXT_LINE_COUNT);
    ++g_oled_show_calls;
    if (g_oled_show_calls == g_oled_show_failure_call) {
        g_oled_show_failure_call = 0U;
        return ML_STATUS_NO_ACK;
    }
    for (index = 0U; index < OLED_TEXT_COLUMN_COUNT; ++index) {
        g_oled_lines[line - 1U][index] =
            (text[index] == '\0') ? ' ' : text[index];
        if (text[index] == '\0') {
            ++index;
            while (index < OLED_TEXT_COLUMN_COUNT) {
                g_oled_lines[line - 1U][index++] = ' ';
            }
            break;
        }
    }
    g_oled_lines[line - 1U][OLED_TEXT_COLUMN_COUNT] = '\0';
    if ((line == OLED_TEXT_LINE_COUNT) &&
        (g_oled_line4_history_count < OLED_HISTORY_SIZE)) {
        memcpy(g_oled_line4_history[g_oled_line4_history_count],
            g_oled_lines[line - 1U], OLED_TEXT_COLUMN_COUNT + 1U);
        ++g_oled_line4_history_count;
    }
    return ML_STATUS_OK;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART0);
    assert(baud == Q3_TELEMETRY_UART_BAUD);
    assert(priority == Q3_TELEMETRY_UART_PRIORITY);
    return ML_STATUS_OK;
}

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    assert(uart == UART0);
    if (g_uart_bytes >= g_uart_send_fail_after) {
        return ML_STATUS_TIMEOUT;
    }
    assert(g_uart_bytes < UART_CAPTURE_SIZE - 1U);
    g_uart_capture[g_uart_bytes++] = (char) byte;
    g_uart_capture[g_uart_bytes] = '\0';
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART0);
    ++g_uart_try_read_calls;
    if (g_uart_input_head == g_uart_input_tail) {
        g_uart_input_head = 0U;
        g_uart_input_tail = 0U;
        return ML_STATUS_BUFFER_EMPTY;
    }
    *byte = g_uart_input[g_uart_input_head++];
    return ML_STATUS_OK;
}

ml_status_t q3_ball_init_with_progress(q3_core_init_progress_t progress,
    void *context)
{
    g_core_init_attempted = true;
    if (progress != 0) {
        progress(Q3_CORE_INIT_START, context);
    }
    if (g_core_init_status != ML_STATUS_OK) {
        g_core_init_finished = true;
        return g_core_init_status;
    }
    if (progress != 0) {
        progress(Q3_CORE_INIT_UART2, context);
        progress(Q3_CORE_INIT_SERVO, context);
        progress(Q3_CORE_INIT_TIMG6, context);
        progress(Q3_CORE_INIT_SAFE, context);
    }
    g_status.state = Q3_STATE_WAIT_VISION;
    g_status.mode = Q3_MODE_PROFILE;
    g_status.initialized = true;
    g_status.profile_valid = true;
    g_status.neutral_us = 1525.0f;
    g_status.response_scale = 1.0f;
    g_status.servo_target_us = 1525U;
    g_status.servo_current_us = 1525U;
    g_status.servo_settled = true;
    g_status.vision_age_ms = 0xFFFFFFFFUL;
    if (progress != 0) {
        progress(Q3_CORE_INIT_COMPLETE, context);
    }
    g_core_init_finished = true;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_init(void)
{
    return q3_ball_init_with_progress(0, 0);
}

q3_core_init_stage_t q3_ball_get_init_stage(void)
{
    return g_core_init_stage;
}

void q3_ball_process(void)
{
    g_status.uptime_ms += g_process_step_ms;
}

ml_status_t q3_ball_start(void)
{
    ++g_start_calls;
    g_status.state = Q3_STATE_PLUS_DRIVE;
    g_status.sequence_started = true;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_abort(void)
{
    ++g_abort_calls;
    g_status.state = Q3_STATE_ABORTED;
    g_status.servo_target_us = 1525U;
    g_status.servo_current_us = 1525U;
    g_status.servo_settled = true;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_get_status(q3_ball_status_t *status)
{
    *status = g_status;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_set_mode(q3_mode_t mode)
{
    if (g_status.state != Q3_STATE_READY) {
        return ML_STATUS_BUSY;
    }
    g_status.mode = mode;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_set_manual_pulse(uint16_t pulse_us)
{
    (void) pulse_us;
    return ML_STATUS_BUSY;
}

ml_status_t q3_ball_arm_map_calibration(void)
{
    if (g_status.state != Q3_STATE_READY) {
        return ML_STATUS_BUSY;
    }
    ++g_arm_map_calls;
    g_status.state = Q3_STATE_MAP_ARMED;
    return ML_STATUS_OK;
}

ml_status_t q3_ball_start_map_calibration(void)
{
    if (g_status.state != Q3_STATE_MAP_ARMED) {
        return ML_STATUS_BUSY;
    }
    ++g_start_map_calls;
    g_status.state = Q3_STATE_MAP_TO_PLUS;
    return ML_STATUS_OK;
}

uint8_t q3_ball_calibration_count(void)
{
    return Q3_PROFILE_POINT_COUNT;
}

ml_status_t q3_ball_get_calibration_point(uint8_t index,
    q3_calibration_point_t *point)
{
    if (index >= Q3_PROFILE_POINT_COUNT || point == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *point = g_map[index];
    return ML_STATUS_OK;
}

void q3_telemetry_storage_init(void)
{
    ++g_telemetry_storage_init_calls;
}

ml_status_t q3_telemetry_uart0_init(void)
{
    ++g_uart0_init_calls;
    g_uart0_init_after_core = g_core_init_attempted && g_core_init_finished;
    if (g_telemetry_init_status != ML_STATUS_OK) {
        return g_telemetry_init_status;
    }
    return uart_init(Q3_TELEMETRY_UART, Q3_TELEMETRY_UART_BAUD,
        Q3_TELEMETRY_UART_PRIORITY);
}

ml_status_t q3_telemetry_init(void)
{
    q3_telemetry_storage_init();
    return q3_telemetry_uart0_init();
}

void q3_telemetry_session_start(void)
{
    ++g_telemetry_start_calls;
    g_telemetry_active = true;
    g_telemetry_count = 0U;
}

void q3_telemetry_session_finish(const q3_ball_status_t *status)
{
    (void) status;
    ++g_telemetry_finish_calls;
    g_telemetry_active = false;
}

ml_status_t q3_telemetry_record(const q3_ball_status_t *status)
{
    (void) status;
    ++g_telemetry_count;
    return ML_STATUS_OK;
}

bool q3_telemetry_session_active(void)
{
    return g_telemetry_active;
}

uint16_t q3_telemetry_count(void)
{
    return g_telemetry_count;
}

ml_status_t q3_telemetry_uart0_handle_byte(uint8_t byte,
    bool export_allowed, uint32_t now_ms)
{
    (void) byte;
    (void) now_ms;
    g_last_export_allowed = export_allowed;
    return ML_STATUS_OK;
}

static void test_oled_retry_and_new_wait_page(void)
{
    reset_test_state();
    g_oled_failures = 2U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_telemetry_storage_init_calls == 1U);
    assert(g_uart0_init_calls == 1U);
    assert(g_uart0_init_after_core);
    assert(g_oled_init_calls == 3U);
    assert(g_delay_total_ms ==
        Q3_OLED_POWER_SETTLE_MS + 100U + 20U + 400U);
    assert_uart_contains("Q3 TERRAIN BOOT\r\n");
    assert_uart_contains("Q3 CORE OK\r\n");
    assert(oled_line4_history_contains("CORE START"));
    assert(oled_line4_history_contains("CORE UART2"));
    assert(oled_line4_history_contains("CORE SERVO"));
    assert(oled_line4_history_contains("CORE TIMG6"));
    assert(oled_line4_history_contains("CORE SAFE"));
    assert(oled_line4_history_contains("CORE OK"));
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[2], "NO VISION RX", 12U) == 0);
    assert(strncmp(g_oled_lines[3], "O000 C00 F00 L00", 16U) == 0);
}

static void test_partial_boot_page_retries_full_redraw(void)
{
    reset_test_state();
    g_oled_show_failure_call = 3U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_oled_init_calls == 2U);
    assert(g_delay_total_ms ==
        Q3_OLED_POWER_SETTLE_MS + 50U + 20U + 400U);
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[2], "NO VISION RX", 12U) == 0);
}

static void test_initial_status_page_retry_clears_wait_core(void)
{
    reset_test_state();
    g_oled_show_failure_call = 18U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[3], "CORE OK", 7U) == 0);
    advance_polls(50U);
    assert(g_oled_init_calls == 2U);
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[2], "NO VISION RX", 12U) == 0);
}

static void test_core_init_failure_shows_diagnostic_and_stays_safe(void)
{
    reset_test_state();
    g_core_init_status = ML_STATUS_TIMEOUT;
    g_core_init_stage = Q3_CORE_INIT_TIMG6;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_telemetry_storage_init_calls == 1U);
    assert(g_uart0_init_calls == 1U);
    assert(g_uart0_init_after_core);
    assert_uart_contains("Q3 CORE FAIL STAGE=TIMG6 ERR=4\r\n");
    assert(strncmp(g_oled_lines[0], "Q3 CORE FAIL", 12U) == 0);
    assert(strncmp(g_oled_lines[1], "STAGE TIMG6", 11U) == 0);
    assert(strncmp(g_oled_lines[2], "ERR 04", 6U) == 0);
    assert(strncmp(g_oled_lines[3], "CHECK UART0 LOG", 15U) == 0);

    key_press();
    key_release();
    queue_uart_byte((uint8_t) 'D');
    advance_polls(2U);
    assert(g_start_calls == 0U);
    assert(g_telemetry_start_calls == 0U);
    assert(g_last_export_allowed == false);
}

static void test_core_ok_reaches_oled_before_uart_log(void)
{
    reset_test_state();
    g_uart_send_fail_after = 1U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(oled_line4_history_contains("CORE OK"));
    assert(strncmp(g_oled_lines[2], "NO VISION RX", 12U) == 0);
}

static void test_uart0_init_failure_does_not_block_q3(void)
{
    reset_test_state();
    g_telemetry_init_status = ML_STATUS_TIMEOUT;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_telemetry_storage_init_calls == 1U);
    assert(g_uart0_init_calls == 1U);
    assert(g_uart0_init_after_core);
    assert(g_status.state == Q3_STATE_WAIT_VISION);
    assert(g_uart_bytes == 0U);
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[2], "NO VISION RX", 12U) == 0);

    queue_uart_byte((uint8_t) 'D');
    queue_uart_byte((uint8_t) 'V');
    g_status.state = Q3_STATE_READY;
    g_status.vision_ready = true;
    advance_polls(1U);
    assert(g_uart_try_read_calls == 0U);
    key_press();
    assert(g_start_calls == 1U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_uart_try_read_calls == 0U);
    assert(!g_last_export_allowed);
}

static void test_wait_vision_oled_diagnostics(void)
{
    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);

    g_status.valid_frames = 1U;
    g_status.vision_diag = Q3_VISION_DIAG_LOST;
    g_status.raw_score = 0.0f;
    g_status.raw_center_x_px = 160;
    advance_polls(10U);
    assert(strncmp(g_oled_lines[2], "BALL LOST", 9U) == 0);
    assert(strncmp(g_oled_lines[3], "RAWX+160", 8U) == 0);

    g_status.raw_score = 0.10f;
    g_status.vision_diag = Q3_VISION_DIAG_LOW_SCORE;
    g_status.raw_center_x_px = 151;
    advance_polls(10U);
    assert(strncmp(g_oled_lines[2], "LOW SCORE", 9U) == 0);
    assert(strncmp(g_oled_lines[3], "S100 RAW0151", 12U) == 0);

    g_status.raw_score = 0.90f;
    g_status.vision_diag = Q3_VISION_DIAG_WAIT_STREAK;
    g_status.vision_valid_streak = 1U;
    advance_polls(10U);
    assert(strncmp(g_oled_lines[2], "VISION 1/3", 10U) == 0);

    g_status.vision_valid_streak = 2U;
    advance_polls(10U);
    assert(strncmp(g_oled_lines[2], "VISION 2/3", 10U) == 0);

    g_status.vision_valid_streak = 3U;
    g_status.vision_diag = Q3_VISION_DIAG_MOVE_TO_O;
    g_status.vision_ready = true;
    g_status.position_cm = 1.20f;
    advance_polls(10U);
    assert(strncmp(g_oled_lines[2], "MOVE TO O", 9U) == 0);
    assert(strncmp(g_oled_lines[3], "LIMIT +-0.8CM", 13U) == 0);
}

static void test_vision_uart_command_reports_diagnostics(void)
{
    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.valid_frames = 7U;
    g_status.vision_valid_streak = 2U;
    g_status.position_cm = 1.25f;
    g_status.raw_score = 0.91f;
    g_status.raw_center_x_px = 151;
    g_status.raw_center_y_px = 113;
    g_status.vision_age_ms = 42U;
    g_status.vision_last_diag_interval_ms = 312U;
    g_status.vision_diag = Q3_VISION_DIAG_SLOW_FRAME;
    g_status.crc_errors = 3U;
    g_status.format_errors = 4U;
    g_status.length_errors = 5U;
    g_status.uart_overflows = 6U;
    queue_uart_byte((uint8_t) 'V');
    advance_polls(1U);
    assert_uart_contains("Q3 VISION state=WAIT_VISION");
    assert_uart_contains(" x=+1.25 score=0.910 ok=7 streak=2");
    assert_uart_contains(" age=42 raw=+151,+113 crc=3 fmt=4 len=5 ovf=6");
    assert_uart_contains(" dt=312 diag=SLOW_FRAME\r\n");
}

static void test_wait_vision_oled_is_throttled_but_streak_updates(void)
{
    uint32_t calls_after_init;
    uint32_t calls_after_noise;

    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.valid_frames = 1U;
    g_status.vision_diag = Q3_VISION_DIAG_WAIT_STREAK;
    g_status.vision_valid_streak = 1U;
    g_status.raw_score = 0.90f;
    g_status.raw_center_x_px = 150;
    advance_polls(1U);
    assert(strncmp(g_oled_lines[2], "VISION 1/3", 10U) == 0);
    calls_after_init = g_oled_show_calls;

    g_status.raw_score = 0.91f;
    g_status.raw_center_x_px = 151;
    advance_polls(10U);
    calls_after_noise = g_oled_show_calls;
    assert(calls_after_noise == calls_after_init);

    g_status.vision_valid_streak = 2U;
    advance_polls(1U);
    assert(g_oled_show_calls > calls_after_noise);
    assert(strncmp(g_oled_lines[2], "VISION 2/3", 10U) == 0);
}

static void test_cal_fault_reason_page_and_uart(void)
{
    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.state = Q3_STATE_CALIBRATION_FAULT;
    g_status.cal_fault_reason = Q3_CAL_FAULT_POSITION_LIMIT;
    g_status.cal_fault_state = Q3_STATE_BOOT_PROBE_PLUS;
    advance_polls(1U);
    assert_uart_contains(
        "Q3 CAL FAULT REASON=POSITION_LIMIT FROM=BOOT_PROBE_PLUS\r\n");
    assert(strncmp(g_oled_lines[0], "Q3 CAL FAULT", 12U) == 0);
    assert(strncmp(g_oled_lines[2], "WHY POS LIMIT", 13U) == 0);
    assert(strncmp(g_oled_lines[3], "CHECK UART0 LOG", 15U) == 0);

    g_status.state = Q3_STATE_WAIT_VISION;
    advance_polls(1U);
    g_status.state = Q3_STATE_CALIBRATION_FAULT;
    g_status.cal_fault_reason = Q3_CAL_FAULT_PLUS_DIRECTION;
    g_status.cal_fault_state = Q3_STATE_BOOT_PROBE_PLUS;
    advance_polls(1U);
    assert_uart_contains(
        "Q3 CAL FAULT REASON=PLUS_DIRECTION FROM=BOOT_PROBE_PLUS\r\n");
    assert(strncmp(g_oled_lines[2], "WHY PLUS DIR", 12U) == 0);
}

static void test_motion_freezes_periodic_oled_and_center_aborts(void)
{
    uint32_t calls_after_start;

    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.state = Q3_STATE_READY;
    g_status.vision_ready = true;
    advance_polls(1U);
    key_press();
    assert(g_start_calls == 1U);
    assert(g_telemetry_start_calls == 1U);
    assert(strncmp(g_oled_lines[3], "PB24 E-STOP", 11U) == 0);
    calls_after_start = g_oled_show_calls;
    g_status.position_cm = 3.0f;
    g_status.velocity_cm_per_s = 4.0f;
    advance_polls(30U);
    assert(g_oled_show_calls == calls_after_start);

    key_release();
    key_press();
    assert(g_abort_calls == 1U);
    assert(g_telemetry_finish_calls == 1U);
    assert(strncmp(g_oled_lines[0], "Q3 STOPPED", 10U) == 0);
}

static void test_final_capture_reports_hold_not_minus_drive(void)
{
    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.state = Q3_STATE_FINAL_CAPTURE;
    g_status.sequence_started = true;
    g_status.final_capture_latched = true;
    g_status.target_cm = Q3_FINAL_HOLD_TARGET_CM;
    g_status.position_cm = -5.90f;
    g_status.velocity_cm_per_s = -1.4f;
    g_status.vision_ready = true;
    advance_polls(1U);
    assert_uart_contains("Q3 CAPTURE -5\r\n");
    assert(strstr(g_uart_capture, "Q3 RUN +5 TO -5\r\n") == 0);
    assert(strncmp(g_oled_lines[0], "RUN HOLD -5", 11U) == 0);
}

static void test_formal_telemetry_is_control_period_limited(void)
{
    reset_test_state();
    g_process_step_ms = 1U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.state = Q3_STATE_PLUS_DRIVE;
    g_status.sequence_started = true;
    g_status.vision_ready = true;
    g_status.servo_settled = false;
    advance_polls(512U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_telemetry_count == 52U);
    assert(g_telemetry_finish_calls == 0U);

    g_status.state = Q3_STATE_TIMEOUT;
    g_status.servo_settled = true;
    advance_polls(1U);
    assert(g_telemetry_finish_calls == 1U);
    assert(g_telemetry_count == 53U);

    queue_uart_byte((uint8_t) 'D');
    advance_polls(1U);
    assert(g_last_export_allowed);
}

static void test_map_arm_confirmation_and_export(void)
{
    reset_test_state();
    assert(q3_ball_app_init() == ML_STATUS_OK);
    g_status.state = Q3_STATE_READY;
    g_status.vision_ready = true;
    advance_polls(1U);
    queue_uart_byte((uint8_t) 'K');
    advance_polls(1U);
    assert(g_arm_map_calls == 1U);
    assert(g_status.state == Q3_STATE_MAP_ARMED);
    assert_uart_contains("MAP ARMED PRESS PB24\r\n");
    key_press();
    assert(g_start_map_calls == 1U);
    assert(g_status.state == Q3_STATE_MAP_TO_PLUS);
    key_release();

    g_status.state = Q3_STATE_MAP_COMPLETE;
    advance_polls(1U);
    assert_uart_contains("Q3_MAP_BEGIN\r\n");
    assert_uart_contains("position_cm,balance_us,roll_plus_us");
    assert_uart_contains("Q3_MAP_END\r\n");
    assert(strncmp(g_oled_lines[0], "MAP COMPLETE", 12U) == 0);
    queue_uart_byte((uint8_t) 'L');
    advance_polls(1U);
    assert(strstr(g_uart_capture, "Q3_MAP_BEGIN\r\n") !=
        strrchr(g_uart_capture, '\0'));
}

int main(void)
{
    test_oled_retry_and_new_wait_page();
    test_partial_boot_page_retries_full_redraw();
    test_initial_status_page_retry_clears_wait_core();
    test_core_init_failure_shows_diagnostic_and_stays_safe();
    test_core_ok_reaches_oled_before_uart_log();
    test_uart0_init_failure_does_not_block_q3();
    test_wait_vision_oled_diagnostics();
    test_vision_uart_command_reports_diagnostics();
    test_wait_vision_oled_is_throttled_but_streak_updates();
    test_cal_fault_reason_page_and_uart();
    test_motion_freezes_periodic_oled_and_center_aborts();
    test_final_capture_reports_hold_not_minus_drive();
    test_formal_telemetry_is_control_period_limited();
    test_map_arm_confirmation_and_export();
    printf("q3 OLED application tests passed\n");
    return 0;
}
