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
static uint32_t g_delay_total_ms;
static uint32_t g_oled_init_calls;
static uint32_t g_oled_failures;
static uint32_t g_oled_show_calls;
static uint32_t g_oled_show_failure_call;
static char g_oled_lines[OLED_TEXT_LINE_COUNT]
    [OLED_TEXT_COLUMN_COUNT + 1U];
static bool g_center_pressed;
static uint32_t g_start_calls;
static uint32_t g_abort_calls;
static uint32_t g_arm_map_calls;
static uint32_t g_start_map_calls;
static bool g_telemetry_active;
static uint16_t g_telemetry_count;
static uint32_t g_telemetry_start_calls;
static uint32_t g_telemetry_finish_calls;

static void reset_test_state(void)
{
    uint8_t index;

    memset(&g_status, 0, sizeof(g_status));
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    memset(g_uart_input, 0, sizeof(g_uart_input));
    memset(g_oled_lines, 0, sizeof(g_oled_lines));
    memset(g_map, 0, sizeof(g_map));
    g_uart_bytes = 0U;
    g_uart_input_head = 0U;
    g_uart_input_tail = 0U;
    g_delay_total_ms = 0U;
    g_oled_init_calls = 0U;
    g_oled_failures = 0U;
    g_oled_show_calls = 0U;
    g_oled_show_failure_call = 0U;
    g_center_pressed = false;
    g_start_calls = 0U;
    g_abort_calls = 0U;
    g_arm_map_calls = 0U;
    g_start_map_calls = 0U;
    g_telemetry_active = false;
    g_telemetry_count = 0U;
    g_telemetry_start_calls = 0U;
    g_telemetry_finish_calls = 0U;
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
    assert(g_uart_bytes < UART_CAPTURE_SIZE - 1U);
    g_uart_capture[g_uart_bytes++] = (char) byte;
    g_uart_capture[g_uart_bytes] = '\0';
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART0);
    if (g_uart_input_head == g_uart_input_tail) {
        g_uart_input_head = 0U;
        g_uart_input_tail = 0U;
        return ML_STATUS_BUFFER_EMPTY;
    }
    *byte = g_uart_input[g_uart_input_head++];
    return ML_STATUS_OK;
}

ml_status_t q3_ball_init(void)
{
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
    return ML_STATUS_OK;
}

void q3_ball_process(void)
{
    g_status.uptime_ms += 10U;
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

ml_status_t q3_telemetry_init(void)
{
    return uart_init(Q3_TELEMETRY_UART, Q3_TELEMETRY_UART_BAUD,
        Q3_TELEMETRY_UART_PRIORITY);
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
    (void) export_allowed;
    (void) now_ms;
    return ML_STATUS_OK;
}

static void test_oled_retry_and_new_wait_page(void)
{
    reset_test_state();
    g_oled_failures = 2U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_oled_init_calls == 3U);
    assert(g_delay_total_ms ==
        Q3_OLED_POWER_SETTLE_MS + 100U + 20U + 400U);
    assert_uart_contains("Q3 TERRAIN BOOT\r\n");
    assert_uart_contains("Q3 OLED FAIL ERR=");
    assert_uart_contains("Q3 OLED OK\r\n");
    assert_uart_contains("Q3 CORE OK\r\n");
    advance_polls(10U);
    assert(strncmp(g_oled_lines[0], "Q3 PLACE BALL O", 15U) == 0);
    assert(strncmp(g_oled_lines[3], "PLACE BALL AT O", 15U) == 0);
}

static void test_partial_boot_page_retries_full_redraw(void)
{
    reset_test_state();
    g_oled_show_failure_call = 3U;
    assert(q3_ball_app_init() == ML_STATUS_OK);
    assert(g_oled_init_calls == 2U);
    assert(g_oled_show_calls == 11U);
    assert(g_delay_total_ms ==
        Q3_OLED_POWER_SETTLE_MS + 50U + 20U + 400U);
    assert(strncmp(g_oled_lines[0], "Q3 TERRAIN CTRL", 15U) == 0);
    assert(strncmp(g_oled_lines[1], "OLED PB2/PB3", 12U) == 0);
    assert(strncmp(g_oled_lines[2], "SERVO 1300-1700", 15U) == 0);
    assert(strncmp(g_oled_lines[3], "WAIT CORE", 9U) == 0);
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
    test_motion_freezes_periodic_oled_and_center_aborts();
    test_map_arm_confirmation_and_export();
    printf("q3 OLED application tests passed\n");
    return 0;
}
