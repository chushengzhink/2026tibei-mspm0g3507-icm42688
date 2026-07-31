#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_balance.h"
#include "ball_balance_app.h"
#include "ball_balance_config.h"
#include "ball_demo.h"
#include "ball_telemetry.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;
UART_Regs g_test_uart0;

static ball_balance_status_t g_status;
static bool g_center_pressed;
static uint32_t g_speed_enable_calls;
static uint32_t g_cascade_enable_calls;
static uint32_t g_disable_calls;
static uint32_t g_target_calls;
static uint32_t g_sequence_start_calls;
static uint32_t g_sequence_abort_calls;
static uint32_t g_demo_init_calls;
static bool g_telemetry_active;
static uint32_t g_telemetry_start_calls;
static uint32_t g_telemetry_record_calls;
static uint32_t g_telemetry_finish_calls;
static uint32_t g_telemetry_uart_calls;
static uint16_t g_telemetry_record_count;
static bool g_last_export_allowed;
static ball_balance_status_t g_last_telemetry_status;
static bool g_uart_byte_pending;
static uint8_t g_uart_byte;
static uint32_t g_oled_show_line_calls;
static char g_oled_lines[OLED_TEXT_LINE_COUNT]
    [OLED_TEXT_COLUMN_COUNT + 1U];

static void test_reset(bool center_pressed)
{
    memset(&g_status, 0, sizeof(g_status));
    memset(g_oled_lines, 0, sizeof(g_oled_lines));
    g_center_pressed = center_pressed;
    g_speed_enable_calls = 0U;
    g_cascade_enable_calls = 0U;
    g_disable_calls = 0U;
    g_target_calls = 0U;
    g_sequence_start_calls = 0U;
    g_sequence_abort_calls = 0U;
    g_demo_init_calls = 0U;
    g_telemetry_active = false;
    g_telemetry_start_calls = 0U;
    g_telemetry_record_calls = 0U;
    g_telemetry_finish_calls = 0U;
    g_telemetry_uart_calls = 0U;
    g_telemetry_record_count = 0U;
    g_last_export_allowed = false;
    memset(&g_last_telemetry_status, 0,
        sizeof(g_last_telemetry_status));
    g_uart_byte_pending = false;
    g_uart_byte = 0U;
    g_oled_show_line_calls = 0U;
    assert(ball_balance_app_init() == ML_STATUS_OK);
}

#if BALL_AUTO_CONTROL_MODE != BALL_AUTO_CONTROL_DISABLED
static uint32_t auto_start_call_count(void)
{
#if BALL_AUTO_CONTROL_MODE == BALL_AUTO_CONTROL_SPEED_TEST
    return g_speed_enable_calls;
#elif BALL_AUTO_CONTROL_MODE == BALL_AUTO_CONTROL_CENTER_LOOP
    return g_cascade_enable_calls;
#else
#error ball_balance_app_test requires an automatic control mode
#endif
}

static void assert_expected_running_mode(void)
{
#if BALL_AUTO_CONTROL_MODE == BALL_AUTO_CONTROL_SPEED_TEST
    assert(g_status.control_mode == BALL_CONTROL_SPEED_TEST);
    assert(g_target_calls == 0U);
#else
    assert(g_status.control_mode == BALL_CONTROL_CASCADE);
    assert(g_target_calls == g_cascade_enable_calls);
    assert(g_status.target_cm == 0.0f);
#endif
    assert(g_status.enabled);
}

static const char *expected_running_text(void)
{
#if BALL_AUTO_CONTROL_MODE == BALL_AUTO_CONTROL_SPEED_TEST
    return "CAL SPEED PD    ";
#else
    return "CAL CENTER P    ";
#endif
}
#endif

static void poll_count(uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; ++index) {
        ball_balance_app_poll();
    }
}

ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner)
{
    assert(resource == ML_BOARD_RESOURCE_PB24);
    assert(owner == ML_BOARD_OWNER_KEY);
    return ML_STATUS_OK;
}

void board_resource_release(
    ml_board_resource_t resource, ml_board_owner_t owner)
{
    (void) resource;
    (void) owner;
}

ml_status_t gpio_init(
    GPIO_Regs *gpio, uint32_t pins, GPIOn_enum gpion,
    GPIO_Mode_enum mode)
{
    assert((gpio == GPIOA) || (gpio == GPIOB));
    assert(pins != 0U);
    (void) gpion;
    assert(mode == IN_UP);
    return ML_STATUS_OK;
}

uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins)
{
    if ((gpio == ML_KEY_CENTER_PORT) &&
        (pins == ML_KEY_CENTER_PIN)) {
        return g_center_pressed ? ML_KEY_ACTIVE_LEVEL : 1U;
    }
    return 1U;
}

ml_status_t OLED_Init(void)
{
    return ML_STATUS_OK;
}

ml_status_t OLED_Clear(void)
{
    return ML_STATUS_OK;
}

ml_status_t OLED_ShowLine(uint8_t line, const char *string)
{
    assert((line >= 1U) && (line <= OLED_TEXT_LINE_COUNT));
    assert(string != 0);
    ++g_oled_show_line_calls;
    memcpy(g_oled_lines[line - 1U], string, OLED_TEXT_COLUMN_COUNT + 1U);
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART0);
    assert(byte != 0);
    if (!g_uart_byte_pending) {
        return ML_STATUS_BUFFER_EMPTY;
    }
    *byte = g_uart_byte;
    g_uart_byte_pending = false;
    return ML_STATUS_OK;
}

ml_status_t ball_telemetry_init(void)
{
    return ML_STATUS_OK;
}

void ball_telemetry_session_start(void)
{
    ++g_telemetry_start_calls;
    g_telemetry_record_count = 0U;
    g_telemetry_active = true;
}

ml_status_t ball_telemetry_record(const ball_balance_status_t *status)
{
    assert(status != 0);
    assert(g_telemetry_active);
    ++g_telemetry_record_calls;
    if (g_telemetry_record_count < BALL_TELEMETRY_CAPACITY) {
        ++g_telemetry_record_count;
    }
    g_last_telemetry_status = *status;
    return ML_STATUS_OK;
}

void ball_telemetry_session_finish(const ball_balance_status_t *status)
{
    assert(status != 0);
    assert(g_telemetry_active);
    ++g_telemetry_finish_calls;
    g_last_telemetry_status = *status;
    g_telemetry_active = false;
}

bool ball_telemetry_session_active(void)
{
    return g_telemetry_active;
}

uint16_t ball_telemetry_count(void)
{
    return g_telemetry_record_count;
}

bool ball_telemetry_full(void)
{
    return false;
}

ml_status_t ball_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms)
{
    (void) byte;
    (void) now_ms;
    ++g_telemetry_uart_calls;
    g_last_export_allowed = export_allowed;
    if (export_allowed && ((byte == (uint8_t) 'C') ||
                           (byte == (uint8_t) 'c'))) {
        g_telemetry_record_count = 0U;
    }
    return export_allowed ? ML_STATUS_OK : ML_STATUS_BUSY;
}

ml_status_t ball_demo_init(void)
{
    ++g_demo_init_calls;
    return ML_STATUS_OK;
}

void ball_demo_process(void)
{
}

ml_status_t ball_demo_short_press(void)
{
    return ML_STATUS_OK;
}

ml_status_t ball_demo_long_press(void)
{
    return ML_STATUS_OK;
}

ml_status_t ball_balance_init(void)
{
    memset(&g_status, 0, sizeof(g_status));
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.sequence_state = BALL_SEQUENCE_IDLE;
    g_status.servo_current_us = 1500U;
    g_status.servo_target_us = 1500U;
    g_status.vision_age_ms = 0xFFFFFFFFUL;
    return ML_STATUS_OK;
}

void ball_balance_process(void)
{
    g_status.uptime_ms += 10U;
}

ml_status_t ball_balance_enable_speed_test(bool enable)
{
    if (!enable) {
        return ball_balance_enable(false);
    }
    ++g_speed_enable_calls;
    if (!g_status.vision_ready) {
        return ML_STATUS_BUSY;
    }
    g_status.enabled = true;
    g_status.state = BALL_BALANCE_ACTIVE;
    g_status.control_mode = BALL_CONTROL_SPEED_TEST;
    g_status.target_velocity_cm_per_s = 0.0f;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_enable(bool enable)
{
    if (enable) {
        ++g_cascade_enable_calls;
        if (!g_status.vision_ready) {
            return ML_STATUS_BUSY;
        }
        g_status.enabled = true;
        g_status.state = BALL_BALANCE_ACTIVE;
        g_status.control_mode = BALL_CONTROL_CASCADE;
    } else {
        ++g_disable_calls;
        g_status.enabled = false;
        g_status.state = BALL_BALANCE_DISABLED;
        g_status.control_mode = BALL_CONTROL_DISABLED;
        g_status.control_output_us = 0.0f;
        g_status.servo_target_us = 1500U;
    }
    return ML_STATUS_OK;
}

ml_status_t ball_balance_set_target_cm(float target_cm)
{
    ++g_target_calls;
    assert(target_cm == 0.0f);
    g_status.target_cm = target_cm;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_start_pm5_sequence(void)
{
    ++g_sequence_start_calls;
    if (!g_status.vision_ready || g_status.sequence_started_once ||
        (g_status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) ||
        (g_status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM)) {
        return ML_STATUS_BUSY;
    }
    g_status.enabled = true;
    g_status.state = BALL_BALANCE_ACTIVE;
    g_status.control_mode = BALL_CONTROL_CASCADE;
    g_status.sequence_state = BALL_SEQUENCE_TO_PLUS_5_CM;
    g_status.sequence_started_once = true;
    g_status.target_cm = BALL_SEQUENCE_PLUS_CM;
    g_status.sequence_elapsed_ms = 0U;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_abort_sequence(void)
{
    if ((g_status.sequence_state != BALL_SEQUENCE_TO_PLUS_5_CM) &&
        (g_status.sequence_state != BALL_SEQUENCE_TO_MINUS_5_CM)) {
        return ML_STATUS_BUSY;
    }
    ++g_sequence_abort_calls;
    g_status.enabled = false;
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.sequence_state = BALL_SEQUENCE_ABORTED;
    g_status.control_output_us = 0.0f;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_get_status(ball_balance_status_t *status)
{
    assert(status != 0);
    *status = g_status;
    return ML_STATUS_OK;
}

#if BALL_AUTO_CONTROL_MODE != BALL_AUTO_CONTROL_DISABLED
static void test_auto_start_and_single_entry(void)
{
    uint32_t running_display_calls;

    test_reset(false);
    poll_count(10U);
    assert(auto_start_call_count() == 0U);
    assert(strcmp(g_oled_lines[0], "CAL AUTO WAIT   ") == 0);
    assert(g_oled_show_line_calls == OLED_TEXT_LINE_COUNT);

    g_status.vision_ready = true;
    poll_count(1U);
    assert(auto_start_call_count() == 1U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_telemetry_active);
    assert_expected_running_mode();
    assert(g_status.target_velocity_cm_per_s == 0.0f);
    assert(strcmp(g_oled_lines[0], expected_running_text()) == 0);
    assert(g_oled_show_line_calls == (2U * OLED_TEXT_LINE_COUNT));
    running_display_calls = g_oled_show_line_calls;
    poll_count(20U);
    assert(auto_start_call_count() == 1U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_oled_show_line_calls == running_display_calls);
}

static void test_pb24_abort_latches_until_reboot(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(1U);
    assert(auto_start_call_count() == 1U);

    g_center_pressed = true;
    poll_count(8U);
    assert(g_disable_calls == 1U);
    assert(!g_status.enabled);
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "CAL ABORTED     ") == 0);

    g_center_pressed = false;
    poll_count(10U);
    assert(auto_start_call_count() == 1U);
}

static void test_vision_loss_restarts_after_reacquire(void)
{
    uint32_t lost_display_calls;
    uint32_t restarted_display_calls;

    test_reset(false);
    g_status.vision_ready = true;
    poll_count(1U);
    assert(auto_start_call_count() == 1U);

    g_status.enabled = false;
    g_status.vision_ready = false;
    g_status.state = BALL_BALANCE_VISION_LOST;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.control_output_us = 0.0f;
    g_status.servo_target_us = 1500U;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "CAL REACQUIRE   ") == 0);
    lost_display_calls = g_oled_show_line_calls;

    g_status.vision_ready = true;
    poll_count(1U);
    assert(auto_start_call_count() == 2U);
    assert(g_telemetry_start_calls == 1U);
    assert_expected_running_mode();
    assert(strcmp(g_oled_lines[0], expected_running_text()) == 0);
    assert(g_oled_show_line_calls ==
        (lost_display_calls + OLED_TEXT_LINE_COUNT));
    restarted_display_calls = g_oled_show_line_calls;
    poll_count(20U);
    assert(g_oled_show_line_calls == restarted_display_calls);
}

static void test_boot_held_pb24_cancels_auto_start(void)
{
    test_reset(true);
    g_status.vision_ready = true;
    poll_count(10U);
    assert(auto_start_call_count() == 0U);
    assert(strcmp(g_oled_lines[0], "CAL ABORTED     ") == 0);
}

static void test_uart_export_waits_for_servo_center(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(1U);
    assert(g_telemetry_active);
    g_status.servo_current_us = 1700U;
    g_status.servo_target_us = 1700U;

    g_center_pressed = true;
    poll_count(8U);
    assert(!g_status.enabled);
    assert(g_status.servo_target_us == 1500U);
    assert(g_telemetry_active);
    assert(g_telemetry_finish_calls == 0U);

    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_telemetry_uart_calls == 1U);
    assert(!g_last_export_allowed);

    g_status.servo_current_us = 1500U;
    poll_count(1U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_telemetry_uart_calls == 2U);
    assert(g_last_export_allowed);
}

static void test_breakaway_fault_latches_display_and_export(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(1U);
    assert(g_telemetry_active);

    g_status.enabled = false;
    g_status.state = BALL_BALANCE_BREAKAWAY_FAULT;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.breakaway_active = false;
    g_status.breakaway_boost_us = 0.0f;
    g_status.breakaway_fault = true;
    g_status.servo_target_us = 1500U;
    g_status.servo_current_us = 1600U;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "CAL STICK FAULT ") == 0);
    assert(g_telemetry_active);
    assert(g_telemetry_finish_calls == 0U);

    g_status.servo_current_us = 1500U;
    poll_count(1U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_telemetry_uart_calls == 1U);
    assert(g_last_export_allowed);
    assert(auto_start_call_count() == 1U);
}
#else
static void assert_csv_ready_displayed(void)
{
    assert(strncmp(g_oled_lines[3], "CSV N", 5U) == 0);
    assert(strncmp(&g_oled_lines[3][8], " D=OUT", 6U) == 0);
    assert(g_telemetry_record_count > 0U);
}

static void test_formal_wait_start_complete_and_export(void)
{
    uint32_t ready_display_calls;
    uint32_t complete_display_calls;

    test_reset(false);
    poll_count(10U);
    assert(g_demo_init_calls == 0U);
    assert(g_sequence_start_calls == 0U);
    assert(g_telemetry_start_calls == 0U);
    assert(strcmp(g_oled_lines[0], "BALL WAIT VISION") == 0);

    g_status.vision_ready = true;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "BALL READY C=GO ") == 0);
    ready_display_calls = g_oled_show_line_calls;

    g_center_pressed = true;
    poll_count(8U);
    assert(g_sequence_start_calls == 1U);
    assert(g_status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(g_status.enabled);
    assert(g_status.target_cm == BALL_SEQUENCE_PLUS_CM);
    assert(g_telemetry_start_calls == 1U);
    assert(g_telemetry_active);
    assert(g_last_telemetry_status.sequence_state ==
        BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(g_oled_show_line_calls == ready_display_calls);

    g_center_pressed = false;
    poll_count(8U);
    g_status.sequence_state = BALL_SEQUENCE_TO_MINUS_5_CM;
    g_status.target_cm = BALL_SEQUENCE_MINUS_CM;
    poll_count(20U);
    assert(g_oled_show_line_calls == ready_display_calls);

    g_status.sequence_state = BALL_SEQUENCE_COMPLETE;
    g_status.sequence_elapsed_ms = 4200U;
    g_status.servo_target_us = 1540U;
    g_status.servo_current_us = 1540U;
    poll_count(1U);
    assert(strcmp(g_oled_lines[0], "BALL HOLD -5    ") == 0);
    assert(g_last_telemetry_status.sequence_state ==
        BALL_SEQUENCE_COMPLETE);
    assert(g_last_telemetry_status.sequence_elapsed_ms == 4200U);
    assert(g_oled_show_line_calls == (ready_display_calls + 1U));
    complete_display_calls = g_oled_show_line_calls;
    poll_count(20U);
    assert(g_oled_show_line_calls == complete_display_calls);
    assert(g_telemetry_active);

    g_center_pressed = true;
    poll_count(8U);
    assert(g_disable_calls == 1U);
    assert(!g_status.enabled);
    assert(g_status.sequence_state == BALL_SEQUENCE_COMPLETE);
    assert(g_status.servo_target_us == BALL_SERVO_CENTER_US);
    assert(g_telemetry_active);

    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_telemetry_uart_calls == 1U);
    assert(!g_last_export_allowed);

    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(1U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_telemetry_uart_calls == 2U);
    assert(g_last_export_allowed);
}

static void test_formal_state_driven_telemetry_and_empty_display(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    g_status.sequence_started_once = true;
    g_status.sequence_state = BALL_SEQUENCE_TO_PLUS_5_CM;
    g_status.enabled = true;
    g_status.state = BALL_BALANCE_ACTIVE;
    g_status.control_mode = BALL_CONTROL_CASCADE;
    g_status.target_cm = BALL_SEQUENCE_PLUS_CM;

    poll_count(1U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_telemetry_active);
    assert(g_telemetry_record_calls == 1U);
    assert(g_telemetry_record_count == 1U);
    poll_count(10U);
    assert(g_telemetry_start_calls == 1U);
    assert(g_telemetry_record_count == 11U);

    test_reset(false);
    g_status.sequence_started_once = true;
    g_status.sequence_state = BALL_SEQUENCE_TIMEOUT;
    g_status.enabled = false;
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(10U);
    assert(strcmp(g_oled_lines[3], "CSV EMPTY REBOOT") == 0);
    assert(g_telemetry_start_calls == 0U);
}

static void test_formal_abort_latches_single_start(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_sequence_start_calls == 1U);
    assert(g_telemetry_active);

    g_center_pressed = false;
    poll_count(8U);
    g_status.servo_target_us = 1600U;
    g_status.servo_current_us = 1600U;
    g_center_pressed = true;
    poll_count(8U);
    assert(g_sequence_abort_calls == 1U);
    assert(g_status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(!g_status.enabled);
    assert(g_telemetry_active);
    assert(g_last_telemetry_status.sequence_state ==
        BALL_SEQUENCE_ABORTED);

    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(1U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);

    g_center_pressed = false;
    poll_count(8U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_sequence_start_calls == 1U);
    assert(g_sequence_abort_calls == 1U);
}

static void test_formal_vision_reacquire_and_abort(void)
{
    uint32_t reacquire_display_calls;

    test_reset(false);
    g_status.vision_ready = true;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_telemetry_active);
    g_center_pressed = false;
    poll_count(8U);

    g_status.enabled = false;
    g_status.vision_ready = false;
    g_status.state = BALL_BALANCE_WAITING_FOR_VISION;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    g_status.servo_current_us = 1550U;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "BALL REACQUIRE  ") == 0);
    assert(g_telemetry_active);
    assert(g_telemetry_finish_calls == 0U);

    reacquire_display_calls = g_oled_show_line_calls;
    g_status.enabled = true;
    g_status.vision_ready = true;
    g_status.state = BALL_BALANCE_ACTIVE;
    g_status.control_mode = BALL_CONTROL_CASCADE;
    poll_count(1U);
    assert(strcmp(g_oled_lines[0], "BALL RUN TO +5  ") == 0);
    assert(g_oled_show_line_calls == (reacquire_display_calls + 1U));
    poll_count(20U);
    assert(g_oled_show_line_calls == (reacquire_display_calls + 1U));
    assert(g_telemetry_active);

    g_status.enabled = false;
    g_status.vision_ready = false;
    g_status.state = BALL_BALANCE_WAITING_FOR_VISION;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_sequence_abort_calls == 1U);
    assert(g_status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(g_telemetry_active);

    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(1U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
}

static void test_formal_timeout_exports_while_frozen_then_centers(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_telemetry_active);
    g_center_pressed = false;
    poll_count(8U);

    g_status.sequence_state = BALL_SEQUENCE_TIMEOUT;
    g_status.sequence_elapsed_ms = BALL_SEQUENCE_TIMEOUT_MS;
    g_status.enabled = false;
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.servo_target_us = 1540U;
    g_status.servo_current_us = 1540U;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "BALL TIMEOUT    ") == 0);
    assert(g_last_telemetry_status.sequence_state ==
        BALL_SEQUENCE_TIMEOUT);
    assert(g_last_telemetry_status.sequence_elapsed_ms ==
        BALL_SEQUENCE_TIMEOUT_MS);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    assert_csv_ready_displayed();

    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_last_export_allowed);

    g_center_pressed = true;
    poll_count(8U);
    assert(g_disable_calls == 1U);
    assert(g_status.sequence_state == BALL_SEQUENCE_TIMEOUT);
    assert(g_status.servo_target_us == BALL_SERVO_CENTER_US);
    assert(!g_telemetry_active);

    g_center_pressed = false;
    poll_count(8U);
    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(10U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    assert(strcmp(g_oled_lines[0], "BALL TIMEOUT    ") == 0);
    assert_csv_ready_displayed();

    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_last_export_allowed);
}

static void test_formal_breakaway_fault_display_and_export(void)
{
    test_reset(false);
    g_status.vision_ready = true;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_telemetry_active);

    g_status.sequence_state = BALL_SEQUENCE_ABORTED;
    g_status.sequence_elapsed_ms = 2200U;
    g_status.enabled = false;
    g_status.state = BALL_BALANCE_BREAKAWAY_FAULT;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.breakaway_active = false;
    g_status.breakaway_boost_us = 0.0f;
    g_status.breakaway_fault = true;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    g_status.servo_current_us = 1600U;
    poll_count(10U);
    assert(strcmp(g_oled_lines[0], "BALL STICK FAULT") == 0);
    assert(g_telemetry_active);
    assert(g_last_telemetry_status.sequence_state == BALL_SEQUENCE_ABORTED);
    assert(g_last_telemetry_status.breakaway_fault);
    assert(g_last_telemetry_status.sequence_elapsed_ms == 2200U);

    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    poll_count(10U);
    assert(!g_telemetry_active);
    assert(g_telemetry_finish_calls == 1U);
    assert_csv_ready_displayed();
    g_uart_byte = (uint8_t) 'D';
    g_uart_byte_pending = true;
    poll_count(1U);
    assert(g_last_export_allowed);
}

static void test_boot_hold_enters_calibration_without_sequence(void)
{
    test_reset(true);
    g_status.vision_ready = true;
    poll_count(20U);
    assert(g_demo_init_calls == 1U);
    assert(g_sequence_start_calls == 0U);
    assert(g_telemetry_start_calls == 0U);
    assert(strcmp(g_oled_lines[0], "CAL C=STEP/HOLD ") == 0);
}
#endif

int main(void)
{
#if BALL_AUTO_CONTROL_MODE != BALL_AUTO_CONTROL_DISABLED
    test_auto_start_and_single_entry();
    test_pb24_abort_latches_until_reboot();
    test_vision_loss_restarts_after_reacquire();
    test_boot_held_pb24_cancels_auto_start();
    test_uart_export_waits_for_servo_center();
    test_breakaway_fault_latches_display_and_export();
#else
    test_formal_wait_start_complete_and_export();
    test_formal_state_driven_telemetry_and_empty_display();
    test_formal_abort_latches_single_start();
    test_formal_vision_reacquire_and_abort();
    test_formal_timeout_exports_while_frozen_then_centers();
    test_formal_breakaway_fault_display_and_export();
    test_boot_hold_enters_calibration_without_sequence();
#endif
    printf("ball balance app tests: PASS\n");
    return 0;
}
