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

#define UART_CAPTURE_SIZE (131072U)

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;
UART_Regs g_test_uart0;

static ball_balance_status_t g_status;
static bool g_center_pressed;
static bool g_uart_byte_pending;
static uint8_t g_uart_byte;
static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;
static char g_oled_lines[OLED_TEXT_LINE_COUNT]
    [OLED_TEXT_COLUMN_COUNT + 1U];

static void poll_count(uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; ++index) {
        ball_balance_app_poll();
    }
}

static void queue_uart_byte(uint8_t byte)
{
    assert(!g_uart_byte_pending);
    g_uart_byte = byte;
    g_uart_byte_pending = true;
}

static void reset_uart_capture(void)
{
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    g_uart_bytes = 0U;
}

static uint32_t uart_capture_checksum(void)
{
    uint32_t checksum = 2166136261UL;
    uint32_t index;

    for (index = 0U; index < g_uart_bytes; ++index) {
        checksum ^= (uint8_t) g_uart_capture[index];
        checksum *= 16777619UL;
    }
    return checksum;
}

static uint32_t assert_csv_shape(const char *csv)
{
    uint32_t comma_count = 0U;
    uint32_t line_count = 0U;

    while (*csv != '\0') {
        if (*csv == ',') {
            ++comma_count;
        } else if (*csv == '\r') {
            assert(comma_count == 25U);
            comma_count = 0U;
            ++line_count;
        }
        ++csv;
    }
    return line_count;
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
    memcpy(g_oled_lines[line - 1U],
        string, OLED_TEXT_COLUMN_COUNT + 1U);
    return ML_STATUS_OK;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART0);
    assert(baud == BALL_TELEMETRY_UART_BAUD);
    assert(priority == BALL_TELEMETRY_UART_PRIORITY);
    return ML_STATUS_OK;
}

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    assert(uart == UART0);
    assert(g_uart_bytes < (UART_CAPTURE_SIZE - 1U));
    g_uart_capture[g_uart_bytes++] = (char) byte;
    g_uart_capture[g_uart_bytes] = '\0';
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

ml_status_t ball_demo_init(void)
{
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
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    g_status.servo_current_us = BALL_SERVO_CENTER_US;
    g_status.vision_age_ms = 0xFFFFFFFFUL;
    return ML_STATUS_OK;
}

void ball_balance_process(void)
{
    g_status.uptime_ms += BALL_CONTROL_PERIOD_MS;
}

ml_status_t ball_balance_get_status(ball_balance_status_t *status)
{
    assert(status != 0);
    *status = g_status;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_start_pm5_sequence(void)
{
    if (!g_status.vision_ready || g_status.sequence_started_once) {
        return ML_STATUS_BUSY;
    }
    g_status.state = BALL_BALANCE_ACTIVE;
    g_status.sequence_state = BALL_SEQUENCE_TO_PLUS_5_CM;
    g_status.control_mode = BALL_CONTROL_CASCADE;
    g_status.sequence_started_once = true;
    g_status.enabled = true;
    g_status.target_cm = BALL_SEQUENCE_PLUS_CM;
    g_status.sequence_elapsed_ms = 0U;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_abort_sequence(void)
{
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.sequence_state = BALL_SEQUENCE_ABORTED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.enabled = false;
    g_status.servo_target_us = BALL_SERVO_CENTER_US;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_enable(bool enable)
{
    g_status.enabled = enable;
    if (!enable) {
        g_status.state = BALL_BALANCE_DISABLED;
        g_status.control_mode = BALL_CONTROL_DISABLED;
        g_status.servo_target_us = BALL_SERVO_CENTER_US;
    }
    return ML_STATUS_OK;
}

ml_status_t ball_balance_enable_speed_test(bool enable)
{
    (void) enable;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_set_target_cm(float target_cm)
{
    g_status.target_cm = target_cm;
    return ML_STATUS_OK;
}

int main(void)
{
    uint16_t record_count;
    uint32_t first_bytes;
    uint32_t first_checksum;

    memset(g_oled_lines, 0, sizeof(g_oled_lines));
    reset_uart_capture();
    assert(ball_balance_app_init() == ML_STATUS_OK);

    g_status.vision_ready = true;
    poll_count(10U);
    g_center_pressed = true;
    poll_count(8U);
    assert(g_status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM);
    assert(ball_telemetry_session_active());
    assert(ball_telemetry_count() > 0U);

    g_center_pressed = false;
    poll_count(8U);
    g_status.sequence_state = BALL_SEQUENCE_TO_MINUS_5_CM;
    g_status.target_cm = BALL_SEQUENCE_MINUS_CM;
    g_status.sequence_elapsed_ms = 2500U;
    poll_count(20U);
    assert(ball_telemetry_session_active());

    g_status.sequence_state = BALL_SEQUENCE_TIMEOUT;
    g_status.sequence_elapsed_ms = BALL_SEQUENCE_TIMEOUT_MS;
    g_status.enabled = false;
    g_status.state = BALL_BALANCE_DISABLED;
    g_status.control_mode = BALL_CONTROL_DISABLED;
    g_status.servo_target_us = 1540U;
    g_status.servo_current_us = 1540U;
    poll_count(10U);
    assert(!ball_telemetry_session_active());
    record_count = ball_telemetry_count();
    assert(record_count > 0U);
    assert(strncmp(g_oled_lines[3], "CSV N", 5U) == 0);

    reset_uart_capture();
    queue_uart_byte((uint8_t) 'D');
    poll_count(1U);
    assert(strncmp(g_uart_capture,
        "time_ms,state,sequence_state,sequence_elapsed_ms,", 49U) == 0);
    assert(assert_csv_shape(g_uart_capture) ==
        ((uint32_t) record_count + 1U));
    first_bytes = g_uart_bytes;
    first_checksum = uart_capture_checksum();

    reset_uart_capture();
    queue_uart_byte((uint8_t) 'D');
    poll_count(1U);
    assert(g_uart_bytes == first_bytes);
    assert(uart_capture_checksum() == first_checksum);

    queue_uart_byte((uint8_t) 'C');
    poll_count(1U);
    assert(ball_telemetry_count() == 0U);
    reset_uart_capture();
    queue_uart_byte((uint8_t) 'D');
    poll_count(1U);
    assert(strcmp(g_uart_capture, "EMPTY\r\n") == 0);

    printf("ball app telemetry integration tests: PASS\n");
    return 0;
}
