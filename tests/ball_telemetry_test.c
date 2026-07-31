#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_telemetry.h"
#include "ml_board.h"

#define UART_CAPTURE_SIZE (131072U)

UART_Regs g_test_uart0;

static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;
static uint32_t g_uart_init_calls;

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART0);
    assert(baud == BALL_TELEMETRY_UART_BAUD);
    assert(priority == BALL_TELEMETRY_UART_PRIORITY);
    ++g_uart_init_calls;
    return ML_STATUS_OK;
}

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    assert(uart == UART0);
    if (g_uart_bytes < (UART_CAPTURE_SIZE - 1U)) {
        g_uart_capture[g_uart_bytes] = (char) byte;
        g_uart_capture[g_uart_bytes + 1U] = '\0';
    }
    ++g_uart_bytes;
    return ML_STATUS_OK;
}

static void reset_uart(void)
{
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    g_uart_bytes = 0U;
}

static void assert_csv_has_26_columns(const char *csv)
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
    assert(line_count >= 2U);
}

static ball_balance_status_t make_status(uint32_t time_ms)
{
    ball_balance_status_t status;

    memset(&status, 0, sizeof(status));
    status.uptime_ms = time_ms;
    status.state = BALL_BALANCE_ACTIVE;
    status.sequence_state = BALL_SEQUENCE_TO_PLUS_5_CM;
    status.sequence_elapsed_ms = 4321U;
    status.control_mode = BALL_CONTROL_CASCADE;
    status.enabled = true;
    status.vision_ready = true;
    status.target_cm = 0.0f;
    status.position_cm = 3.125f;
    status.error_cm = -3.125f;
    status.integral_cm_s = -4.125f;
    status.target_velocity_cm_per_s = -2.50f;
    status.velocity_cm_per_s = 1.25f;
    status.speed_error_cm_per_s = -3.75f;
    status.control_output_us = -93.5f;
    status.breakaway_active = true;
    status.brake_active = true;
    status.breakaway_boost_us = -12.5f;
    status.servo_target_us = 1594U;
    status.servo_current_us = 1580U;
    status.raw_center_x_px = 115;
    status.raw_center_y_px = 112;
    status.raw_score = 0.875f;
    status.vision_age_ms = 7U;
    status.vision_frame_interval_ms = 20U;
    return status;
}

static void test_period_csv_and_commands(void)
{
    static const char header[] =
        "time_ms,state,sequence_state,sequence_elapsed_ms,control_mode,"
        "enabled,vision_ready,target_cm,"
        "position_cm,error_cm,integral_cm_s,target_velocity_cm_s,"
        "velocity_cm_s,speed_error_cm_s,control_output_us,"
        "breakaway_active,brake_active,breakaway_boost_us,"
        "breakaway_fault,"
        "servo_target_us,"
        "servo_current_us,raw_x_px,raw_y_px,score,vision_age_ms,"
        "frame_interval_ms\r\n";
    ball_balance_status_t status = make_status(100U);

    assert(ball_telemetry_init() == ML_STATUS_OK);
    assert(g_uart_init_calls == 1U);
    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('D', true, 0U) ==
        ML_STATUS_BUFFER_EMPTY);
    assert(strcmp(g_uart_capture, "EMPTY\r\n") == 0);
    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('d', true, 0U) ==
        ML_STATUS_BUFFER_EMPTY);
    assert(strcmp(g_uart_capture, "EMPTY\r\n") == 0);
    ball_telemetry_session_start();
    assert(ball_telemetry_session_active());
    assert(ball_telemetry_record(&status) == ML_STATUS_OK);
    status.position_cm = 2.5f;
    assert(ball_telemetry_record(&status) == ML_STATUS_OK);
    assert(ball_telemetry_count() == 1U);
    status.uptime_ms = 105U;
    assert(ball_telemetry_record(&status) == ML_STATUS_BUSY);
    status.uptime_ms = 110U;
    assert(ball_telemetry_record(&status) == ML_STATUS_OK);
    status.uptime_ms = 120U;
    status.enabled = false;
    status.state = BALL_BALANCE_BREAKAWAY_FAULT;
    status.sequence_state = BALL_SEQUENCE_TIMEOUT;
    status.sequence_elapsed_ms = 5000U;
    status.control_mode = BALL_CONTROL_DISABLED;
    status.integral_cm_s = 0.0f;
    status.breakaway_active = false;
    status.brake_active = false;
    status.breakaway_boost_us = 0.0f;
    status.breakaway_fault = true;
    status.servo_target_us = 1500U;
    status.servo_current_us = 1500U;
    ball_telemetry_session_finish(&status);
    assert(!ball_telemetry_session_active());
    assert(ball_telemetry_count() == 3U);

    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('D', true, 120U) ==
        ML_STATUS_OK);
    assert(strncmp(g_uart_capture, header, sizeof(header) - 1U) == 0);
    assert(strstr(g_uart_capture,
        "100,3,1,4321,1,1,1,0.000,2.500,-3.125,-4.125,-2.50,1.25,"
        "-3.75,-93.5,1,1,-12.5,0,1594,1580,115,112,0.875,7,20\r\n") != 0);
    assert(strstr(g_uart_capture,
        "120,5,4,5000,0,0,1,0.000,2.500,-3.125,0.000,-2.50,1.25,"
        "-3.75,-93.5,0,0,0.0,1,1500,1500,115,112,0.875,7,20\r\n") != 0);
    assert_csv_has_26_columns(g_uart_capture);
    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('d', true, 120U) ==
        ML_STATUS_OK && g_uart_bytes > 0U);
    assert(ball_telemetry_uart0_handle_byte('C', true, 120U) ==
        ML_STATUS_OK);
    assert(ball_telemetry_count() == 0U && !ball_telemetry_full());
}

static void test_busy_rate_limit_and_capacity(void)
{
    ball_balance_status_t status = make_status(0U);
    const char *line_599;
    uint16_t index;

    ball_telemetry_session_start();
    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('D', true, 0U) ==
        ML_STATUS_BUSY);
    assert(strcmp(g_uart_capture, "BUSY\r\n") == 0);
    assert(ball_telemetry_uart0_handle_byte('D', false, 500U) ==
        ML_STATUS_BUSY && g_uart_bytes == 6U);
    assert(ball_telemetry_uart0_handle_byte('D', false, 1000U) ==
        ML_STATUS_BUSY && g_uart_bytes == 12U);

    for (index = 0U; index < BALL_TELEMETRY_CAPACITY; ++index) {
        status.uptime_ms = (uint32_t) index * BALL_TELEMETRY_PERIOD_MS;
        status.raw_center_x_px = (int16_t) index;
        assert(ball_telemetry_record(&status) == ML_STATUS_OK);
        if (index == (BALL_TELEMETRY_CAPACITY - 1U)) {
            status.position_cm = 2.75f;
            assert(ball_telemetry_record(&status) == ML_STATUS_OK);
            assert(ball_telemetry_count() == (index + 1U));
            status.position_cm = 3.125f;
        }
    }
    assert(BALL_TELEMETRY_CAPACITY == 600U);
    assert(ball_telemetry_count() == BALL_TELEMETRY_CAPACITY);
    assert(ball_telemetry_full());
    status.uptime_ms += BALL_TELEMETRY_PERIOD_MS;
    assert(ball_telemetry_record(&status) == ML_STATUS_BUFFER_FULL);
    ball_telemetry_session_finish(&status);

    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('D', true,
        status.uptime_ms) == ML_STATUS_OK);
    assert(g_uart_bytes < UART_CAPTURE_SIZE);
    line_599 = strstr(g_uart_capture, "\r\n5990,3,1,4321,1,1,1,");
    assert(line_599 != 0);
    assert(strstr(line_599, ",2.750,-3.125,") != 0);
    assert(strstr(line_599, ",599,112,0.875,") != 0);
    assert(strstr(g_uart_capture, "\r\n6000,3,1,4321,1,1,1,") == 0);
}

static void test_fixed_point_saturation(void)
{
    ball_balance_status_t status = make_status(0U);

    ball_telemetry_session_start();
    status.position_cm = 100.0f;
    status.error_cm = -100.0f;
    status.velocity_cm_per_s = 1000.0f;
    status.integral_cm_s = -100.0f;
    status.control_output_us = -4000.0f;
    status.breakaway_boost_us = 4000.0f;
    assert(ball_telemetry_record(&status) == ML_STATUS_OK);
    ball_telemetry_session_finish(&status);
    reset_uart();
    assert(ball_telemetry_uart0_handle_byte('D', true, 0U) ==
        ML_STATUS_OK);
    assert(strstr(g_uart_capture,
        "32.767,-32.768,-32.768,-2.50,327.67,-3.75,-3276.8,"
        "1,1,3276.7,0") != 0);
}

int main(void)
{
    test_period_csv_and_commands();
    test_busy_rate_limit_and_capacity();
    test_fixed_point_saturation();
    printf("ball telemetry tests: PASS\n");
    return 0;
}
