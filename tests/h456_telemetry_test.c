#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "h456_telemetry.h"
#include "ml_board.h"

#define UART_CAPTURE_SIZE (262144U)

UART_Regs g_test_uart0;

static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;

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

static h456_telemetry_sample_t make_sample(uint32_t timestamp_ms,
    h456_mode_t mode)
{
    h456_telemetry_sample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.timestamp_ms = timestamp_ms;
    sample.mode = mode;
    sample.mission_state = H456_MISSION_RUNNING;
    sample.progress_mm = 1234.0f;
    sample.fused_heading_deg = 356.25f;
    sample.target_center_mm_s = 240.0f;
    sample.actual_center_mm_s = 235.5f;
    sample.pwm_left_count = 12345U;
    sample.pwm_right_count = 12001U;
    sample.line_bits = 6U;
    sample.line_usable = true;
    sample.line_correction_mm_s = -23.0f;
    sample.final_steering_bias_mm_s = -18.0f;
    sample.ball_target_cm = 0.0f;
    sample.ball_position_cm = 0.375f;
    sample.ball_error_min_cm = -0.4f;
    sample.ball_error_max_cm = 0.8f;
    sample.ball_velocity_cm_s = -1.25f;
    sample.ball_control_output_us = 42.5f;
    sample.servo_target_us = 1483U;
    sample.servo_current_us = 1491U;
    sample.raw_x_px = -42;
    sample.raw_y_px = 119;
    sample.vision_age_ms = 15U;
    sample.frame_interval_ms = 33U;
    sample.vision_ready = true;
    sample.ball_enabled = true;
    return sample;
}

static void assert_csv_has_34_columns(const char *csv)
{
    uint32_t commas = 0U;
    uint32_t lines = 0U;

    while (*csv != '\0') {
        if (*csv == ',') {
            ++commas;
        } else if (*csv == '\r') {
            assert(commas == 33U);
            commas = 0U;
            ++lines;
        }
        ++csv;
    }
    assert(lines >= 2U);
}

static void test_period_export_and_commands(void)
{
    h456_telemetry_sample_t sample = make_sample(1000U, H456_MODE_4);

    assert(h456_telemetry_init() == ML_STATUS_OK);
    assert(h456_telemetry_storage_bytes() == 26400U);
    reset_uart();
    assert(h456_telemetry_uart0_handle_byte('D', true, 0U) ==
        ML_STATUS_BUFFER_EMPTY);
    assert(strcmp(g_uart_capture, "EMPTY\r\n") == 0);

    h456_telemetry_session_start(H456_MODE_4, 1000U);
    assert(h456_telemetry_session_active());
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_OK);
    sample.timestamp_ms = 1019U;
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_BUSY);
    sample.timestamp_ms = 1020U;
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_OK);
    sample.timestamp_ms = 1010U;
    assert(h456_telemetry_record(&sample, false) ==
        ML_STATUS_INVALID_ARGUMENT);
    sample.timestamp_ms = 1030U;
    sample.mission_state = H456_MISSION_COMPLETE;
    sample.score_point_passed = true;
    sample.ball_violation = true;
    h456_telemetry_set_result(7050U, 1.125f, false);
    h456_telemetry_session_finish(&sample);
    assert(!h456_telemetry_session_active());
    assert(h456_telemetry_count() == 3U);

    reset_uart();
    assert(h456_telemetry_uart0_handle_byte('D', true, 1030U) ==
        ML_STATUS_OK);
    assert(strncmp(g_uart_capture, "time_ms,mode,mission_state,",
        strlen("time_ms,mode,mission_state,")) == 0);
    assert(strstr(g_uart_capture, "raw_x_px,raw_y_px") != 0);
    assert(strstr(g_uart_capture,
        "0,4,1,7050,0,1.125,0,1234,356.25,240.0,235.5,") != 0);
    assert(strstr(g_uart_capture,
        "1483,1491,-42,119,15,33,1,1,0,0") != 0);
    assert(strstr(g_uart_capture,
        "30,4,3,7050,0,1.125,1,1234,356.25,240.0,235.5,") != 0);
    assert_csv_has_34_columns(g_uart_capture);
    assert(h456_telemetry_uart0_handle_byte('C', true, 1030U) ==
        ML_STATUS_OK);
    assert(h456_telemetry_count() == 0U);
}

static void test_lap_period_capacity_and_terminal_replace(void)
{
    h456_telemetry_sample_t sample = make_sample(0U, H456_MODE_5);
    uint16_t i;

    assert(h456_telemetry_init() == ML_STATUS_OK);
    h456_telemetry_session_start(H456_MODE_5, 0U);
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_OK);
    sample.timestamp_ms = 49U;
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_BUSY);
    for (i = 1U; i < H456_TELEMETRY_CAPACITY; ++i) {
        sample.timestamp_ms = (uint32_t) i * 50U;
        assert(h456_telemetry_record(&sample, false) == ML_STATUS_OK);
    }
    assert(h456_telemetry_count() == H456_TELEMETRY_CAPACITY);
    assert(h456_telemetry_full());
    sample.timestamp_ms = 30000U;
    assert(h456_telemetry_record(&sample, false) ==
        ML_STATUS_BUFFER_FULL);
    sample.mission_state = H456_MISSION_COMPLETE;
    h456_telemetry_session_finish(&sample);
    assert(h456_telemetry_count() == H456_TELEMETRY_CAPACITY);
    reset_uart();
    assert(h456_telemetry_uart0_handle_byte('D', true, 30000U) ==
        ML_STATUS_OK);
    assert(strstr(g_uart_capture, "30000,5,3,") != 0);
}

static void test_busy_rate_limit(void)
{
    h456_telemetry_sample_t sample = make_sample(0U, H456_MODE_4);

    assert(h456_telemetry_init() == ML_STATUS_OK);
    h456_telemetry_session_start(H456_MODE_4, 0U);
    assert(h456_telemetry_record(&sample, false) == ML_STATUS_OK);
    reset_uart();
    assert(h456_telemetry_uart0_handle_byte('D', false, 0U) ==
        ML_STATUS_BUSY);
    assert(strcmp(g_uart_capture, "BUSY\r\n") == 0);
    reset_uart();
    assert(h456_telemetry_uart0_handle_byte('D', false, 500U) ==
        ML_STATUS_BUSY);
    assert(g_uart_bytes == 0U);
    assert(h456_telemetry_uart0_handle_byte('D', false, 1000U) ==
        ML_STATUS_BUSY);
    assert(strcmp(g_uart_capture, "BUSY\r\n") == 0);
}

int main(void)
{
    test_period_export_and_commands();
    test_lap_period_capacity_and_terminal_replace();
    test_busy_rate_limit();
    printf("H456 telemetry tests passed\n");
    return 0;
}
