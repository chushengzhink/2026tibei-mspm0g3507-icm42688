#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "h5_telemetry.h"
#include "ml_board.h"

#define UART_CAPTURE_SIZE (262144U)

UART_Regs g_test_uart0;
static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    assert(uart == UART0);
    if (g_uart_bytes < UART_CAPTURE_SIZE - 1U) {
        g_uart_capture[g_uart_bytes] = (char) byte;
        g_uart_capture[g_uart_bytes + 1U] = '\0';
    }
    ++g_uart_bytes;
    return ML_STATUS_OK;
}

static h5_telemetry_sample_t sample(uint32_t timestamp_ms)
{
    h5_telemetry_sample_t value;

    memset(&value, 0, sizeof(value));
    value.timestamp_ms = timestamp_ms;
    value.mode = H5_MODE_5;
    value.mission_state = H5_MISSION_RUNNING;
    value.progress_mm = 1200.0f;
    value.fused_heading_deg = 45.0f;
    value.expected_heading_deg = 35.0f;
    value.heading_error_deg = -10.0f;
    value.target_center_mm_s = 240.0f;
    value.actual_center_mm_s = 235.0f;
    value.line_bits = 6U;
    value.line_usable = true;
    value.ball_target_cm = 0.0f;
    value.ball_position_cm = 0.25f;
    value.ball_error_min_cm = -0.2f;
    value.ball_error_max_cm = 0.3f;
    value.ball_velocity_cm_s = -0.4f;
    value.ball_control_output_us = 12.0f;
    value.servo_target_us = 1512U;
    value.servo_current_us = 1510U;
    value.raw_x_px = 160;
    value.vision_age_ms = 20U;
    value.frame_interval_ms = 20U;
    value.vision_ready = true;
    value.ball_enabled = true;
    return value;
}

int main(void)
{
    h5_telemetry_sample_t value = sample(1000U);

    assert(h5_telemetry_init() == ML_STATUS_OK);
    assert(h5_telemetry_storage_bytes() ==
        H5_TELEMETRY_CAPACITY * H5_TELEMETRY_RECORD_BYTES);
    h5_telemetry_session_start(H5_MODE_5, 1000U);
    assert(h5_telemetry_record(&value, true) == ML_STATUS_OK);
    value.timestamp_ms = 1049U;
    assert(h5_telemetry_record(&value, false) == ML_STATUS_BUSY);
    value.timestamp_ms = 1050U;
    assert(h5_telemetry_record(&value, false) == ML_STATUS_OK);
    assert(h5_telemetry_count() == 2U);
    h5_telemetry_set_result(27800U, 0.82f, true);
    h5_telemetry_session_finish(&value);
    assert(!h5_telemetry_session_active());

    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    g_uart_bytes = 0U;
    assert(h5_telemetry_uart0_handle_byte('D', true, 2000U) ==
        ML_STATUS_OK);
    assert(strstr(g_uart_capture, "time_ms,mode,mission_state") != 0);
    assert(strstr(g_uart_capture, "27800,1") != 0);
    puts("H5 telemetry tests passed");
    return 0;
}
