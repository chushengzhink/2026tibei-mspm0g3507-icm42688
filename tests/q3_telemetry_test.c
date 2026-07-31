#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ml_board.h"
#include "q3_ball_config.h"
#include "q3_ball_telemetry.h"

#define UART_CAPTURE_SIZE (131072U)

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;
UART_Regs g_test_uart0;

static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;

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

static void reset_uart(void)
{
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    g_uart_bytes = 0U;
}

static q3_ball_status_t make_status(uint32_t time_ms)
{
    q3_ball_status_t status;

    memset(&status, 0, sizeof(status));
    status.uptime_ms = time_ms;
    status.state = Q3_STATE_MINUS_BRAKE;
    status.vision_ready = true;
    status.brake_active = true;
    status.plus_captured = true;
    status.axis_sign = -1;
    status.position_cm = -3.125f;
    status.velocity_cm_per_s = -2.5f;
    status.target_cm = -4.7f;
    status.control_output_us = 93.5f;
    status.servo_target_us = 1431U;
    status.servo_current_us = 1440U;
    status.neutral_us = 1525.0f;
    status.response_scale = 1.125f;
    status.predicted_stop_cm = -4.625f;
    status.stall_progress_cm = 0.125f;
    status.vision_age_ms = 7U;
    status.raw_score = 0.875f;
    status.profile_index = 2U;
    status.rescue_stage = Q3_RESCUE_ROCK;
    status.rescue_attempts = 1U;
    status.sequence_elapsed_ms = 3210U;
    status.vision_frame_interval_ms = 20U;
    status.raw_center_x_px = 115;
    status.raw_center_y_px = 112;
    return status;
}

static void test_csv_and_capacity(void)
{
    q3_ball_status_t status = make_status(100U);
    uint16_t index;

    assert(q3_telemetry_init() == ML_STATUS_OK);
    reset_uart();
    assert(q3_telemetry_uart0_handle_byte('D', true, 0U) ==
        ML_STATUS_BUFFER_EMPTY);
    assert(strcmp(g_uart_capture, "EMPTY\r\n") == 0);

    q3_telemetry_session_start();
    for (index = 0U; index < Q3_TELEMETRY_CAPACITY; ++index) {
        status.uptime_ms = (uint32_t) index * 10U;
        assert(q3_telemetry_record(&status) == ML_STATUS_OK);
    }
    assert(q3_telemetry_count() == 512U);
    status.uptime_ms += 10U;
    assert(q3_telemetry_record(&status) == ML_STATUS_BUFFER_FULL);
    q3_telemetry_session_finish(&status);

    reset_uart();
    assert(q3_telemetry_uart0_handle_byte('D', true,
        status.uptime_ms) == ML_STATUS_OK);
    assert(strstr(g_uart_capture,
        "response_scale,predicted_stop_cm,stall_progress_cm") != 0);
    assert(strstr(g_uart_capture,
        "100,11,-3.125,-2.50,-4.700,93.5,1431,1440,1525.0,1.125,"
        "-4.625,0.125,7,0.875,39,2,2,1,3210,20,115,112\r\n") != 0);
    assert(q3_telemetry_uart0_handle_byte('C', true,
        status.uptime_ms) == ML_STATUS_OK);
    assert(q3_telemetry_count() == 0U);
}

int main(void)
{
    test_csv_and_capacity();
    printf("q3 telemetry tests passed\n");
    return 0;
}
