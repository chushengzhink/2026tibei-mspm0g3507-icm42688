#include "chassis_telemetry.h"
#include "chassis_telemetry_uart.h"
#include "ml_board.h"

#include <stdio.h>
#include <string.h>

#define UART_CAPTURE_SIZE (1024U)

static int g_failures;
static uint32_t g_export_bytes;
static uint16_t g_export_lines;
static char g_uart_capture[UART_CAPTURE_SIZE];
static uint32_t g_uart_bytes;
static uint16_t g_uart_lines;
static int32_t g_uart_fail_after = -1;
static bool g_uart_reenter_on_first;
static ml_status_t g_uart_reentry_status;

UART_Regs g_test_uart0;

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    if (uart != UART0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if ((g_uart_fail_after >= 0) &&
        (g_uart_bytes >= (uint32_t) g_uart_fail_after)) {
        return ML_STATUS_TIMEOUT;
    }
    if (g_uart_reenter_on_first && (g_uart_bytes == 0U)) {
        g_uart_reenter_on_first = false;
        g_uart_reentry_status =
            chassis_telemetry_uart0_handle_byte('D', true);
    }
    if (g_uart_bytes < (UART_CAPTURE_SIZE - 1U)) {
        g_uart_capture[g_uart_bytes] = (char) byte;
        g_uart_capture[g_uart_bytes + 1U] = '\0';
    }
    ++g_uart_bytes;
    if (byte == (uint8_t) '\n') {
        ++g_uart_lines;
    }
    return ML_STATUS_OK;
}

static void reset_uart_capture(void)
{
    memset(g_uart_capture, 0, sizeof(g_uart_capture));
    g_uart_bytes = 0U;
    g_uart_lines = 0U;
    g_uart_fail_after = -1;
    g_uart_reenter_on_first = false;
    g_uart_reentry_status = ML_STATUS_OK;
}

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static ml_status_t count_writer(
    const char *data, uint16_t length, void *context)
{
    uint16_t index;

    (void) context;
    g_export_bytes += length;
    for (index = 0U; index < length; ++index) {
        if (data[index] == '\n') {
            ++g_export_lines;
        }
    }
    return ML_STATUS_OK;
}

int main(void)
{
    static const char csv_header[] =
        "time_ms,left_ticks,right_ticks,x_mm,y_mm,"
        "encoder_heading_deg,fused_heading_deg,imu_yaw_deg,"
        "fused_yaw_rate_dps,fusion_active,elapsed_ms,lap_total_ms,"
        "target_center_mm_s,actual_center_mm_s,left_pwm_count,"
        "right_pwm_count,line_bits,line_correction_mm_s,line_usable,"
        "line_recovering,"
        "line_pattern_invalid\r\n";
    static const char two_record_csv[] =
        "time_ms,left_ticks,right_ticks,x_mm,y_mm,"
        "encoder_heading_deg,fused_heading_deg,imu_yaw_deg,"
        "fused_yaw_rate_dps,fusion_active,elapsed_ms,lap_total_ms,"
        "target_center_mm_s,actual_center_mm_s,left_pwm_count,"
        "right_pwm_count,line_bits,line_correction_mm_s,line_usable,"
        "line_recovering,"
        "line_pattern_invalid\r\n"
        "100,1,2,3.000,4.000,185.00,185.50,6.00,7.25,1,50,200,"
        "100.0,90.0,12000,11000,6,0,1,0,0\r\n"
        "200,-3,4,-1.250,2.500,360.00,360.50,45.00,-8.50,0,"
        "150,200,0.0,5.0,0,0,15,-120,0,1,1\r\n";
    chassis_telemetry_record_t record;
    uint16_t index;

    check(sizeof(chassis_telemetry_record_t) == 44U,
        "binary record is exactly 44 bytes");
    chassis_telemetry_init();
    for (index = 0U; index < CHASSIS_TELEMETRY_CAPACITY; ++index) {
        check(chassis_telemetry_record((uint32_t) index * 100U,
            index, -(int32_t) index, 1.25f, -2.5f,
            90.0f, 89.5f, -45.0f, 12.5f,
            120.5f, 119.5f, 123U, 456U, true) == ML_STATUS_OK,
            "record fits before capacity");
    }
    check(chassis_telemetry_count() == CHASSIS_TELEMETRY_CAPACITY &&
        !chassis_telemetry_overflowed(),
        "600 samples fill but do not overflow the buffer");
    check(chassis_telemetry_record(60000U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_BUFFER_FULL &&
        chassis_telemetry_overflowed(),
        "the next sample stops recording and latches overflow");
    check(chassis_telemetry_get(10U, &record) == ML_STATUS_OK &&
        record.timestamp_ms == 1000U &&
        record.encoder_heading_cdeg == 9000 &&
        record.fused_heading_cdeg == 8950 &&
        record.fused_yaw_rate_cdps == 1250 &&
        record.target_center_dmm_s == 1205 &&
        record.actual_center_dmm_s == 1195 &&
        record.pwm_left_count == 123U &&
        record.pwm_right_count == 456U &&
        record.fusion_active == 1U,
        "stored binary data can be inspected without conversion loss");
    check(chassis_telemetry_export_csv(count_writer, 0) == ML_STATUS_OK &&
        g_export_lines == CHASSIS_TELEMETRY_CAPACITY + 1U &&
        g_export_bytes > 1000U,
        "CSV export writes one header and all samples");
    chassis_telemetry_clear();
    check(chassis_telemetry_count() == 0U &&
        !chassis_telemetry_overflowed(),
        "clear resets count and overflow state");
    check(chassis_telemetry_record(0U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 400.0f, 0.0f, 0.0f,
        0U, 0U, true) == ML_STATUS_OK &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.fused_yaw_rate_cdps == INT16_MAX,
        "positive yaw rate saturates in the 0.01 dps field");
    chassis_telemetry_clear();
    check(chassis_telemetry_record(0U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, -400.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.fused_yaw_rate_cdps == INT16_MIN,
        "negative yaw rate saturates in the 0.01 dps field");
    chassis_telemetry_clear();
    chassis_telemetry_set_line_correction(200.0f);
    check(chassis_telemetry_record(0U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.line_correction_mm_s == INT8_MAX,
        "positive line correction saturates in the signed byte field");
    chassis_telemetry_clear();
    chassis_telemetry_set_line_correction(120.0f);
    check(chassis_telemetry_record(0U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.line_correction_mm_s == 120,
        "outer-single positive correction is stored exactly");
    chassis_telemetry_clear();
    chassis_telemetry_set_line_correction(-200.0f);
    check(chassis_telemetry_record(0U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.line_correction_mm_s == INT8_MIN,
        "negative line correction saturates in the signed byte field");
    chassis_telemetry_clear();
    reset_uart_capture();
    check(chassis_telemetry_uart0_handle_byte('D', true) == ML_STATUS_OK &&
        strcmp(g_uart_capture, csv_header) == 0 &&
        g_uart_bytes == (sizeof(csv_header) - 1U) &&
        g_uart_lines == 1U,
        "empty uppercase D exports one complete CRLF-terminated header");
    reset_uart_capture();
    g_uart_reenter_on_first = true;
    check(chassis_telemetry_uart0_handle_byte('D', true) == ML_STATUS_OK &&
        g_uart_reentry_status == ML_STATUS_BUSY &&
        strcmp(g_uart_capture, csv_header) == 0,
        "a reentrant D is rejected without duplicating the active export");
    chassis_telemetry_session_start(50U);
    chassis_telemetry_set_line_state(0x06U, true, false, false);
    chassis_telemetry_set_line_correction(42.9f);
    check(chassis_telemetry_record(100U, 1, 2, 3.0f, 4.0f,
        5.0f, 5.5f, 6.0f, 7.25f, 100.0f, 90.0f,
        12000U, 11000U, true) == ML_STATUS_OK,
        "first UART command test record is stored");
    chassis_telemetry_set_line_correction(0.0f);
    check(chassis_telemetry_record(100U, 1, 2, 3.0f, 4.0f,
        185.0f, 185.5f, 6.0f, 7.25f, 100.0f, 90.0f,
        12000U, 11000U, true) == ML_STATUS_OK &&
        chassis_telemetry_count() == 1U &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.encoder_heading_cdeg == 18500 &&
        record.fused_heading_cdeg == 18550 &&
        record.line_correction_mm_s == 0 &&
        (record.line_state_flags & CHASSIS_TELEMETRY_LINE_USABLE) != 0U,
        "same-timestamp stop snapshot replaces instead of duplicating");
    reset_uart_capture();
    check(chassis_uart0_send_busy() == ML_STATUS_OK &&
        strcmp(g_uart_capture, "BUSY\r\n") == 0 &&
        chassis_telemetry_count() == 1U,
        "race busy reply is short and leaves RAM unchanged");
    reset_uart_capture();
    check(chassis_uart0_send_diagnostic_banner(1U) == ML_STATUS_OK &&
        strcmp(g_uart_capture, "UART0,115200,TX,0001\r\n") == 0 &&
        chassis_telemetry_count() == 1U,
        "diagnostic banner is complete and does not export or clear RAM");
    reset_uart_capture();
    check(chassis_uart0_send_diagnostic_banner(10000U) == ML_STATUS_OK &&
        strcmp(g_uart_capture, "UART0,115200,TX,9999\r\n") == 0,
        "diagnostic banner sequence saturates at four digits");
    reset_uart_capture();
    check(chassis_uart0_send_diagnostic_rx(0x44U, 0x1FU) ==
        ML_STATUS_OK &&
        strcmp(g_uart_capture, "RX,44,ERR,1F\r\n") == 0 &&
        chassis_telemetry_count() == 1U,
        "diagnostic RX reply is hexadecimal and leaves RAM unchanged");
    reset_uart_capture();
    g_uart_fail_after = 5;
    check(chassis_uart0_send_diagnostic_banner(2U) ==
        ML_STATUS_TIMEOUT && g_uart_bytes == 5U,
        "diagnostic banner propagates TX failure without continuing");
    chassis_telemetry_set_line_state(0x0FU, false, true, true);
    chassis_telemetry_set_line_correction(-120.0f);
    check(chassis_telemetry_record(200U, -3, 4, -1.25f, 2.5f,
        360.0f, 360.5f, 45.0f, -8.5f, 0.0f, 5.0f,
        0U, 0U, false) == ML_STATUS_OK,
        "second UART record preserves cumulative headings past 360 degrees");
    chassis_telemetry_session_finish(250U);
    reset_uart_capture();
    check(chassis_telemetry_uart0_handle_byte('D', true) == ML_STATUS_OK &&
        strcmp(g_uart_capture, two_record_csv) == 0 &&
        g_uart_bytes == (sizeof(two_record_csv) - 1U) &&
        g_uart_lines == 3U,
        "single uppercase D exports ordered complete CSV lines");
    reset_uart_capture();
    check(chassis_telemetry_uart0_handle_byte('d', true) == ML_STATUS_OK &&
        strcmp(g_uart_capture, two_record_csv) == 0 &&
        g_uart_lines == 3U,
        "single lowercase d repeats the same complete CSV");
    reset_uart_capture();
    check(chassis_telemetry_uart0_handle_byte('D', false) ==
        ML_STATUS_BUSY && g_uart_bytes == 0U,
        "export is rejected while the chassis is moving");
    check(chassis_telemetry_uart0_handle_byte('C', false) ==
        ML_STATUS_BUSY && chassis_telemetry_count() == 2U,
        "clear is rejected while running or braking");
    check(chassis_telemetry_uart0_handle_byte('C', true) == ML_STATUS_OK &&
        chassis_telemetry_count() == 0U,
        "single uppercase C clears telemetry while stopped");
    check(chassis_telemetry_record(200U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK &&
        chassis_telemetry_uart0_handle_byte('c', true) == ML_STATUS_OK &&
        chassis_telemetry_count() == 0U,
        "single lowercase c also clears telemetry");
    check(chassis_telemetry_uart0_handle_byte('X', true) ==
        ML_STATUS_INVALID_ARGUMENT,
        "unknown UART commands are rejected");
    check(chassis_telemetry_record(300U, 0, 0, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0U, 0U, false) == ML_STATUS_OK,
        "UART failure test record is stored");
    reset_uart_capture();
    g_uart_fail_after = 5;
    check(chassis_telemetry_uart0_handle_byte('D', true) ==
        ML_STATUS_TIMEOUT && g_uart_bytes == 5U,
        "UART send failures stop CSV export and propagate the status");
    if (g_failures == 0) {
        printf("chassis telemetry tests passed\n");
    }
    return g_failures == 0 ? 0 : 1;
}
