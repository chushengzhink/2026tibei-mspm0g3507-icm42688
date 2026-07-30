#include "chassis_self_test_view.h"

#include <stdio.h>
#include <string.h>

static int g_failures;

static void test_line(
    const char *actual, const char *expected, const char *message)
{
    if ((strlen(actual) != CHASSIS_SELF_TEST_VIEW_COLUMNS) ||
        (strcmp(actual, expected) != 0)) {
        ++g_failures;
        printf("FAIL: %s: [%s]\n", message, actual);
    }
}

int main(void)
{
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];

    chassis_self_test_view_encoder(3U, 2U, line);
    test_line(line, "L A1B1 R A1B0   ", "live AB states are formatted once");

    chassis_self_test_view_bad(12U, 12345U, line);
    test_line(line, "BAD L0012 R9999 ", "BAD counters are fixed width");

    chassis_self_test_view_tick('L', 0, line);
    test_line(line, "L +0000000      ", "zero tick delta is signed");

    chassis_self_test_view_tick('L', 1456, line);
    test_line(line, "L +0001456      ", "positive tick delta is formatted");

    chassis_self_test_view_tick('R', -1498, line);
    test_line(line, "R -0001498      ", "negative tick delta is formatted");

    chassis_self_test_view_tick('L', 10000000, line);
    test_line(line, "L +9999999      ", "positive tick delta saturates");

    chassis_self_test_view_tick('R', INT32_MIN, line);
    test_line(line, "R -9999999      ", "negative tick delta saturates");

    chassis_self_test_view_keys(1U, 0U, 1U, 0U, 1U, line);
    test_line(line, "K U1L0D1C0R1    ",
        "verified key actions follow physical channel order");

    chassis_self_test_view_sw6(1U, 0U, 1U, 0U, 1U, line);
    test_line(line, "K 12345:10101   ",
        "unmapped SW6 channels use neutral labels");

    chassis_self_test_view_calibration(27U, line);
    test_line(line, "CAL:027/300     ", "calibration progress is final text");

    chassis_self_test_view_move(34U, 101U, line);
    test_line(line, "G:034 A:101     ", "movement diagnostics are scaled");

    chassis_self_test_view_status(4U, line);
    test_line(line, "STATUS:04       ", "status is fixed width");

    chassis_self_test_view_uart_divisor(2U, 11U, line);
    test_line(line, "BRD I0002 F11   ",
        "UART hardware divisors are visible");

    chassis_self_test_view_uart_divisor(12345U, 123U, line);
    test_line(line, "BRD I9999 F99   ",
        "UART divisor display saturates");

    chassis_self_test_view_uart_tx(1U, 0U, line);
    test_line(line, "TX0001 TO000    ",
        "UART banner and timeout counters are visible");

    chassis_self_test_view_uart_tx(10000U, 1000U, line);
    test_line(line, "TX9999 TO999    ",
        "UART TX diagnostics saturate");

    chassis_self_test_view_uart_rx(false, 0U, 0U, 0U, line);
    test_line(line, "RX-- E00 O0000  ",
        "UART RX starts without a fabricated byte");

    chassis_self_test_view_uart_rx(true, 0x44U, 0x1FU, 12345U, line);
    test_line(line, "RX44 E1F O9999  ",
        "UART RX byte, errors and overflow are hexadecimal and bounded");

    chassis_self_test_view_fusion_heading(90.04f, -89.96f, line);
    test_line(line, "E+090.0 F-090.0 ",
        "encoder and fused headings share one fixed-width line");

    chassis_self_test_view_fusion_heading(181.0f, -181.0f, line);
    test_line(line, "E-179.0 F+179.0 ",
        "display headings wrap at 180 degrees");

    chassis_self_test_view_fusion_imu(-90.0f, 60.0f, line);
    test_line(line, "I-090.0 R+060.0 ",
        "raw IMU yaw and fused yaw rate retain their signs");

    chassis_self_test_view_fusion_imu(0.0f, 1200.0f, line);
    test_line(line, "I+000.0 R+999.9 ",
        "fusion rate display saturates without changing line width");

    chassis_self_test_view_fusion_status(true, 60U, line);
    test_line(line, "IMU ON  N0060   ",
        "active fusion and sample count are visible");

    chassis_self_test_view_fusion_status(false, 12000U, line);
    test_line(line, "IMU OFF N9999   ",
        "inactive fusion and saturated sample count are visible");

    if (g_failures == 0) {
        printf("PASS: chassis self-test view formatting\n");
    }
    return g_failures == 0 ? 0 : 1;
}
