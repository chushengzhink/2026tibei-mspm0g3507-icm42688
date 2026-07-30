#include "chassis_self_test_view.h"

static char view_hex_digit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char) ('0' + value) :
        (char) ('A' + (value - 10U));
}

static void view_copy_literal(char line[], const char *text)
{
    uint8_t index = 0U;

    while ((index < CHASSIS_SELF_TEST_VIEW_COLUMNS) &&
        (text[index] != '\0')) {
        line[index] = text[index];
        ++index;
    }
    while (index < CHASSIS_SELF_TEST_VIEW_COLUMNS) {
        line[index] = ' ';
        ++index;
    }
    line[CHASSIS_SELF_TEST_VIEW_COLUMNS] = '\0';
}

static void view_put_unsigned(
    char line[], uint8_t column, uint32_t value, uint8_t digits)
{
    uint32_t limit = 1U;
    uint32_t divisor;
    uint8_t index;

    for (index = 0U; index < digits; ++index) {
        limit *= 10U;
    }
    if (value >= limit) {
        value = limit - 1U;
    }
    divisor = limit / 10U;
    for (index = 0U; index < digits; ++index) {
        line[column + index] =
            (char) ('0' + ((value / divisor) % 10U));
        if (divisor > 1U) {
            divisor /= 10U;
        }
    }
}

static float view_wrap_heading(float heading_deg)
{
    if (heading_deg != heading_deg) {
        return 0.0f;
    }
    while (heading_deg >= 180.0f) {
        heading_deg -= 360.0f;
    }
    while (heading_deg < -180.0f) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}

static void view_put_signed_tenths(
    char line[], uint8_t sign_column, float value)
{
    uint32_t scaled;

    if (value != value) {
        value = 0.0f;
    }
    if (value < 0.0f) {
        line[sign_column] = '-';
        value = -value;
    } else {
        line[sign_column] = '+';
    }
    if (value >= 999.9f) {
        scaled = 9999U;
    } else {
        scaled = (uint32_t) (value * 10.0f + 0.5f);
    }
    view_put_unsigned(line, (uint8_t) (sign_column + 1U),
        scaled / 10U, 3U);
    line[sign_column + 5U] = (char) ('0' + (scaled % 10U));
}

void chassis_self_test_view_encoder(
    uint8_t left_state, uint8_t right_state, char line[])
{
    view_copy_literal(line, "L A0B0 R A0B0");
    line[3] = (char) ('0' + ((left_state >> 1U) & 1U));
    line[5] = (char) ('0' + (left_state & 1U));
    line[10] = (char) ('0' + ((right_state >> 1U) & 1U));
    line[12] = (char) ('0' + (right_state & 1U));
}

void chassis_self_test_view_bad(
    uint32_t left_bad, uint32_t right_bad, char line[])
{
    view_copy_literal(line, "BAD L0000 R0000");
    view_put_unsigned(line, 5U, left_bad, 4U);
    view_put_unsigned(line, 11U, right_bad, 4U);
}

void chassis_self_test_view_tick(char wheel, int32_t ticks, char line[])
{
    uint32_t magnitude;

    view_copy_literal(line, "L +0000000");
    line[0] = wheel;
    if (ticks < 0) {
        line[2] = '-';
        magnitude = (uint32_t) (-(ticks + 1)) + 1U;
    } else {
        magnitude = (uint32_t) ticks;
    }
    view_put_unsigned(line, 3U, magnitude, 7U);
}

void chassis_self_test_view_keys(
    uint8_t up, uint8_t left, uint8_t down, uint8_t center,
    uint8_t right, char line[])
{
    view_copy_literal(line, "K U1L1D1C1R1");
    line[3] = (char) ('0' + (up & 1U));
    line[5] = (char) ('0' + (left & 1U));
    line[7] = (char) ('0' + (down & 1U));
    line[9] = (char) ('0' + (center & 1U));
    line[11] = (char) ('0' + (right & 1U));
}

void chassis_self_test_view_sw6(
    uint8_t channel_1, uint8_t channel_2, uint8_t channel_3,
    uint8_t channel_4, uint8_t channel_5, char line[])
{
    view_copy_literal(line, "K 12345:11111");
    line[8] = (char) ('0' + (channel_1 & 1U));
    line[9] = (char) ('0' + (channel_2 & 1U));
    line[10] = (char) ('0' + (channel_3 & 1U));
    line[11] = (char) ('0' + (channel_4 & 1U));
    line[12] = (char) ('0' + (channel_5 & 1U));
}

void chassis_self_test_view_calibration(uint16_t samples, char line[])
{
    view_copy_literal(line, "CAL:000/300");
    view_put_unsigned(line, 4U, samples, 3U);
}

void chassis_self_test_view_move(
    uint32_t gyro_tenths_dps, uint32_t accel_hundredths_g, char line[])
{
    view_copy_literal(line, "G:000 A:000");
    view_put_unsigned(line, 2U, gyro_tenths_dps, 3U);
    view_put_unsigned(line, 8U, accel_hundredths_g, 3U);
}

void chassis_self_test_view_status(uint32_t status, char line[])
{
    view_copy_literal(line, "STATUS:00");
    view_put_unsigned(line, 7U, status, 2U);
}

void chassis_self_test_view_uart_divisor(
    uint32_t integer_divisor, uint32_t fractional_divisor, char line[])
{
    view_copy_literal(line, "BRD I0000 F00");
    view_put_unsigned(line, 5U, integer_divisor, 4U);
    view_put_unsigned(line, 11U, fractional_divisor, 2U);
}

void chassis_self_test_view_uart_tx(
    uint32_t banners, uint32_t timeouts, char line[])
{
    view_copy_literal(line, "TX0000 TO000");
    view_put_unsigned(line, 2U, banners, 4U);
    view_put_unsigned(line, 9U, timeouts, 3U);
}

void chassis_self_test_view_uart_rx(
    bool valid, uint8_t byte, uint8_t error_flags,
    uint32_t overflows, char line[])
{
    view_copy_literal(line, "RX-- E00 O0000");
    if (valid) {
        line[2] = view_hex_digit((uint8_t) (byte >> 4U));
        line[3] = view_hex_digit(byte);
    }
    line[6] = view_hex_digit((uint8_t) (error_flags >> 4U));
    line[7] = view_hex_digit(error_flags);
    view_put_unsigned(line, 10U, overflows, 4U);
}

void chassis_self_test_view_fusion_heading(
    float encoder_heading_deg, float fused_heading_deg, char line[])
{
    view_copy_literal(line, "E+000.0 F+000.0");
    view_put_signed_tenths(line, 1U,
        view_wrap_heading(encoder_heading_deg));
    view_put_signed_tenths(line, 9U,
        view_wrap_heading(fused_heading_deg));
}

void chassis_self_test_view_fusion_imu(
    float imu_yaw_deg, float fused_yaw_rate_dps, char line[])
{
    view_copy_literal(line, "I+000.0 R+000.0");
    view_put_signed_tenths(line, 1U,
        view_wrap_heading(imu_yaw_deg));
    view_put_signed_tenths(line, 9U, fused_yaw_rate_dps);
}

void chassis_self_test_view_fusion_status(
    bool fusion_active, uint16_t sample_count, char line[])
{
    view_copy_literal(line, fusion_active ?
        "IMU ON  N0000" : "IMU OFF N0000");
    view_put_unsigned(line, 9U, sample_count, 4U);
}
