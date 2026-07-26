#include "headfile.h"
#include "icm42688.h"

static void icm42688_show_layout(void)
{
    (void) OLED_ShowString(1U, 1U, "ACC(g) GYR(d/s) ");
    (void) OLED_ShowString(2U, 1U, "X:+00.00 +0000  ");
    (void) OLED_ShowString(3U, 1U, "Y:+00.00 +0000  ");
    (void) OLED_ShowString(4U, 1U, "Z:+00.00 +0000  ");
}

static int32_t icm42688_round_dps(float value)
{
    return (value < 0.0f) ?
        (int32_t) (value - 0.5f) : (int32_t) (value + 0.5f);
}

static void icm42688_show_axis(
    uint8_t line, float acceleration_g, float angular_rate_dps)
{
    (void) OLED_ShowFloat(line, 3U, acceleration_g, 2U, 2U);
    (void) OLED_ShowSignedNum(
        line, 10U, icm42688_round_dps(angular_rate_dps), 4U);
}

static void icm42688_show_error(ml_status_t status, bool init_error)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U,
        init_error ? "ICM42688 INIT   " : "ICM42688 READ   ");
    (void) OLED_ShowString(2U, 1U, "STAT:00 ADDR:68 ");
    (void) OLED_ShowNum(2U, 6U, (uint32_t) status, 2U);
    (void) OLED_ShowString(3U, 1U, "SDA:PA0 SCL:PA1 ");
    (void) OLED_ShowString(4U, 1U,
        init_error ? "WHO EXPECT:47   " : "RETRYING...     ");
}

int main(void)
{
    icm42688_data_t data;
    ml_status_t status;
    bool read_error = false;

    if (system_init() != ML_STATUS_OK) {
        while (1) {
        }
    }
    if (OLED_Init() != ML_STATUS_OK) {
        (void) board_led_init();
        while (1) {
            board_led_toggle();
            delay_ms(200U);
        }
    }

    status = icm42688_init();
    if (status != ML_STATUS_OK) {
        icm42688_show_error(status, true);
        while (1) {
            delay_ms(1000U);
        }
    }

    icm42688_show_layout();
    while (1) {
        status = icm42688_read(&data);
        if (status == ML_STATUS_OK) {
            if (read_error) {
                icm42688_show_layout();
                read_error = false;
            }
            icm42688_show_axis(2U, data.accel_x_g, data.gyro_x_dps);
            icm42688_show_axis(3U, data.accel_y_g, data.gyro_y_dps);
            icm42688_show_axis(4U, data.accel_z_g, data.gyro_z_dps);
        } else if (!read_error) {
            icm42688_show_error(status, false);
            read_error = true;
        }
        delay_ms(100U);
    }
}
