#include "headfile.h"
#include "icm42688.h"
#include "imu_attitude.h"

#define ATTITUDE_SAMPLE_PERIOD_MS  (10U)
#define ATTITUDE_DISPLAY_PERIOD_MS (100U)
#define ATTITUDE_DT_MAX_MS         (50U)

static volatile uint32_t g_attitude_milliseconds;

/* Body frame is NWU: X forward, Y left, Z up. The installed sensor has
 * +Y forward and +X left, so its right-handed +Z axis points down. */
static const imu_attitude_config_t g_body_axis_config = {
    {
        IMU_ATTITUDE_AXIS_Y,
        IMU_ATTITUDE_AXIS_X,
        IMU_ATTITUDE_AXIS_Z
    },
    {1, 1, -1}
};

static void attitude_timer_callback(void *context)
{
    (void) context;
    ++g_attitude_milliseconds;
}

static ml_status_t attitude_verify_timer_tick(void)
{
    uint32_t before_ms = g_attitude_milliseconds;

    delay_ms(20U);
    return (g_attitude_milliseconds != before_ms) ?
        ML_STATUS_OK : ML_STATUS_TIMEOUT;
}

static uint32_t attitude_wait_for_sample(uint32_t previous_ms)
{
    uint32_t now_ms;

    do {
        now_ms = g_attitude_milliseconds;
        if ((uint32_t) (now_ms - previous_ms) <
            ATTITUDE_SAMPLE_PERIOD_MS) {
            delay_ms(1U);
        }
    } while ((uint32_t) (now_ms - previous_ms) <
        ATTITUDE_SAMPLE_PERIOD_MS);
    return now_ms;
}

static void attitude_show_error(const char *title, ml_status_t status)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, title);
    (void) OLED_ShowString(2U, 1U, "STAT:00         ");
    (void) OLED_ShowNum(2U, 6U, (uint32_t) status, 2U);
    (void) OLED_ShowString(3U, 1U, "SDA:PA0 SCL:PA1 ");
    (void) OLED_ShowString(4U, 1U, "CHECK AND RETRY ");
}

static void attitude_show_calibration_layout(void)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, "KEEP STILL      ");
    (void) OLED_ShowString(2U, 1U, "CAL:000/300     ");
    (void) OLED_ShowString(3U, 1U, "GYRO ZERO BIAS  ");
    (void) OLED_ShowString(4U, 1U, "WAIT ABOUT 3 SEC");
}

static void attitude_show_calibration_progress(
    const imu_attitude_t *attitude, bool restarted)
{
    (void) OLED_ShowNum(2U, 5U,
        (uint32_t) imu_attitude_calibration_progress(attitude), 3U);
    (void) OLED_ShowString(4U, 1U,
        restarted ? "MOVED - RETRY   " : "WAIT ABOUT 3 SEC");
}

static void attitude_show_layout(void)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, "ATTITUDE REL DEG");
    (void) OLED_ShowString(2U, 1U, "P:+000.0 DEG    ");
    (void) OLED_ShowString(3U, 1U, "R:+000.0 DEG    ");
    (void) OLED_ShowString(4U, 1U, "Y:+000.0 DEG    ");
}

static void attitude_show_angles(const imu_attitude_angles_t *angles)
{
    (void) OLED_ShowFloat(2U, 3U, angles->pitch_deg, 3U, 1U);
    (void) OLED_ShowFloat(3U, 3U, angles->roll_deg, 3U, 1U);
    (void) OLED_ShowFloat(4U, 3U, angles->yaw_deg, 3U, 1U);
}

static ml_status_t attitude_calibrate(imu_attitude_t *attitude)
{
    icm42688_data_t sample;
    uint32_t last_sample_ms = g_attitude_milliseconds;
    uint32_t last_display_ms = last_sample_ms;
    bool read_error = false;

    attitude_show_calibration_layout();
    while (1) {
        uint32_t now_ms = attitude_wait_for_sample(last_sample_ms);
        imu_attitude_calibration_status_t calibration_status;
        ml_status_t status;

        last_sample_ms = now_ms;
        status = icm42688_read(&sample);
        if (status != ML_STATUS_OK) {
            if (!read_error) {
                attitude_show_error("ICM42688 READ   ", status);
                read_error = true;
            }
            continue;
        }
        if (read_error) {
            attitude_show_calibration_layout();
            last_display_ms = now_ms;
            read_error = false;
        }

        calibration_status =
            imu_attitude_calibration_update(attitude, &sample);
        if (calibration_status == IMU_ATTITUDE_CALIBRATION_INVALID) {
            return ML_STATUS_INVALID_ARGUMENT;
        }
        if (calibration_status == IMU_ATTITUDE_CALIBRATION_COMPLETE) {
            return ML_STATUS_OK;
        }
        if ((calibration_status == IMU_ATTITUDE_CALIBRATION_RESTARTED) ||
            ((uint32_t) (now_ms - last_display_ms) >=
             ATTITUDE_DISPLAY_PERIOD_MS)) {
            attitude_show_calibration_progress(attitude,
                calibration_status ==
                    IMU_ATTITUDE_CALIBRATION_RESTARTED);
            last_display_ms = now_ms;
        }
    }
}

int main(void)
{
    icm42688_data_t sample;
    imu_attitude_t attitude;
    imu_attitude_angles_t angles;
    ml_status_t status;
    uint32_t last_update_ms;
    uint32_t last_display_ms;
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
        attitude_show_error("ICM42688 INIT   ", status);
        while (1) {
            delay_ms(1000U);
        }
    }
    status = imu_attitude_init(&attitude, &g_body_axis_config);
    if (status != ML_STATUS_OK) {
        attitude_show_error("ATTITUDE INIT   ", status);
        while (1) {
            delay_ms(1000U);
        }
    }
    status = tim_interrupt_ms_init_ex(TIMG8, 1U, 1U,
        attitude_timer_callback, 0);
    if (status != ML_STATUS_OK) {
        attitude_show_error("TIMER HW FAIL   ", status);
        while (1) {
            delay_ms(1000U);
        }
    }
    __enable_irq();
    status = attitude_verify_timer_tick();
    if (status != ML_STATUS_OK) {
        attitude_show_error("TIMER IRQ FAIL  ", status);
        while (1) {
            delay_ms(1000U);
        }
    }
    status = attitude_calibrate(&attitude);
    if (status != ML_STATUS_OK) {
        attitude_show_error("CALIBRATION ERR ", status);
        while (1) {
            delay_ms(1000U);
        }
    }

    attitude_show_layout();
    last_update_ms = g_attitude_milliseconds;
    last_display_ms = last_update_ms;
    while (1) {
        uint32_t now_ms = attitude_wait_for_sample(last_update_ms);
        uint32_t elapsed_ms = (uint32_t) (now_ms - last_update_ms);

        last_update_ms = now_ms;
        if (elapsed_ms > ATTITUDE_DT_MAX_MS) {
            continue;
        }
        status = icm42688_read(&sample);
        if (status != ML_STATUS_OK) {
            if (!read_error) {
                attitude_show_error("ICM42688 READ   ", status);
                read_error = true;
            }
            continue;
        }
        if (read_error) {
            attitude_show_layout();
            last_display_ms = now_ms;
            read_error = false;
            continue;
        }

        status = imu_attitude_update(&attitude, &sample,
            (float) elapsed_ms / 1000.0f, &angles);
        if (status != ML_STATUS_OK) {
            attitude_show_error("ATTITUDE UPDATE ", status);
            read_error = true;
            continue;
        }
        if ((uint32_t) (now_ms - last_display_ms) >=
            ATTITUDE_DISPLAY_PERIOD_MS) {
            attitude_show_angles(&angles);
            last_display_ms = now_ms;
        }
    }
}
