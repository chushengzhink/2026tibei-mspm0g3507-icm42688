#include "attitude_view.h"

#include "ml_oled.h"

static const char *attitude_view_error_title(
    icm42688_service_state_t state)
{
    switch (state) {
        case ICM42688_SERVICE_STATE_SENSOR_INIT_ERROR:
            return "ICM42688 INIT   ";
        case ICM42688_SERVICE_STATE_ATTITUDE_INIT_ERROR:
            return "ATTITUDE INIT   ";
        case ICM42688_SERVICE_STATE_TIMER_HARDWARE_ERROR:
            return "TIMER HW FAIL   ";
        case ICM42688_SERVICE_STATE_TIMER_INTERRUPT_ERROR:
            return "TIMER IRQ FAIL  ";
        case ICM42688_SERVICE_STATE_SENSOR_READ_ERROR:
            return "ICM42688 READ   ";
        case ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR:
            return "ATTITUDE UPDATE ";
        default:
            return "ATTITUDE ERROR  ";
    }
}

ml_status_t attitude_view_init(void)
{
    return OLED_Init();
}

void attitude_view_show_error(
    icm42688_service_state_t state, ml_status_t status)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, attitude_view_error_title(state));
    (void) OLED_ShowString(2U, 1U, "STAT:00         ");
    (void) OLED_ShowNum(2U, 6U, (uint32_t) status, 2U);
    (void) OLED_ShowString(3U, 1U, "SDA:PA0 SCL:PA1 ");
    (void) OLED_ShowString(4U, 1U, "CHECK AND RETRY ");
}

void attitude_view_show_calibration(void)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, "KEEP STILL      ");
    (void) OLED_ShowString(2U, 1U, "CAL:000/300     ");
    (void) OLED_ShowString(3U, 1U, "GYRO ZERO BIAS  ");
    (void) OLED_ShowString(4U, 1U, "WAIT ABOUT 3 SEC");
}

void attitude_view_show_calibration_progress(
    uint16_t calibration_samples, bool restarted)
{
    (void) OLED_ShowNum(
        2U, 5U, (uint32_t) calibration_samples, 3U);
    (void) OLED_ShowString(4U, 1U,
        restarted ? "MOVED - RETRY   " : "WAIT ABOUT 3 SEC");
}

void attitude_view_show_angles_layout(void)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, "ATTITUDE REL DEG");
    (void) OLED_ShowString(2U, 1U, "P:+000.0 DEG    ");
    (void) OLED_ShowString(3U, 1U, "R:+000.0 DEG    ");
    (void) OLED_ShowString(4U, 1U, "Y:+000.0 DEG    ");
}

void attitude_view_show_angles(const imu_attitude_angles_t *angles)
{
    if (angles == 0) {
        return;
    }
    (void) OLED_ShowFloat(2U, 3U, angles->pitch_deg, 3U, 1U);
    (void) OLED_ShowFloat(3U, 3U, angles->roll_deg, 3U, 1U);
    (void) OLED_ShowFloat(4U, 3U, angles->yaw_deg, 3U, 1U);
}
