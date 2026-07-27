#include "headfile.h"
#include "icm42688_service.h"

#define ATTITUDE_DISPLAY_PERIOD_MS (100U)

/* Body frame is NWU: X forward, Y left, Z up. The installed sensor has
 * +Y forward and +X left, so its right-handed +Z axis points down. */
static const icm42688_service_config_t g_icm42688_service_config = {
    {
        {
            IMU_ATTITUDE_AXIS_Y,
            IMU_ATTITUDE_AXIS_X,
            IMU_ATTITUDE_AXIS_Z
        },
        {1, 1, -1}
    },
    TIMG8,
    1U
};

static const char *attitude_error_title(icm42688_service_state_t state)
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

static void attitude_show_error(
    icm42688_service_state_t state, ml_status_t status)
{
    (void) OLED_Clear();
    (void) OLED_ShowString(1U, 1U, attitude_error_title(state));
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
    uint16_t calibration_samples, bool restarted)
{
    (void) OLED_ShowNum(
        2U, 5U, (uint32_t) calibration_samples, 3U);
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

int main(void)
{
    icm42688_service_t service;
    icm42688_service_output_t output = {
        {0.0f, 0.0f, 0.0f}, 0U, 0U
    };
    icm42688_service_event_t event;
    icm42688_service_state_t state;
    ml_status_t status;
    uint32_t last_display_ms = 0U;

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

    __enable_irq();
    status = icm42688_service_init(
        &service, &g_icm42688_service_config);
    if (status != ML_STATUS_OK) {
        attitude_show_error(
            icm42688_service_get_state(&service), status);
        while (1) {
            delay_ms(1000U);
        }
    }

    attitude_show_calibration_layout();
    while (1) {
        event = icm42688_service_poll(&service, &output);
        state = icm42688_service_get_state(&service);

        switch (event) {
            case ICM42688_SERVICE_EVENT_NONE:
                delay_ms(1U);
                break;
            case ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS:
                if ((uint32_t) (output.timestamp_ms - last_display_ms) >=
                    ATTITUDE_DISPLAY_PERIOD_MS) {
                    attitude_show_calibration_progress(
                        output.calibration_samples, false);
                    last_display_ms = output.timestamp_ms;
                }
                break;
            case ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED:
                attitude_show_calibration_progress(
                    output.calibration_samples, true);
                last_display_ms = output.timestamp_ms;
                break;
            case ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE:
                attitude_show_layout();
                last_display_ms = output.timestamp_ms;
                break;
            case ICM42688_SERVICE_EVENT_ANGLES_UPDATED:
                if ((uint32_t) (output.timestamp_ms - last_display_ms) >=
                    ATTITUDE_DISPLAY_PERIOD_MS) {
                    attitude_show_angles(&output.angles);
                    last_display_ms = output.timestamp_ms;
                }
                break;
            case ICM42688_SERVICE_EVENT_READ_ERROR:
            case ICM42688_SERVICE_EVENT_UPDATE_ERROR:
                attitude_show_error(state,
                    icm42688_service_get_last_status(&service));
                break;
            case ICM42688_SERVICE_EVENT_READ_RECOVERED:
                if (state == ICM42688_SERVICE_STATE_READY) {
                    attitude_show_layout();
                } else {
                    attitude_show_calibration_layout();
                }
                last_display_ms = output.timestamp_ms;
                break;
            case ICM42688_SERVICE_EVENT_UPDATE_RECOVERED:
                attitude_show_layout();
                attitude_show_angles(&output.angles);
                last_display_ms = output.timestamp_ms;
                break;
            case ICM42688_SERVICE_EVENT_TIMING_RESET:
            default:
                break;
        }
    }
}
