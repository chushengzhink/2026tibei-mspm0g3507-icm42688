#include "attitude_app.h"

#include "attitude_view.h"
#include "icm42688_service.h"
#include "ml_board.h"
#include "ml_delay.h"

#define ATTITUDE_DISPLAY_PERIOD_MS (100U)

typedef enum {
    ATTITUDE_APP_STATE_UNINITIALIZED = 0,
    ATTITUDE_APP_STATE_RUNNING,
    ATTITUDE_APP_STATE_DISPLAY_ERROR,
    ATTITUDE_APP_STATE_SERVICE_ERROR
} attitude_app_state_t;

typedef struct {
    icm42688_service_t service;
    icm42688_service_output_t output;
    uint32_t last_display_ms;
    attitude_app_state_t state;
} attitude_app_context_t;

/* Body frame is NWU: X forward, Y left, Z up. The installed sensor has
 * +Y forward and +X left, so its right-handed +Z axis points down. */
static const icm42688_service_config_t g_attitude_service_config = {
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

static attitude_app_context_t g_attitude_app;

ml_status_t attitude_app_init(void)
{
    ml_status_t status;

    g_attitude_app.output.angles.pitch_deg = 0.0f;
    g_attitude_app.output.angles.roll_deg = 0.0f;
    g_attitude_app.output.angles.yaw_deg = 0.0f;
    g_attitude_app.output.calibration_samples = 0U;
    g_attitude_app.output.timestamp_ms = 0U;
    g_attitude_app.last_display_ms = 0U;
    g_attitude_app.state = ATTITUDE_APP_STATE_UNINITIALIZED;

    status = attitude_view_init();
    if (status != ML_STATUS_OK) {
        (void) board_led_init();
        g_attitude_app.state = ATTITUDE_APP_STATE_DISPLAY_ERROR;
        return status;
    }

    status = icm42688_service_init(
        &g_attitude_app.service, &g_attitude_service_config);
    if (status != ML_STATUS_OK) {
        attitude_view_show_error(
            icm42688_service_get_state(&g_attitude_app.service), status);
        g_attitude_app.state = ATTITUDE_APP_STATE_SERVICE_ERROR;
        return status;
    }

    attitude_view_show_calibration();
    g_attitude_app.state = ATTITUDE_APP_STATE_RUNNING;
    return ML_STATUS_OK;
}

void attitude_app_poll(void)
{
    icm42688_service_event_t event;
    icm42688_service_state_t service_state;

    if (g_attitude_app.state == ATTITUDE_APP_STATE_DISPLAY_ERROR) {
        board_led_toggle();
        delay_ms(200U);
        return;
    }
    if (g_attitude_app.state != ATTITUDE_APP_STATE_RUNNING) {
        delay_ms(1000U);
        return;
    }

    event = icm42688_service_poll(
        &g_attitude_app.service, &g_attitude_app.output);
    service_state = icm42688_service_get_state(&g_attitude_app.service);

    switch (event) {
        case ICM42688_SERVICE_EVENT_NONE:
            delay_ms(1U);
            break;
        case ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS:
            if ((uint32_t) (g_attitude_app.output.timestamp_ms -
                g_attitude_app.last_display_ms) >=
                ATTITUDE_DISPLAY_PERIOD_MS) {
                attitude_view_show_calibration_progress(
                    g_attitude_app.output.calibration_samples, false);
                g_attitude_app.last_display_ms =
                    g_attitude_app.output.timestamp_ms;
            }
            break;
        case ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED:
            attitude_view_show_calibration_progress(
                g_attitude_app.output.calibration_samples, true);
            g_attitude_app.last_display_ms =
                g_attitude_app.output.timestamp_ms;
            break;
        case ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE:
            attitude_view_show_angles_layout();
            g_attitude_app.last_display_ms =
                g_attitude_app.output.timestamp_ms;
            break;
        case ICM42688_SERVICE_EVENT_ANGLES_UPDATED:
            if ((uint32_t) (g_attitude_app.output.timestamp_ms -
                g_attitude_app.last_display_ms) >=
                ATTITUDE_DISPLAY_PERIOD_MS) {
                attitude_view_show_angles(&g_attitude_app.output.angles);
                g_attitude_app.last_display_ms =
                    g_attitude_app.output.timestamp_ms;
            }
            break;
        case ICM42688_SERVICE_EVENT_READ_ERROR:
        case ICM42688_SERVICE_EVENT_UPDATE_ERROR:
            attitude_view_show_error(service_state,
                icm42688_service_get_last_status(
                    &g_attitude_app.service));
            break;
        case ICM42688_SERVICE_EVENT_READ_RECOVERED:
            if (service_state == ICM42688_SERVICE_STATE_READY) {
                attitude_view_show_angles_layout();
            } else {
                attitude_view_show_calibration();
            }
            g_attitude_app.last_display_ms =
                g_attitude_app.output.timestamp_ms;
            break;
        case ICM42688_SERVICE_EVENT_UPDATE_RECOVERED:
            attitude_view_show_angles_layout();
            attitude_view_show_angles(&g_attitude_app.output.angles);
            g_attitude_app.last_display_ms =
                g_attitude_app.output.timestamp_ms;
            break;
        case ICM42688_SERVICE_EVENT_TIMING_RESET:
        default:
            break;
    }
}
