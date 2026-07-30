#include "icm42688_service.h"

#include <math.h>
#include <stdio.h>

static int g_failures;
static tim_callback_t g_timer_callback;
static void *g_timer_context;
static ml_status_t g_read_status = ML_STATUS_OK;
static icm42688_data_t g_sample = {
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f
};

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static void advance_ms(uint32_t milliseconds)
{
    uint32_t index;

    for (index = 0U; index < milliseconds; ++index) {
        if (g_timer_callback != 0) {
            g_timer_callback(g_timer_context);
        }
    }
}

ml_status_t tim_interrupt_ms_init_ex(
    GPTIMER_Regs *timer, uint32_t time_ms, uint8_t priority,
    tim_callback_t callback, void *context)
{
    (void) timer;
    (void) time_ms;
    (void) priority;
    g_timer_callback = callback;
    g_timer_context = context;
    return ML_STATUS_OK;
}

ml_status_t tim_interrupt_ms_init(
    GPTIMER_Regs *timer, uint32_t time_ms, uint8_t priority)
{
    return tim_interrupt_ms_init_ex(timer, time_ms, priority, 0, 0);
}

void tim_irq_dispatch(GPTIMER_Regs *timer)
{
    (void) timer;
}

void delay_ms(uint32_t delay)
{
    advance_ms(delay);
}

void delay_us(uint32_t delay)
{
    (void) delay;
}

void SysTick_Init(void)
{
}

void SysTick_Wait(uint32_t delay)
{
    (void) delay;
}

ml_status_t icm42688_init(void)
{
    return ML_STATUS_OK;
}

ml_status_t icm42688_read(icm42688_data_t *data)
{
    if (g_read_status != ML_STATUS_OK) {
        return g_read_status;
    }
    *data = g_sample;
    return ML_STATUS_OK;
}

int main(void)
{
    icm42688_service_t service;
    icm42688_service_output_t output;
    const icm42688_service_config_t config = {
        {
            {IMU_ATTITUDE_AXIS_X, IMU_ATTITUDE_AXIS_Y,
             IMU_ATTITUDE_AXIS_Z},
            {1, 1, 1}
        },
        (GPTIMER_Regs *) 1,
        1U
    };
    icm42688_service_event_t event;
    uint16_t index;

    check(icm42688_service_init(&service, &config) == ML_STATUS_OK,
        "service initializes with a running timer");

    advance_ms(10U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS &&
        output.calibration_samples == 1U &&
        fabsf(output.accel_norm_g - 1.0f) < 0.001f &&
        fabsf(output.gyro_norm_dps) < 0.001f,
        "calibration progress includes vector norms");

    g_sample.gyro_x_dps = 4.0f;
    advance_ms(10U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED &&
        output.calibration_samples == 0U &&
        output.calibration_restart_count == 1U,
        "movement restart is counted and reported");

    g_sample.gyro_x_dps = 0.0f;
    g_read_status = ML_STATUS_TIMEOUT;
    advance_ms(10U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_READ_ERROR &&
        output.read_error_count == 1U &&
        icm42688_service_get_state(&service) ==
            ICM42688_SERVICE_STATE_SENSOR_READ_ERROR,
        "sensor read errors are counted and exposed");

    g_read_status = ML_STATUS_OK;
    advance_ms(10U);
    check(icm42688_service_poll(&service, &output) ==
        ICM42688_SERVICE_EVENT_READ_RECOVERED,
        "sensor read recovery is reported");

    advance_ms(60U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_TIMING_RESET &&
        output.timing_reset_count == 1U,
        "slow polling is counted and reported");

    g_sample.accel_x_g = NAN;
    advance_ms(10U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_UPDATE_ERROR &&
        output.update_error_count == 1U &&
        icm42688_service_get_state(&service) ==
            ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR,
        "invalid IMU data is counted and reported");

    g_sample.accel_x_g = 0.0f;
    g_sample.accel_y_g = 0.0f;
    g_sample.accel_z_g = 1.0f;
    g_sample.gyro_x_dps = 0.0f;
    g_sample.gyro_y_dps = 0.0f;
    g_sample.gyro_z_dps = 0.0f;
    check(icm42688_service_init(&service, &config) == ML_STATUS_OK,
        "service reinitializes for mapped gyro output test");
    for (index = 0U; index < IMU_ATTITUDE_CALIBRATION_SAMPLES; ++index) {
        advance_ms(10U);
        event = icm42688_service_poll(&service, &output);
    }
    check(event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE,
        "service completes stationary calibration");
    g_sample.gyro_z_dps = 30.0f;
    advance_ms(10U);
    event = icm42688_service_poll(&service, &output);
    check(event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED &&
        fabsf(output.body_gyro_z_dps - 30.0f) < 0.01f,
        "service exposes mapped bias-corrected body Z gyro rate");

    if (g_failures == 0) {
        printf("PASS: ICM42688 service diagnostics\n");
    }
    return g_failures == 0 ? 0 : 1;
}
