#include "icm42688_service.h"

#include "ml_delay.h"

#define ICM42688_SERVICE_TIMER_PERIOD_MS  (1U)
#define ICM42688_SERVICE_SAMPLE_PERIOD_MS (10U)
#define ICM42688_SERVICE_DT_MAX_MS         (50U)
#define ICM42688_SERVICE_TIMER_TEST_MS     (20U)

static void icm42688_service_timer_callback(void *callback_context)
{
    icm42688_service_t *context =
        (icm42688_service_t *) callback_context;

    if (context != 0) {
        ++context->milliseconds;
    }
}

static void icm42688_service_set_error(icm42688_service_t *context,
    icm42688_service_state_t state, ml_status_t status)
{
    context->state = state;
    context->last_status = status;
}

static void icm42688_service_fill_output(
    const icm42688_service_t *context, icm42688_service_output_t *output,
    uint32_t timestamp_ms)
{
    output->angles = context->angles;
    output->calibration_samples =
        imu_attitude_calibration_progress(&context->attitude);
    output->timestamp_ms = timestamp_ms;
}

ml_status_t icm42688_service_init(icm42688_service_t *context,
    const icm42688_service_config_t *config)
{
    uint32_t before_ms;
    ml_status_t status;

    if ((context == 0) || (config == 0) || (config->timer == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    context->config = *config;
    context->angles.pitch_deg = 0.0f;
    context->angles.roll_deg = 0.0f;
    context->angles.yaw_deg = 0.0f;
    context->milliseconds = 0U;
    context->last_sample_ms = 0U;
    context->last_status = ML_STATUS_NOT_INITIALIZED;
    context->state = ICM42688_SERVICE_STATE_UNINITIALIZED;
    context->initialized = false;
    context->calibrated = false;
    context->read_error = false;
    context->update_error = false;

    status = icm42688_init();
    if (status != ML_STATUS_OK) {
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_SENSOR_INIT_ERROR, status);
        return status;
    }

    status = imu_attitude_init(&context->attitude, &config->axis_config);
    if (status != ML_STATUS_OK) {
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_ATTITUDE_INIT_ERROR, status);
        return status;
    }

    status = tim_interrupt_ms_init_ex(config->timer,
        ICM42688_SERVICE_TIMER_PERIOD_MS, config->timer_priority,
        icm42688_service_timer_callback, context);
    if (status != ML_STATUS_OK) {
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_TIMER_HARDWARE_ERROR, status);
        return status;
    }

    before_ms = context->milliseconds;
    delay_ms(ICM42688_SERVICE_TIMER_TEST_MS);
    if (context->milliseconds == before_ms) {
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_TIMER_INTERRUPT_ERROR,
            ML_STATUS_TIMEOUT);
        return ML_STATUS_TIMEOUT;
    }

    context->last_sample_ms = context->milliseconds;
    context->last_status = ML_STATUS_OK;
    context->state = ICM42688_SERVICE_STATE_CALIBRATING;
    context->initialized = true;
    return ML_STATUS_OK;
}

icm42688_service_event_t icm42688_service_poll(
    icm42688_service_t *context, icm42688_service_output_t *output)
{
    icm42688_data_t sample;
    imu_attitude_calibration_status_t calibration_status;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    ml_status_t status;
    bool first_error;

    if ((context == 0) || (output == 0)) {
        return ICM42688_SERVICE_EVENT_UPDATE_ERROR;
    }
    if (!context->initialized) {
        context->last_status = ML_STATUS_NOT_INITIALIZED;
        return ICM42688_SERVICE_EVENT_UPDATE_ERROR;
    }

    now_ms = context->milliseconds;
    icm42688_service_fill_output(context, output, now_ms);
    elapsed_ms = (uint32_t) (now_ms - context->last_sample_ms);
    if (elapsed_ms < ICM42688_SERVICE_SAMPLE_PERIOD_MS) {
        return ICM42688_SERVICE_EVENT_NONE;
    }
    context->last_sample_ms = now_ms;
    if (elapsed_ms > ICM42688_SERVICE_DT_MAX_MS) {
        context->last_status = ML_STATUS_OK;
        return ICM42688_SERVICE_EVENT_TIMING_RESET;
    }

    status = icm42688_read(&sample);
    if (status != ML_STATUS_OK) {
        first_error = !context->read_error;
        context->read_error = true;
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_SENSOR_READ_ERROR, status);
        return first_error ? ICM42688_SERVICE_EVENT_READ_ERROR :
            ICM42688_SERVICE_EVENT_NONE;
    }
    if (context->read_error) {
        context->read_error = false;
        context->last_status = ML_STATUS_OK;
        context->state = context->calibrated ?
            ICM42688_SERVICE_STATE_READY :
            ICM42688_SERVICE_STATE_CALIBRATING;
        return ICM42688_SERVICE_EVENT_READ_RECOVERED;
    }

    if (!context->calibrated) {
        calibration_status = imu_attitude_calibration_update(
            &context->attitude, &sample);
        icm42688_service_fill_output(context, output, now_ms);
        if (calibration_status == IMU_ATTITUDE_CALIBRATION_INVALID) {
            context->update_error = true;
            icm42688_service_set_error(context,
                ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR,
                ML_STATUS_INVALID_ARGUMENT);
            return ICM42688_SERVICE_EVENT_UPDATE_ERROR;
        }
        context->last_status = ML_STATUS_OK;
        context->state = ICM42688_SERVICE_STATE_CALIBRATING;
        if (calibration_status == IMU_ATTITUDE_CALIBRATION_RESTARTED) {
            return ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED;
        }
        if (calibration_status == IMU_ATTITUDE_CALIBRATION_COMPLETE) {
            context->calibrated = true;
            context->state = ICM42688_SERVICE_STATE_READY;
            return ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE;
        }
        return ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS;
    }

    status = imu_attitude_update(&context->attitude, &sample,
        (float) elapsed_ms / 1000.0f, &context->angles);
    if (status != ML_STATUS_OK) {
        first_error = !context->update_error;
        context->update_error = true;
        icm42688_service_set_error(context,
            ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR, status);
        return first_error ? ICM42688_SERVICE_EVENT_UPDATE_ERROR :
            ICM42688_SERVICE_EVENT_NONE;
    }

    icm42688_service_fill_output(context, output, now_ms);
    context->last_status = ML_STATUS_OK;
    context->state = ICM42688_SERVICE_STATE_READY;
    if (context->update_error) {
        context->update_error = false;
        return ICM42688_SERVICE_EVENT_UPDATE_RECOVERED;
    }
    return ICM42688_SERVICE_EVENT_ANGLES_UPDATED;
}

icm42688_service_state_t icm42688_service_get_state(
    const icm42688_service_t *context)
{
    if (context == 0) {
        return ICM42688_SERVICE_STATE_UNINITIALIZED;
    }
    return context->state;
}

ml_status_t icm42688_service_get_last_status(
    const icm42688_service_t *context)
{
    if (context == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    return context->last_status;
}
