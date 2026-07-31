#include "imu_attitude.h"

#include <math.h>

#define IMU_ATTITUDE_SAMPLE_RATE_HZ       (100.0f)
#define IMU_ATTITUDE_GYRO_RANGE_DPS       (1000.0f)
#define IMU_ATTITUDE_GAIN                 (0.5f)
#define IMU_ATTITUDE_ACCEL_REJECTION_DEG  (10.0f)
#define IMU_ATTITUDE_REJECTION_TIMEOUT_S  (5.0f)
#define IMU_ATTITUDE_STILL_GYRO_DPS       (3.0f)
#define IMU_ATTITUDE_STILL_ACCEL_MIN_G    (0.85f)
#define IMU_ATTITUDE_STILL_ACCEL_MAX_G    (1.15f)
#define IMU_ATTITUDE_GYRO_STDDEV_MAX_DPS  (0.25f)
#define IMU_ATTITUDE_ACCEL_USE_MIN_G      (0.80f)
#define IMU_ATTITUDE_ACCEL_USE_MAX_G      (1.20f)
#define IMU_ATTITUDE_DT_MIN_S             (0.005f)
#define IMU_ATTITUDE_DT_MAX_S             (0.050f)
#define IMU_ATTITUDE_FLOAT_LIMIT          (1000000.0f)

const imu_attitude_config_t imu_attitude_default_config = {
    {
        IMU_ATTITUDE_AXIS_X,
        IMU_ATTITUDE_AXIS_Y,
        IMU_ATTITUDE_AXIS_Z
    },
    {1, 1, 1},
    0U
};

static bool imu_attitude_float_valid(float value)
{
    return (value == value) &&
        (value > -IMU_ATTITUDE_FLOAT_LIMIT) &&
        (value < IMU_ATTITUDE_FLOAT_LIMIT);
}

static bool imu_attitude_config_valid(const imu_attitude_config_t *config)
{
    uint8_t used = 0U;
    uint8_t axis;

    if (config == 0) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        uint8_t source = config->source_axis[axis];

        if ((source > IMU_ATTITUDE_AXIS_Z) ||
            ((config->axis_sign[axis] != 1) &&
             (config->axis_sign[axis] != -1)) ||
            ((used & (uint8_t) (1U << source)) != 0U)) {
            return false;
        }
        used |= (uint8_t) (1U << source);
    }
    return used == 0x07U;
}

static bool imu_attitude_sample_valid(const icm42688_data_t *sample)
{
    if (sample == 0) {
        return false;
    }
    return imu_attitude_float_valid(sample->accel_x_g) &&
        imu_attitude_float_valid(sample->accel_y_g) &&
        imu_attitude_float_valid(sample->accel_z_g) &&
        imu_attitude_float_valid(sample->gyro_x_dps) &&
        imu_attitude_float_valid(sample->gyro_y_dps) &&
        imu_attitude_float_valid(sample->gyro_z_dps);
}

static void imu_attitude_map_sample(const imu_attitude_t *context,
    const icm42688_data_t *sample, float accel_g[3], float gyro_dps[3])
{
    const float source_accel[3] = {
        sample->accel_x_g, sample->accel_y_g, sample->accel_z_g
    };
    const float source_gyro[3] = {
        sample->gyro_x_dps, sample->gyro_y_dps, sample->gyro_z_dps
    };
    uint8_t axis;

    for (axis = 0U; axis < 3U; ++axis) {
        uint8_t source = context->config.source_axis[axis];
        float sign = (float) context->config.axis_sign[axis];

        accel_g[axis] = source_accel[source] * sign;
        gyro_dps[axis] = source_gyro[source] * sign;
    }
}

static void imu_attitude_reset_calibration(imu_attitude_t *context)
{
    uint8_t axis;

    context->calibration_samples = 0U;
    context->calibrated = false;
    for (axis = 0U; axis < 3U; ++axis) {
        context->gyro_bias_dps[axis] = 0.0f;
        context->last_gyro_dps[axis] = 0.0f;
        context->gyro_mean_dps[axis] = 0.0f;
        context->gyro_m2[axis] = 0.0f;
        context->accel_sum_g[axis] = 0.0f;
    }
    FusionAhrsRestart(&context->ahrs);
}

static float imu_attitude_vector_norm_squared(const float vector[3])
{
    return vector[0] * vector[0] + vector[1] * vector[1] +
        vector[2] * vector[2];
}

static bool imu_attitude_stationary(
    const float accel_g[3], const float gyro_dps[3])
{
    const float gyro_limit_squared = IMU_ATTITUDE_STILL_GYRO_DPS *
        IMU_ATTITUDE_STILL_GYRO_DPS;
    const float accel_min_squared = IMU_ATTITUDE_STILL_ACCEL_MIN_G *
        IMU_ATTITUDE_STILL_ACCEL_MIN_G;
    const float accel_max_squared = IMU_ATTITUDE_STILL_ACCEL_MAX_G *
        IMU_ATTITUDE_STILL_ACCEL_MAX_G;
    float accel_squared = imu_attitude_vector_norm_squared(accel_g);

    return (imu_attitude_vector_norm_squared(gyro_dps) <
            gyro_limit_squared) &&
        (accel_squared >= accel_min_squared) &&
        (accel_squared <= accel_max_squared);
}

static void imu_attitude_accumulate_calibration(imu_attitude_t *context,
    const float accel_g[3], const float gyro_dps[3])
{
    float count = (float) (context->calibration_samples + 1U);
    FusionVector accelerometer;
    uint8_t axis;

    for (axis = 0U; axis < 3U; ++axis) {
        float delta = gyro_dps[axis] - context->gyro_mean_dps[axis];

        context->gyro_mean_dps[axis] += delta / count;
        context->gyro_m2[axis] += delta *
            (gyro_dps[axis] - context->gyro_mean_dps[axis]);
        context->accel_sum_g[axis] += accel_g[axis];
    }
    ++context->calibration_samples;

    accelerometer.axis.x = accel_g[0];
    accelerometer.axis.y = accel_g[1];
    accelerometer.axis.z = accel_g[2];
    FusionAhrsSetSamplePeriod(
        &context->ahrs, 1.0f / IMU_ATTITUDE_SAMPLE_RATE_HZ);
    FusionAhrsUpdateNoMagnetometer(
        &context->ahrs, FUSION_VECTOR_ZERO, accelerometer);
}

static bool imu_attitude_gyro_variance_valid(
    const imu_attitude_t *context)
{
    const float maximum_variance = IMU_ATTITUDE_GYRO_STDDEV_MAX_DPS *
        IMU_ATTITUDE_GYRO_STDDEV_MAX_DPS;
    float divisor = (float) (context->calibration_samples - 1U);
    uint8_t axis;

    for (axis = 0U; axis < 3U; ++axis) {
        if ((context->gyro_m2[axis] / divisor) > maximum_variance) {
            return false;
        }
    }
    return true;
}

static bool imu_attitude_finish_calibration(imu_attitude_t *context)
{
    float accel[3];
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    float cr;
    float sr;
    float cp;
    float sp;
    float horizontal;
    FusionQuaternion quaternion;
    uint8_t axis;

    if (!imu_attitude_gyro_variance_valid(context)) {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        context->gyro_bias_dps[axis] = context->gyro_mean_dps[axis];
        accel[axis] = context->accel_sum_g[axis] /
            (float) context->calibration_samples;
    }

    horizontal = sqrtf(accel[1] * accel[1] + accel[2] * accel[2]);
    roll = atan2f(accel[1], accel[2]);
    pitch = atan2f(-accel[0], horizontal);
    half_roll = roll * 0.5f;
    half_pitch = pitch * 0.5f;
    cr = cosf(half_roll);
    sr = sinf(half_roll);
    cp = cosf(half_pitch);
    sp = sinf(half_pitch);

    quaternion.element.w = cr * cp;
    quaternion.element.x = sr * cp;
    quaternion.element.y = cr * sp;
    quaternion.element.z = -sr * sp;
    FusionAhrsSetQuaternion(&context->ahrs, quaternion);
    context->calibrated = true;
    return true;
}

ml_status_t imu_attitude_init(
    imu_attitude_t *context, const imu_attitude_config_t *config)
{
    FusionAhrsSettings settings;

    if (context == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    context->initialized = false;
    context->calibrated = false;
    if (config == 0) {
        config = &imu_attitude_default_config;
    }
    if (!imu_attitude_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    context->config = *config;
    context->calibration_samples_required =
        (config->calibration_samples_required == 0U) ?
        IMU_ATTITUDE_CALIBRATION_SAMPLES :
        config->calibration_samples_required;
    settings.sampleRate = IMU_ATTITUDE_SAMPLE_RATE_HZ;
    settings.convention = FusionConventionNwu;
    settings.gain = IMU_ATTITUDE_GAIN;
    settings.gyroscopeRange = IMU_ATTITUDE_GYRO_RANGE_DPS;
    settings.accelerationRejection = IMU_ATTITUDE_ACCEL_REJECTION_DEG;
    settings.magneticRejection = 0.0f;
    settings.rejectionTimeout = IMU_ATTITUDE_REJECTION_TIMEOUT_S;
    FusionAhrsInitialise(&context->ahrs);
    FusionAhrsSetSettings(&context->ahrs, &settings);
    imu_attitude_reset_calibration(context);
    context->initialized = true;
    return ML_STATUS_OK;
}

imu_attitude_calibration_status_t imu_attitude_calibration_update(
    imu_attitude_t *context, const icm42688_data_t *sample)
{
    float accel_g[3];
    float gyro_dps[3];

    if ((context == 0) || !context->initialized ||
        !imu_attitude_sample_valid(sample)) {
        return IMU_ATTITUDE_CALIBRATION_INVALID;
    }
    if (context->calibrated) {
        return IMU_ATTITUDE_CALIBRATION_COMPLETE;
    }

    imu_attitude_map_sample(context, sample, accel_g, gyro_dps);
    if (!imu_attitude_stationary(accel_g, gyro_dps)) {
        imu_attitude_reset_calibration(context);
        return IMU_ATTITUDE_CALIBRATION_RESTARTED;
    }

    imu_attitude_accumulate_calibration(context, accel_g, gyro_dps);
    if (context->calibration_samples <
        context->calibration_samples_required) {
        return IMU_ATTITUDE_CALIBRATION_IN_PROGRESS;
    }
    if (!imu_attitude_finish_calibration(context)) {
        imu_attitude_reset_calibration(context);
        return IMU_ATTITUDE_CALIBRATION_RESTARTED;
    }
    return IMU_ATTITUDE_CALIBRATION_COMPLETE;
}

ml_status_t imu_attitude_update(imu_attitude_t *context,
    const icm42688_data_t *sample, float dt_s,
    imu_attitude_angles_t *angles)
{
    float accel_g[3];
    float gyro_dps[3];
    float accel_squared;
    const float accel_min_squared = IMU_ATTITUDE_ACCEL_USE_MIN_G *
        IMU_ATTITUDE_ACCEL_USE_MIN_G;
    const float accel_max_squared = IMU_ATTITUDE_ACCEL_USE_MAX_G *
        IMU_ATTITUDE_ACCEL_USE_MAX_G;
    FusionVector accelerometer;
    FusionVector gyroscope;
    FusionEuler euler;

    if ((context == 0) || (angles == 0) ||
        !imu_attitude_sample_valid(sample) ||
        !imu_attitude_float_valid(dt_s) ||
        (dt_s < IMU_ATTITUDE_DT_MIN_S) ||
        (dt_s > IMU_ATTITUDE_DT_MAX_S)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!context->initialized || !context->calibrated) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    imu_attitude_map_sample(context, sample, accel_g, gyro_dps);
    gyro_dps[0] -= context->gyro_bias_dps[0];
    gyro_dps[1] -= context->gyro_bias_dps[1];
    gyro_dps[2] -= context->gyro_bias_dps[2];
    context->last_gyro_dps[0] = gyro_dps[0];
    context->last_gyro_dps[1] = gyro_dps[1];
    context->last_gyro_dps[2] = gyro_dps[2];

    gyroscope.axis.x = gyro_dps[0];
    gyroscope.axis.y = gyro_dps[1];
    gyroscope.axis.z = gyro_dps[2];
    accel_squared = imu_attitude_vector_norm_squared(accel_g);
    if ((accel_squared >= accel_min_squared) &&
        (accel_squared <= accel_max_squared)) {
        accelerometer.axis.x = accel_g[0];
        accelerometer.axis.y = accel_g[1];
        accelerometer.axis.z = accel_g[2];
    } else {
        accelerometer = FUSION_VECTOR_ZERO;
    }

    FusionAhrsSetSamplePeriod(&context->ahrs, dt_s);
    FusionAhrsUpdateNoMagnetometer(
        &context->ahrs, gyroscope, accelerometer);
    euler = FusionQuaternionToEuler(
        FusionAhrsGetQuaternion(&context->ahrs));
    if (!imu_attitude_float_valid(euler.angle.pitch) ||
        !imu_attitude_float_valid(euler.angle.roll) ||
        !imu_attitude_float_valid(euler.angle.yaw)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    angles->pitch_deg = euler.angle.pitch;
    angles->roll_deg = euler.angle.roll;
    angles->yaw_deg = euler.angle.yaw;
    return ML_STATUS_OK;
}

uint16_t imu_attitude_calibration_progress(
    const imu_attitude_t *context)
{
    if ((context == 0) || !context->initialized) {
        return 0U;
    }
    return context->calibration_samples;
}

ml_status_t imu_attitude_get_gyro_bias(
    const imu_attitude_t *context, float bias_dps[3])
{
    uint8_t axis;

    if ((context == 0) || (bias_dps == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!context->initialized || !context->calibrated) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        bias_dps[axis] = context->gyro_bias_dps[axis];
    }
    return ML_STATUS_OK;
}

ml_status_t imu_attitude_get_body_gyro_dps(
    const imu_attitude_t *context, float gyro_dps[3])
{
    uint8_t axis;

    if ((context == 0) || (gyro_dps == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!context->initialized || !context->calibrated) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        gyro_dps[axis] = context->last_gyro_dps[axis];
    }
    return ML_STATUS_OK;
}
