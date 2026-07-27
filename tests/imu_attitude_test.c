#include "imu_attitude.h"

#include <math.h>
#include <stdio.h>

#define TEST_TOLERANCE_BIAS_DPS (0.001f)
#define TEST_TOLERANCE_ANGLE_DEG (1.0f)

static int g_failures;

static void test_check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static bool test_near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static imu_attitude_calibration_status_t test_calibrate(
    imu_attitude_t *attitude, const icm42688_data_t *sample)
{
    imu_attitude_calibration_status_t status =
        IMU_ATTITUDE_CALIBRATION_INVALID;
    uint16_t index;

    for (index = 0U; index < IMU_ATTITUDE_CALIBRATION_SAMPLES; ++index) {
        status = imu_attitude_calibration_update(attitude, sample);
    }
    return status;
}

static void test_bias_and_yaw(void)
{
    const icm42688_data_t still = {
        0.0f, 0.0f, 1.0f,
        0.4f, -0.2f, 0.1f
    };
    icm42688_data_t rotating = still;
    imu_attitude_t attitude;
    imu_attitude_angles_t angles = {0.0f, 0.0f, 0.0f};
    float bias[3];
    uint16_t index;

    test_check(imu_attitude_init(&attitude, 0) == ML_STATUS_OK,
        "default attitude configuration is accepted");
    test_check(test_calibrate(&attitude, &still) ==
        IMU_ATTITUDE_CALIBRATION_COMPLETE,
        "stationary calibration completes after 300 samples");
    test_check(imu_attitude_get_gyro_bias(&attitude, bias) == ML_STATUS_OK,
        "calibrated gyro bias is available");
    test_check(test_near(bias[0], 0.4f, TEST_TOLERANCE_BIAS_DPS) &&
        test_near(bias[1], -0.2f, TEST_TOLERANCE_BIAS_DPS) &&
        test_near(bias[2], 0.1f, TEST_TOLERANCE_BIAS_DPS),
        "known gyro bias is recovered");

    test_check(imu_attitude_update(
        &attitude, &still, 0.01f, &angles) == ML_STATUS_OK,
        "stationary attitude update succeeds");
    test_check(test_near(angles.pitch_deg, 0.0f,
        TEST_TOLERANCE_ANGLE_DEG) &&
        test_near(angles.roll_deg, 0.0f, TEST_TOLERANCE_ANGLE_DEG) &&
        test_near(angles.yaw_deg, 0.0f, TEST_TOLERANCE_ANGLE_DEG),
        "level calibration starts at zero attitude");

    rotating.gyro_z_dps += 90.0f;
    for (index = 0U; index < 100U; ++index) {
        test_check(imu_attitude_update(
            &attitude, &rotating, 0.01f, &angles) == ML_STATUS_OK,
            "yaw integration update succeeds");
    }
    test_check(test_near(
        angles.yaw_deg, 90.0f, TEST_TOLERANCE_ANGLE_DEG),
        "90 dps for one second produces 90 degree relative yaw");
}

static void test_tilt_initialisation(void)
{
    const icm42688_data_t tilted = {
        0.0f, 0.5f, 0.8660254f,
        0.0f, 0.0f, 0.0f
    };
    imu_attitude_t attitude;
    imu_attitude_angles_t angles;

    test_check(imu_attitude_init(&attitude, 0) == ML_STATUS_OK,
        "tilt test initialises");
    test_check(test_calibrate(&attitude, &tilted) ==
        IMU_ATTITUDE_CALIBRATION_COMPLETE,
        "tilted stationary calibration completes");
    test_check(imu_attitude_update(
        &attitude, &tilted, 0.01f, &angles) == ML_STATUS_OK,
        "tilted attitude update succeeds");
    test_check(test_near(
        angles.roll_deg, 30.0f, TEST_TOLERANCE_ANGLE_DEG) &&
        test_near(angles.pitch_deg, 0.0f, TEST_TOLERANCE_ANGLE_DEG),
        "mean gravity initialises roll and pitch");
}

static void test_movement_and_invalid_inputs(void)
{
    icm42688_data_t sample = {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f
    };
    const imu_attitude_config_t invalid_mapping = {
        {IMU_ATTITUDE_AXIS_X, IMU_ATTITUDE_AXIS_X, IMU_ATTITUDE_AXIS_Z},
        {1, 1, 1}
    };
    imu_attitude_t attitude;
    imu_attitude_angles_t angles;
    uint16_t index;

    test_check(imu_attitude_init(&attitude, &invalid_mapping) ==
        ML_STATUS_INVALID_ARGUMENT,
        "duplicate source axes are rejected");
    test_check(imu_attitude_calibration_update(&attitude, &sample) ==
        IMU_ATTITUDE_CALIBRATION_INVALID,
        "failed reinitialisation leaves the context inactive");
    test_check(imu_attitude_init(&attitude, 0) == ML_STATUS_OK,
        "movement test initialises");
    for (index = 0U; index < 50U; ++index) {
        (void) imu_attitude_calibration_update(&attitude, &sample);
    }
    sample.gyro_x_dps = 4.0f;
    test_check(imu_attitude_calibration_update(&attitude, &sample) ==
        IMU_ATTITUDE_CALIBRATION_RESTARTED,
        "movement restarts calibration");
    test_check(imu_attitude_calibration_progress(&attitude) == 0U,
        "movement clears calibration progress");

    sample.gyro_x_dps = 0.0f;
    test_check(test_calibrate(&attitude, &sample) ==
        IMU_ATTITUDE_CALIBRATION_COMPLETE,
        "calibration completes after movement stops");
    sample.accel_z_g = 2.0f;
    test_check(imu_attitude_update(
        &attitude, &sample, 0.01f, &angles) == ML_STATUS_OK,
        "acceleration shock falls back to gyro-only update");
    test_check((angles.pitch_deg == angles.pitch_deg) &&
        (angles.roll_deg == angles.roll_deg) &&
        (angles.yaw_deg == angles.yaw_deg),
        "acceleration shock does not produce NaN");
    test_check(imu_attitude_update(
        &attitude, &sample, 0.20f, &angles) ==
        ML_STATUS_INVALID_ARGUMENT,
        "out-of-range sample period is rejected");
}

static void test_axis_mapping(void)
{
    const imu_attitude_config_t mapping = {
        {IMU_ATTITUDE_AXIS_Y, IMU_ATTITUDE_AXIS_X, IMU_ATTITUDE_AXIS_Z},
        {1, 1, -1}
    };
    const icm42688_data_t sample = {
        0.0f, 0.0f, -1.0f,
        0.0f, 0.0f, 0.0f
    };
    imu_attitude_t attitude;
    imu_attitude_angles_t angles;

    test_check(imu_attitude_init(&attitude, &mapping) == ML_STATUS_OK,
        "axis permutation is accepted");
    test_check(test_calibrate(&attitude, &sample) ==
        IMU_ATTITUDE_CALIBRATION_COMPLETE,
        "mapped gravity calibration completes");
    test_check(imu_attitude_update(
        &attitude, &sample, 0.01f, &angles) == ML_STATUS_OK,
        "mapped attitude update succeeds");
    test_check(test_near(angles.pitch_deg, 0.0f,
        TEST_TOLERANCE_ANGLE_DEG) &&
        test_near(angles.roll_deg, 0.0f, TEST_TOLERANCE_ANGLE_DEG),
        "installed sensor mapping converts downward sensor Z to body Z");
}

int main(void)
{
    test_bias_and_yaw();
    test_tilt_initialisation();
    test_movement_and_invalid_inputs();
    test_axis_mapping();

    if (g_failures == 0) {
        printf("PASS: imu_attitude synthetic tests\n");
    }
    return g_failures == 0 ? 0 : 1;
}
