#include "chassis_heading_fusion.h"

#include <math.h>
#include <stdio.h>

#define TEST_PI (3.14159265358979323846f)

static int g_failures;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static chassis_heading_fusion_config_t test_config(void)
{
    chassis_heading_fusion_config_t config;

    config.left_mm_per_tick = 1.0f;
    config.right_mm_per_tick = 1.0f;
    config.effective_track_mm = 100.0f;
    config.heading_time_constant_s = 1.0f;
    config.imu_rate_weight = 0.75f;
    config.imu_heading_sign = -1.0f;
    config.imu_max_delta_deg = 25.0f;
    config.imu_stale_ms = 100U;
    return config;
}

static void test_sign_and_agreement(void)
{
    chassis_heading_fusion_config_t config = test_config();
    chassis_heading_fusion_t fusion;
    uint32_t timestamp = 10U;
    uint8_t index;

    check(chassis_heading_fusion_init(&fusion, &config) == ML_STATUS_OK,
        "heading fusion initializes");
    for (index = 0U; index < 50U; ++index) {
        chassis_heading_fusion_set_imu(&fusion, -57.29578f,
            -57.29578f, timestamp, true);
        check(chassis_heading_fusion_update(&fusion,
            -1, 1, 1U, 20U) == ML_STATUS_OK,
            "matching encoder and gyro updates succeed");
        timestamp += 20U;
    }
    check(fabsf(chassis_heading_fusion_heading(&fusion) - 1.0f) < 0.02f,
        "negative IMU yaw rate maps to positive encoder heading");
    check(fabsf(chassis_heading_fusion_yaw_rate(&fusion) - 1.0f) < 0.02f,
        "fused yaw rate agrees with matching sensors");
    check(chassis_heading_fusion_active(&fusion),
        "fresh valid IMU data activates fusion");
}

static void test_transient_and_encoder_anchor(void)
{
    chassis_heading_fusion_config_t config = test_config();
    chassis_heading_fusion_t fusion;
    float peak_heading;
    uint32_t timestamp = 10U;
    uint8_t index;

    (void) chassis_heading_fusion_init(&fusion, &config);
    for (index = 0U; index < 10U; ++index) {
        chassis_heading_fusion_set_imu(&fusion, -1.0f,
            -50.0f, timestamp, true);
        (void) chassis_heading_fusion_update(&fusion, 0, 0, 1U, 20U);
        timestamp += 20U;
    }
    peak_heading = chassis_heading_fusion_heading(&fusion);
    check(peak_heading > 0.12f,
        "gyro transient moves fused heading before encoder correction");
    for (index = 0U; index < 50U; ++index) {
        chassis_heading_fusion_set_imu(&fusion, -1.0f,
            0.0f, timestamp, true);
        (void) chassis_heading_fusion_update(&fusion, 0, 0, 1U, 20U);
        timestamp += 20U;
    }
    check(chassis_heading_fusion_heading(&fusion) < peak_heading * 0.40f,
        "encoder low-frequency anchor removes gyro-only displacement");
}

static void test_stale_fallback_and_recovery(void)
{
    chassis_heading_fusion_config_t config = test_config();
    chassis_heading_fusion_t fusion;
    float before_fallback;
    float after_fallback;
    uint8_t index;

    (void) chassis_heading_fusion_init(&fusion, &config);
    chassis_heading_fusion_set_imu(&fusion, 0.0f, 0.0f, 10U, true);
    (void) chassis_heading_fusion_update(&fusion, 0, 0, 1U, 20U);
    before_fallback = chassis_heading_fusion_heading(&fusion);
    for (index = 0U; index < 6U; ++index) {
        (void) chassis_heading_fusion_update(&fusion, 1, 2, 1U, 20U);
    }
    after_fallback = chassis_heading_fusion_heading(&fusion);
    check(!chassis_heading_fusion_active(&fusion),
        "stale IMU data falls back to encoders");
    check(after_fallback > before_fallback,
        "fallback advances continuously with encoder increments");
    chassis_heading_fusion_set_imu(&fusion, 0.0f, 0.0f, 200U, true);
    (void) chassis_heading_fusion_update(&fusion, 0, 0, 1U, 20U);
    check(chassis_heading_fusion_active(&fusion) &&
        fabsf(chassis_heading_fusion_heading(&fusion) -
            after_fallback) < 0.02f,
        "IMU recovery reactivates without a heading jump");
}

static void test_jump_rejection(void)
{
    chassis_heading_fusion_config_t config = test_config();
    chassis_heading_fusion_t fusion;

    (void) chassis_heading_fusion_init(&fusion, &config);
    chassis_heading_fusion_set_imu(&fusion, 0.0f,
        -2000.0f, 10U, true);
    (void) chassis_heading_fusion_update(&fusion, 0, 0, 1U, 20U);
    check(!chassis_heading_fusion_active(&fusion) &&
        fabsf(chassis_heading_fusion_heading(&fusion)) < 0.001f,
        "single-cycle IMU changes above 25 degrees are rejected");
    check(chassis_heading_fusion_reset(&fusion,
        4.0f * TEST_PI) == ML_STATUS_OK,
        "continuous heading reset accepts multi-turn angles");
}

static void test_wrap_crossing_is_continuous(void)
{
    chassis_heading_fusion_config_t config = test_config();
    chassis_heading_fusion_t fusion;

    (void) chassis_heading_fusion_init(&fusion, &config);
    (void) chassis_heading_fusion_reset(&fusion,
        179.0f * TEST_PI / 180.0f);
    chassis_heading_fusion_set_imu(&fusion, -1.0f,
        -57.29578f, 10U, true);
    (void) chassis_heading_fusion_update(&fusion, -1, 1, 1U, 20U);
    check(chassis_heading_fusion_heading(&fusion) > TEST_PI &&
        chassis_heading_fusion_heading(&fusion) < TEST_PI + 0.05f,
        "fused heading remains continuous across the 180 degree boundary");
}

int main(void)
{
    test_sign_and_agreement();
    test_transient_and_encoder_anchor();
    test_stale_fallback_and_recovery();
    test_jump_rejection();
    test_wrap_crossing_is_continuous();
    if (g_failures == 0) {
        puts("chassis heading fusion tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
