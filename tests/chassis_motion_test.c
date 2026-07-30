#include "chassis_motion.h"

#include <math.h>
#include <stdio.h>

static int g_failures;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static chassis_config_t test_config(void)
{
    chassis_config_t config = g_chassis_default_config;

    config.left.mm_per_tick = 1.0f;
    config.right.mm_per_tick = 1.0f;
    config.effective_track_mm = 100.0f;
    config.minimum_profile_speed_mm_s = 1.0f;
    config.maximum_wheel_speed_mm_s = 500.0f;
    config.distance_tolerance_mm = 0.6f;
    config.angle_tolerance_deg = 1.0f;
    return config;
}

static void finish_three_cycles(chassis_motion_t *motion)
{
    (void) chassis_motion_update(motion, 0, 0, 1U);
    (void) chassis_motion_update(motion, 0, 0, 1U);
}

static void test_default_hardware_config(void)
{
    check(g_chassis_default_config.left.encoder_sign == -1 &&
        g_chassis_default_config.left.motor_sign == -1 &&
        g_chassis_default_config.right.encoder_sign == -1 &&
        g_chassis_default_config.right.motor_sign == 1,
        "default wheel signs match the intended chassis directions");
    check(fabsf(g_chassis_default_config.left.mm_per_tick - 0.1413727f) <
        0.000001f &&
        fabsf(g_chassis_default_config.right.mm_per_tick - 0.1434926f) <
        0.000001f,
        "default wheel distances match the provisional rolling calibration");
    check(fabsf(g_chassis_default_config.effective_track_mm - 214.2f) <
        0.001f,
        "default effective track matches the measured chassis width");
    check(g_chassis_default_config.imu_heading_sign == -1.0f &&
        fabsf(g_chassis_default_config.heading_fusion_time_constant_s -
            1.0f) < 0.001f &&
        fabsf(g_chassis_default_config.heading_rate_imu_weight -
            0.75f) < 0.001f,
        "default fusion parameters match the measured heading convention");
    check(g_chassis_default_config.left.pid_output_limit == 11500.0f &&
        g_chassis_default_config.left.pid_integral_limit == 5750.0f &&
        g_chassis_race_config.left.pid_output_limit == 14000.0f &&
        g_chassis_race_config.left.pid_integral_limit == 7000.0f,
        "self-test remains 35 percent while race config can reach 40 percent");
}

static void test_move(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};

    check(chassis_motion_init(&motion, &config) == ML_STATUS_OK,
        "motion core initializes");
    check(chassis_motion_move(&motion, 10.0f, 100.0f) == ML_STATUS_OK,
        "distance command starts");
    (void) chassis_motion_update(&motion, 5, 5, 1U);
    check(motion.status.target_left_mm_s > 0.0f &&
        motion.status.target_right_mm_s > 0.0f &&
        motion.status.target_left_mm_s <= 8.01f,
        "distance command accelerates without a speed step");
    (void) chassis_motion_update(&motion, 5, 5, 1U);
    finish_three_cycles(&motion);
    check(motion.status.result == CHASSIS_RESULT_COMPLETE &&
        fabsf(motion.status.pose.x_mm - 10.0f) < 0.01f,
        "distance completion requires three in-tolerance updates");
}

static void test_rotate_and_arc(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_rotate(&motion, 3.14159265f * 0.5f,
        2.0f) == ML_STATUS_OK, "rotation command starts");
    (void) chassis_motion_update(&motion, -79, 79, 1U);
    finish_three_cycles(&motion);
    check(motion.status.result == CHASSIS_RESULT_COMPLETE &&
        fabsf(motion.status.pose.heading_rad - 1.58f) < 0.02f,
        "in-place rotation updates and completes from odometry");

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_arc(&motion, 200.0f, 3.14159265f * 0.5f,
        100.0f) == ML_STATUS_OK, "arc command starts");
    (void) chassis_motion_update(&motion, 236, 393, 1U);
    finish_three_cycles(&motion);
    check(motion.status.result == CHASSIS_RESULT_COMPLETE &&
        motion.status.pose.x_mm > 100.0f &&
        motion.status.pose.y_mm > 100.0f,
        "arc command reaches the requested differential angle");
}

static void test_stall_and_emergency(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_set_wheel_speed(&motion, 100.0f, 100.0f,
        CHASSIS_MODE_WHEEL_SPEED) == ML_STATUS_OK,
        "direct wheel command starts");
    (void) chassis_motion_update(&motion, 0, 0, config.stall_cycles);
    check(motion.status.result == CHASSIS_RESULT_FAULT &&
        motion.status.fault == CHASSIS_FAULT_STALL,
        "eight empty control periods trigger stall protection");

    (void) chassis_motion_init(&motion, &config);
    chassis_motion_stop(&motion, true);
    check(motion.status.emergency_stop_latched &&
        chassis_motion_move(&motion, 10.0f, 50.0f) == ML_STATUS_BUSY,
        "emergency stop latches and blocks later motion");
}

static void test_continuous_velocity_update(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};
    uint8_t cycle;

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_set_wheel_speed(&motion, 100.0f, 100.0f,
        CHASSIS_MODE_VELOCITY) == ML_STATUS_OK,
        "velocity command starts once");
    for (cycle = 0U; cycle < config.stall_cycles; ++cycle) {
        check(chassis_motion_update_wheel_speed(&motion,
            100.0f, 100.0f, CHASSIS_MODE_VELOCITY) == ML_STATUS_OK,
            "active velocity command updates without restart");
        (void) chassis_motion_update(&motion, 0, 0, 1U);
    }
    check(motion.status.fault == CHASSIS_FAULT_STALL,
        "continuous updates preserve the 160 ms stall counter");
}

static void test_continuous_direct_wheel_update(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};
    uint8_t cycle;

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_set_wheel_speed(&motion, 80.0f, 120.0f,
        CHASSIS_MODE_WHEEL_SPEED) == ML_STATUS_OK,
        "direct wheel mode starts once");
    chassis_motion_set_imu_sample(&motion, 0.0f, -100.0f, 20U, true);
    for (cycle = 0U; cycle < config.stall_cycles; ++cycle) {
        check(chassis_motion_update_wheel_speed(&motion,
            80.0f, 120.0f,
            CHASSIS_MODE_WHEEL_SPEED) == ML_STATUS_OK,
            "active direct wheel targets update without restart");
        (void) chassis_motion_update(&motion, 0, 0, 1U);
        if (cycle == 0U) {
            check(motion.status.heading_fusion_active &&
                fabsf(motion.status.target_left_mm_s - 80.0f) < 0.001f &&
                fabsf(motion.status.target_right_mm_s - 120.0f) < 0.001f,
                "direct wheel mode records IMU but applies no yaw feedback");
        }
    }
    check(motion.status.fault == CHASSIS_FAULT_STALL,
        "direct wheel updates preserve the 160 ms stall counter");
}

static void test_heading_and_yaw_rate_feedback(void)
{
    chassis_config_t config = test_config();
    chassis_motion_t motion = {0};
    float target_difference;

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_move(&motion, 100.0f, 100.0f) == ML_STATUS_OK,
        "fusion straight feedback test starts");
    chassis_motion_set_imu_sample(&motion, -1.0f,
        -100.0f, 10U, true);
    (void) chassis_motion_update(&motion, 0, 0, 1U);
    target_difference = motion.status.target_right_mm_s -
        motion.status.target_left_mm_s;
    check(motion.status.heading_fusion_active &&
        target_difference < 0.0f,
        "positive fused yaw produces a negative straight correction");
    check(fabsf(target_difference) <=
        config.maximum_wheel_speed_mm_s *
            config.heading_correction_limit_ratio * 2.0f + 0.01f,
        "straight differential correction stays inside the 25 percent limit");

    (void) chassis_motion_init(&motion, &config);
    check(chassis_motion_set_wheel_speed(&motion,
        0.0f, 100.0f, CHASSIS_MODE_VELOCITY) == ML_STATUS_OK,
        "velocity yaw-rate feedback test starts");
    chassis_motion_set_imu_sample(&motion, 0.0f, 0.0f, 20U, true);
    (void) chassis_motion_update(&motion, 0, 0, 1U);
    check(motion.status.heading_fusion_active &&
        motion.status.target_left_mm_s < 0.0f &&
        motion.status.target_right_mm_s > 100.0f,
        "velocity mode corrects a measured yaw-rate deficit");
}

int main(void)
{
    test_default_hardware_config();
    test_move();
    test_rotate_and_arc();
    test_stall_and_emergency();
    test_continuous_velocity_update();
    test_continuous_direct_wheel_update();
    test_heading_and_yaw_rate_feedback();
    if (g_failures == 0) {
        printf("chassis motion tests passed\n");
    }
    return g_failures == 0 ? 0 : 1;
}
