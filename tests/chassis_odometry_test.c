#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "chassis_odometry.h"

#define TEST_PI (3.14159265358979323846f)

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void test_straight(void)
{
    chassis_odometry_t odometry;
    chassis_pose_t pose;

    assert(chassis_odometry_init(&odometry, 0.5f, 0.5f, 100.0f) ==
        ML_STATUS_OK);
    assert(chassis_odometry_update(&odometry, 200, 200) == ML_STATUS_OK);
    pose = chassis_odometry_get(&odometry);
    assert(near(pose.x_mm, 100.0f, 0.001f));
    assert(near(pose.y_mm, 0.0f, 0.001f));
    assert(near(pose.heading_rad, 0.0f, 0.001f));
}

static void test_in_place_rotation(void)
{
    chassis_odometry_t odometry;
    chassis_pose_t pose;
    const float mm_per_tick = 0.1f;
    const float track_mm = 100.0f;
    int32_t ticks = (int32_t) ((TEST_PI * track_mm * 0.25f) /
        mm_per_tick);

    assert(chassis_odometry_init(&odometry,
        mm_per_tick, mm_per_tick, track_mm) == ML_STATUS_OK);
    assert(chassis_odometry_update(&odometry, -ticks, ticks) ==
        ML_STATUS_OK);
    pose = chassis_odometry_get(&odometry);
    assert(near(pose.x_mm, 0.0f, 0.001f));
    assert(near(pose.y_mm, 0.0f, 0.001f));
    assert(near(pose.heading_rad, TEST_PI * 0.5f, 0.002f));
}

static void test_arc_and_reset(void)
{
    chassis_odometry_t odometry;
    chassis_pose_t pose;

    assert(chassis_odometry_init(&odometry, 1.0f, 1.0f, 100.0f) ==
        ML_STATUS_OK);
    assert(chassis_odometry_update(&odometry, 50, 150) == ML_STATUS_OK);
    pose = chassis_odometry_get(&odometry);
    assert(near(pose.heading_rad, 1.0f, 0.001f));
    assert(near(pose.x_mm, 87.758f, 0.01f));
    assert(near(pose.y_mm, 47.943f, 0.01f));
    assert(chassis_odometry_reset(&odometry, 10.0f, 20.0f,
        4.0f * TEST_PI) == ML_STATUS_OK);
    pose = chassis_odometry_get(&odometry);
    assert(near(pose.x_mm, 10.0f, 0.001f));
    assert(near(pose.y_mm, 20.0f, 0.001f));
    assert(near(pose.heading_rad, 0.0f, 0.001f));
}

static void test_fused_heading_position_update(void)
{
    chassis_odometry_t odometry;
    chassis_pose_t pose;

    assert(chassis_odometry_init(&odometry, 1.0f, 1.0f, 100.0f) ==
        ML_STATUS_OK);
    assert(chassis_odometry_update_with_heading(&odometry,
        100, 100, TEST_PI * 0.5f) == ML_STATUS_OK);
    pose = chassis_odometry_get(&odometry);
    assert(near(pose.encoder_heading_rad, 0.0f, 0.001f));
    assert(near(pose.heading_rad, TEST_PI * 0.5f, 0.001f));
    assert(near(pose.x_mm, 70.711f, 0.02f));
    assert(near(pose.y_mm, 70.711f, 0.02f));
}

int main(void)
{
    test_straight();
    test_in_place_rotation();
    test_arc_and_reset();
    test_fused_heading_position_update();
    puts("chassis odometry tests passed");
    return 0;
}
