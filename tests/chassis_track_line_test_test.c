#include "chassis_track_line_test.h"

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

static bool near_value(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void test_speed_selection_and_unconditional_start(void)
{
    chassis_track_line_test_t test = {0};

    check(chassis_track_line_test_init(&test,
              &g_chassis_track_line_test_default_config) == ML_STATUS_OK,
        "line-only diagnostic initializes");
    check(near_value(test.config.speed_mm_s[test.speed_index],
              60.0f, 0.001f),
        "line-only diagnostic defaults to 60 mm/s");
    check(chassis_track_line_test_can_start(&test),
        "READY permits Center start without an infrared pattern gate");
    chassis_track_line_test_speed_up(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              120.0f, 0.001f),
        "first Up selects 120 mm/s");
    chassis_track_line_test_speed_up(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              200.0f, 0.001f),
        "second Up selects 200 mm/s");
    chassis_track_line_test_speed_up(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              280.0f, 0.001f),
        "third Up selects 280 mm/s");
    chassis_track_line_test_speed_up(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              350.0f, 0.001f),
        "fourth Up selects 350 mm/s");
    chassis_track_line_test_speed_up(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              350.0f, 0.001f),
        "stopped Up selection saturates at 350 mm/s");
    chassis_track_line_test_speed_down(&test);
    check(near_value(test.config.speed_mm_s[test.speed_index],
              280.0f, 0.001f),
        "stopped Down selection returns to 280 mm/s");
    check(chassis_track_line_test_start(&test,
              50.0f, 100U) == ML_STATUS_OK,
        "Center starts immediately from READY");
    check(!chassis_track_line_test_can_start(&test),
        "RUNNING rejects a second start");
    chassis_track_line_test_speed_up(&test);
    chassis_track_line_test_speed_down(&test);
    check(test.speed_index == 3U,
        "direction keys cannot change speed while running");
}

static void test_unlimited_speed_profile(void)
{
    chassis_track_line_test_t test = {0};
    chassis_track_line_test_output_t output;

    (void) chassis_track_line_test_init(&test,
        &g_chassis_track_line_test_default_config);
    chassis_track_line_test_speed_up(&test);
    chassis_track_line_test_speed_up(&test);
    chassis_track_line_test_speed_up(&test);
    chassis_track_line_test_speed_up(&test);
    (void) chassis_track_line_test_start(&test, 100.0f, 1000U);
    (void) chassis_track_line_test_update(&test,
        100.0f, 0.0f, 0.0f, 1020U, &output);
    check(output.state == CHASSIS_TRACK_LINE_TEST_RUNNING &&
        near_value(output.requested_speed_mm_s, 8.0f, 0.001f),
        "line-only speed rises by 8 mm/s per 20 ms cycle");
    test.commanded_speed_mm_s = 350.0f;
    (void) chassis_track_line_test_update(&test,
        1100.0f, 350.0f, 350.0f, 1040U, &output);
    check(output.state == CHASSIS_TRACK_LINE_TEST_RUNNING &&
        !output.command_stop && !output.finished &&
        near_value(output.progress_mm, 1000.0f, 0.001f) &&
        near_value(output.requested_speed_mm_s, 350.0f, 0.001f),
        "default diagnostic runs beyond the former 1000 mm limit");
    (void) chassis_track_line_test_update(&test,
        6241.5928f, 350.0f, 350.0f, 20000U, &output);
    check(output.state == CHASSIS_TRACK_LINE_TEST_RUNNING &&
        !output.command_stop && !output.finished &&
        near_value(output.progress_mm, 6141.5928f, 0.01f),
        "default diagnostic continues through one nominal lap");
    (void) chassis_track_line_test_update(&test,
        7000.0f, 350.0f, 350.0f, 22000U, &output);
    check(output.state == CHASSIS_TRACK_LINE_TEST_RUNNING &&
        !output.command_stop && !output.finished,
        "default diagnostic has no automatic distance stop");
}

static void test_positive_distance_compatibility(void)
{
    chassis_track_line_test_config_t config =
        g_chassis_track_line_test_default_config;
    chassis_track_line_test_t test = {0};
    chassis_track_line_test_output_t output;

    config.distance_mm = 1000.0f;
    (void) chassis_track_line_test_init(&test, &config);
    (void) chassis_track_line_test_start(&test, 100.0f, 1000U);
    test.commanded_speed_mm_s = 120.0f;
    (void) chassis_track_line_test_update(&test,
        1090.0f, 120.0f, 120.0f, 1040U, &output);
    check(near_value(output.requested_speed_mm_s, 112.0f, 0.001f),
        "positive distance retains deceleration limiting");
    (void) chassis_track_line_test_update(&test,
        1100.0f, 120.0f, 120.0f, 1060U, &output);
    check(output.state == CHASSIS_TRACK_LINE_TEST_BRAKING &&
        output.command_stop &&
        near_value(output.progress_mm, 1000.0f, 0.001f),
        "positive distance retains automatic braking");
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 0.0f, 1080U, &output);
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 30.0f, 1100U, &output);
    check(test.stopped_cycles == 0U,
        "either wheel above 20 mm/s resets stopped confirmation");
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 0.0f, 1120U, &output);
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 0.0f, 1140U, &output);
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 0.0f, 1160U, &output);
    check(output.finished &&
        output.state == CHASSIS_TRACK_LINE_TEST_COMPLETE &&
        test.stop_time_ms == 1160U,
        "third stopped cycle completes the repeatable diagnostic run");
    (void) chassis_track_line_test_update(&test,
        1101.0f, 0.0f, 0.0f, 5000U, &output);
    check(output.elapsed_ms == 160U,
        "completed diagnostic time remains frozen at confirmed stop");
}

static void test_repeat_after_completion(void)
{
    chassis_track_line_test_t test = {0};

    (void) chassis_track_line_test_init(&test,
        &g_chassis_track_line_test_default_config);
    (void) chassis_track_line_test_start(&test, 0.0f, 0U);
    test.state = CHASSIS_TRACK_LINE_TEST_COMPLETE;
    check(chassis_track_line_test_can_start(&test),
        "COMPLETE permits repeat without an infrared pattern gate");
    check(chassis_track_line_test_start(&test,
              2000.0f, 2000U) == ML_STATUS_OK,
        "normal completion can be repositioned and started again");
}

int main(void)
{
    test_speed_selection_and_unconditional_start();
    test_unlimited_speed_profile();
    test_positive_distance_compatibility();
    test_repeat_after_completion();
    if (g_failures == 0) {
        puts("chassis track line test tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
