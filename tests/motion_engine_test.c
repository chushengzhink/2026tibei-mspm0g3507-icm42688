#include "motion_engine.h"

#include <stdio.h>

static int g_failures;

static const robot_calibration_t g_calibration = {
    1.0f, 1.0f, 100.0f, 10.0f,
    1.0f, 1.0f, 1, 1, 1, 1, 0, 0U
};

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static line_sample_t line_sample(
    float error_mm, bool lost, bool transverse, bool centered)
{
    line_sample_t line = {0U, 0U, 0U, error_mm,
        lost, transverse, centered};
    return line;
}

static void test_straight_and_stop(void)
{
    motion_engine_t engine;
    motion_engine_output_t output;
    motion_status_t status;
    line_sample_t line = line_sample(0.0f, false, false, true);

    check(motion_engine_init(&engine, &g_calibration, line) ==
        ML_STATUS_OK, "motion engine initializes");
    check(motion_engine_start_straight(&engine, 10.0f, 80.0f) ==
        ML_STATUS_OK, "straight command starts");
    check(motion_engine_start_straight(&engine, 10.0f, 80.0f) ==
        ML_STATUS_BUSY, "running engine rejects a second command");
    motion_engine_velocity_tick(&engine, 5, 5, &output);
    check(output.apply_speeds &&
        output.left_speed_mm_s == output.right_speed_mm_s,
        "straight mode emits balanced wheel targets");
    motion_engine_velocity_tick(&engine, 5, 5, &output);
    status = motion_engine_get_status(&engine);
    check(status.result == MOTION_RESULT_COMPLETE &&
        motion_engine_take_stop_request(&engine),
        "straight mode completes at its target and requests a stop");
    motion_engine_stop(&engine);
    check(motion_engine_get_status(&engine).result ==
        MOTION_RESULT_CANCELLED,
        "explicit stop reports cancellation");
}

static void test_line_and_find_a(void)
{
    motion_engine_t engine;
    motion_engine_output_t output;
    line_sample_t centered = line_sample(0.0f, false, false, true);
    line_sample_t offset = line_sample(10.0f, false, false, false);
    line_sample_t lost = line_sample(0.0f, true, false, false);
    line_sample_t transverse = line_sample(0.0f, false, true, true);
    uint16_t index;

    (void) motion_engine_init(&engine, &g_calibration, centered);
    (void) motion_engine_start_line(&engine, 100.0f, 100.0f, centered);
    motion_engine_line_tick(&engine, offset);
    motion_engine_velocity_tick(&engine, 1, 1, &output);
    check(output.left_speed_mm_s > output.right_speed_mm_s,
        "line error produces differential correction");
    for (index = 0U; index < 100U; ++index) {
        motion_engine_line_tick(&engine, lost);
    }
    check(motion_engine_get_status(&engine).fault ==
        MOTION_FAULT_LINE_LOST,
        "persistent line loss raises the configured fault");

    (void) motion_engine_start_find_a(&engine, 80.0f);
    for (index = 0U; index < 3U; ++index) {
        motion_engine_line_tick(&engine, transverse);
    }
    check(engine.find_stage == MOTION_FIND_A_ON_LINE,
        "three transverse samples enter the A-line stage");
    for (index = 0U; index < 3U; ++index) {
        motion_engine_line_tick(&engine, centered);
    }
    check(engine.find_stage == MOTION_FIND_A_TO_AXLE &&
        engine.target_distance_mm == 10.0f,
        "clearing the A line creates the sensor-to-axle target");
}

static void test_turn_circle_and_faults(void)
{
    motion_engine_t engine;
    motion_engine_output_t output;
    line_sample_t line = line_sample(0.0f, false, false, true);
    uint16_t index;

    (void) motion_engine_init(&engine, &g_calibration, line);
    check(motion_engine_start_turn90(&engine, true, false) ==
        ML_STATUS_OK, "turn command starts");
    motion_engine_velocity_tick(&engine, 40, -40, &output);
    check(output.left_speed_mm_s > 0.0f &&
        output.right_speed_mm_s < 0.0f,
        "right turn emits opposing wheel targets");
    motion_engine_velocity_tick(&engine, 40, -40, &output);
    check(motion_engine_get_status(&engine).result ==
        MOTION_RESULT_COMPLETE,
        "turn completes from mean absolute wheel distance");

    check(motion_engine_start_circle(&engine, 299U) ==
        ML_STATUS_INVALID_ARGUMENT &&
        motion_engine_start_circle(&engine, 601U) ==
        ML_STATUS_INVALID_ARGUMENT,
        "circle radius remains limited to 300-600 mm");
    check(motion_engine_start_circle(&engine, 300U) == ML_STATUS_OK,
        "valid circle command starts");
    motion_engine_velocity_tick(&engine, 1, 1, &output);
    check(output.apply_speeds &&
        output.left_speed_mm_s > output.right_speed_mm_s,
        "circle command applies outer and inner wheel ratios");
    motion_engine_velocity_tick(&engine, 10000, 10000, &output);
    check(motion_engine_get_status(&engine).result ==
        MOTION_RESULT_COMPLETE,
        "circle completes only after both wheel targets are reached");

    (void) motion_engine_start_circle(&engine, 300U);
    for (index = 0U; index < 40U; ++index) {
        motion_engine_velocity_tick(&engine, 50, 1, &output);
    }
    check(motion_engine_get_status(&engine).fault ==
        MOTION_FAULT_CIRCLE_SYNC,
        "sustained circle progress mismatch raises a sync fault");

    (void) motion_engine_start_straight(&engine, 1000.0f, 80.0f);
    for (index = 0U; index < 12U; ++index) {
        motion_engine_velocity_tick(&engine, 0, 0, &output);
    }
    check(motion_engine_get_status(&engine).fault ==
        MOTION_FAULT_ENCODER_STALL,
        "eight qualified stationary updates raise the stall fault");
}

int main(void)
{
    test_straight_and_stop();
    test_line_and_find_a();
    test_turn_circle_and_faults();
    if (g_failures == 0) {
        printf("PASS: motion engine tests\n");
    }
    return g_failures == 0 ? 0 : 1;
}
