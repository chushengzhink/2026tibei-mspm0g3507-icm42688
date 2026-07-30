#include "chassis_track_line_test.h"

#include <float.h>
#include <math.h>

const chassis_track_line_test_config_t
    g_chassis_track_line_test_default_config = {
        {60.0f, 120.0f, 200.0f, 280.0f, 350.0f},
        0.0f,
        400.0f,
        20.0f,
        20U,
        3U
    };

static bool line_test_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float line_test_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float line_test_ramp(float current, float target, float step)
{
    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

static bool line_test_stopped_selectable(
    const chassis_track_line_test_t *test)
{
    return (test->state == CHASSIS_TRACK_LINE_TEST_READY) ||
        (test->state == CHASSIS_TRACK_LINE_TEST_COMPLETE);
}

static bool line_test_config_valid(
    const chassis_track_line_test_config_t *config)
{
    uint8_t index;

    if ((config == 0) || !line_test_float_valid(config->distance_mm) ||
        !line_test_float_valid(config->acceleration_mm_s2) ||
        !line_test_float_valid(config->stop_speed_mm_s) ||
        (config->distance_mm < 0.0f) ||
        (config->acceleration_mm_s2 <= 0.0f) ||
        (config->stop_speed_mm_s <= 0.0f) ||
        (config->control_period_ms == 0U) ||
        (config->stopped_cycles_required == 0U)) {
        return false;
    }
    for (index = 0U;
         index < CHASSIS_TRACK_LINE_TEST_SPEED_COUNT; ++index) {
        if (!line_test_float_valid(config->speed_mm_s[index]) ||
            (config->speed_mm_s[index] <= 0.0f)) {
            return false;
        }
    }
    return true;
}

static void line_test_fill_output(
    const chassis_track_line_test_t *test, uint32_t now_ms,
    chassis_track_line_test_output_t *output)
{
    output->state = test->state;
    output->requested_speed_mm_s = test->commanded_speed_mm_s;
    output->selected_speed_mm_s =
        test->config.speed_mm_s[test->speed_index];
    output->progress_mm = test->progress_mm;
    if (test->state == CHASSIS_TRACK_LINE_TEST_READY) {
        output->elapsed_ms = 0U;
    } else if (test->state == CHASSIS_TRACK_LINE_TEST_COMPLETE) {
        output->elapsed_ms = (uint32_t)
            (test->stop_time_ms - test->start_time_ms);
    } else {
        output->elapsed_ms = (uint32_t)
            (now_ms - test->start_time_ms);
    }
    output->speed_index = test->speed_index;
    output->can_start = chassis_track_line_test_can_start(test);
    output->command_stop =
        (test->state == CHASSIS_TRACK_LINE_TEST_BRAKING) ||
        (test->state == CHASSIS_TRACK_LINE_TEST_COMPLETE);
    output->finished =
        test->state == CHASSIS_TRACK_LINE_TEST_COMPLETE;
}

ml_status_t chassis_track_line_test_init(
    chassis_track_line_test_t *test,
    const chassis_track_line_test_config_t *config)
{
    if ((test == 0) || !line_test_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    test->config = *config;
    test->state = CHASSIS_TRACK_LINE_TEST_READY;
    test->start_distance_mm = 0.0f;
    test->progress_mm = 0.0f;
    test->commanded_speed_mm_s = 0.0f;
    test->start_time_ms = 0U;
    test->stop_time_ms = 0U;
    test->speed_index = 0U;
    test->stopped_cycles = 0U;
    test->initialized = true;
    return ML_STATUS_OK;
}

void chassis_track_line_test_speed_up(chassis_track_line_test_t *test)
{
    if ((test != 0) && test->initialized &&
        line_test_stopped_selectable(test) &&
        (test->speed_index + 1U <
         CHASSIS_TRACK_LINE_TEST_SPEED_COUNT)) {
        ++test->speed_index;
    }
}

void chassis_track_line_test_speed_down(chassis_track_line_test_t *test)
{
    if ((test != 0) && test->initialized &&
        line_test_stopped_selectable(test) &&
        (test->speed_index > 0U)) {
        --test->speed_index;
    }
}

bool chassis_track_line_test_can_start(
    const chassis_track_line_test_t *test)
{
    return (test != 0) && test->initialized &&
        line_test_stopped_selectable(test);
}

ml_status_t chassis_track_line_test_start(
    chassis_track_line_test_t *test, float center_distance_mm,
    uint32_t now_ms)
{
    if ((test == 0) || !test->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!line_test_float_valid(center_distance_mm) ||
        !chassis_track_line_test_can_start(test)) {
        return ML_STATUS_BUSY;
    }
    test->state = CHASSIS_TRACK_LINE_TEST_RUNNING;
    test->start_distance_mm = center_distance_mm;
    test->progress_mm = 0.0f;
    test->commanded_speed_mm_s = 0.0f;
    test->start_time_ms = now_ms;
    test->stop_time_ms = 0U;
    test->stopped_cycles = 0U;
    return ML_STATUS_OK;
}

ml_status_t chassis_track_line_test_update(
    chassis_track_line_test_t *test, float center_distance_mm,
    float measured_left_mm_s, float measured_right_mm_s,
    uint32_t now_ms, chassis_track_line_test_output_t *output)
{
    float remaining_mm;
    float target_speed_mm_s;
    float distance_speed_limit_mm_s;
    float step_mm_s;

    if ((test == 0) || !test->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((output == 0) || !line_test_float_valid(center_distance_mm) ||
        !line_test_float_valid(measured_left_mm_s) ||
        !line_test_float_valid(measured_right_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (test->state == CHASSIS_TRACK_LINE_TEST_RUNNING) {
        test->progress_mm = center_distance_mm -
            test->start_distance_mm;
        if (test->progress_mm < 0.0f) {
            test->progress_mm = 0.0f;
        }
        if ((test->config.distance_mm > 0.0f) &&
            (test->progress_mm >= test->config.distance_mm)) {
            test->state = CHASSIS_TRACK_LINE_TEST_BRAKING;
            test->commanded_speed_mm_s = 0.0f;
        } else {
            target_speed_mm_s =
                test->config.speed_mm_s[test->speed_index];
            if (test->config.distance_mm > 0.0f) {
                remaining_mm = test->config.distance_mm -
                    test->progress_mm;
                distance_speed_limit_mm_s = sqrtf(2.0f *
                    test->config.acceleration_mm_s2 * remaining_mm);
                if (target_speed_mm_s > distance_speed_limit_mm_s) {
                    target_speed_mm_s = distance_speed_limit_mm_s;
                }
            }
            step_mm_s = test->config.acceleration_mm_s2 *
                (float) test->config.control_period_ms / 1000.0f;
            test->commanded_speed_mm_s = line_test_ramp(
                test->commanded_speed_mm_s,
                target_speed_mm_s, step_mm_s);
        }
    } else if (test->state == CHASSIS_TRACK_LINE_TEST_BRAKING) {
        if ((line_test_abs(measured_left_mm_s) <
             test->config.stop_speed_mm_s) &&
            (line_test_abs(measured_right_mm_s) <
             test->config.stop_speed_mm_s)) {
            if (test->stopped_cycles < UINT8_MAX) {
                ++test->stopped_cycles;
            }
        } else {
            test->stopped_cycles = 0U;
        }
        if (test->stopped_cycles >=
            test->config.stopped_cycles_required) {
            test->state = CHASSIS_TRACK_LINE_TEST_COMPLETE;
            test->stop_time_ms = now_ms;
        }
    }
    line_test_fill_output(test, now_ms, output);
    return ML_STATUS_OK;
}
