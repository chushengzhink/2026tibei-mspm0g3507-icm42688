#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "h456_mission.h"

static void stop_after_braking(h456_mission_t *mission,
    h456_mission_output_t *output, float distance_mm,
    float heading_deg, uint32_t *now_ms)
{
    uint32_t guard;

    for (guard = 0U; guard < 200U; ++guard) {
        *now_ms += mission->config.control_period_ms;
        assert(h456_mission_update(mission, distance_mm,
            mission->commanded_speed_mm_s == 0.0f ? 0.0f : 40.0f,
            mission->commanded_speed_mm_s == 0.0f ? 0.0f : 40.0f,
            heading_deg, *now_ms, false, output) == ML_STATUS_OK);
        if (output->finished) {
            return;
        }
    }
    assert(0);
}

static void test_h4_pass_time_and_line_only_braking(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    uint32_t now_ms = 1000U;
    uint32_t score_ms;
    uint32_t i;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_4,
        100.0f, 10.0f, now_ms) == ML_STATUS_OK);
    for (i = 0U; i < 100U; ++i) {
        now_ms += 20U;
        assert(h456_mission_update(&mission,
            100.0f + (float) (i * 10U), 200.0f, 200.0f,
            10.0f, now_ms, false, &output) == ML_STATUS_OK);
    }
    assert(fabsf(output.linear_mm_s - 220.0f) < 0.01f);
    now_ms = 8550U;
    assert(h456_mission_update(&mission, 1600.0f,
        220.0f, 220.0f, 10.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.state == H456_MISSION_BRAKING);
    assert(output.score_point_passed);
    assert(output.score_elapsed_ms == 7550U);
    assert(output.score_elapsed_ms < 8000U);
    assert(output.linear_mm_s < 220.0f);
    assert(output.linear_mm_s > 0.0f);
    assert(output.route_feedforward_rad_s == 0.0f);
    assert(output.heading_feedback_rad_s == 0.0f);
    score_ms = output.score_elapsed_ms;
    stop_after_braking(&mission, &output, 1700.0f, 20.0f, &now_ms);
    assert(output.state == H456_MISSION_COMPLETE);
    assert(output.result == H456_MISSION_RESULT_PASS);
    assert(output.score_elapsed_ms == score_ms);
    assert(output.command_stop);
}

static void test_h4_timeout_and_emergency(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    uint32_t now_ms = 0U;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_4,
        0.0f, 0.0f, now_ms) == ML_STATUS_OK);
    now_ms = 8000U;
    assert(h456_mission_update(&mission, 1400.0f,
        200.0f, 200.0f, 0.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.state == H456_MISSION_BRAKING);
    assert(!output.score_point_passed);
    stop_after_braking(&mission, &output, 1450.0f, 0.0f, &now_ms);
    assert(output.result == H456_MISSION_RESULT_TIME_LIMIT);

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_4,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    assert(h456_mission_update(&mission, 100.0f,
        100.0f, 100.0f, 0.0f, 20U, true,
        &output) == ML_STATUS_OK);
    assert(output.state == H456_MISSION_FAULT_EMERGENCY);
    assert(output.result == H456_MISSION_RESULT_FAULT);
    assert(output.command_stop);
}

static void test_mode_specific_cruise_speeds(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    h456_mode_t modes[3] = {
        H456_MODE_4, H456_MODE_5, H456_MODE_6
    };
    float expected[3] = {220.0f, 240.0f, 240.0f};
    uint32_t settle_cycles[3] = {100U, 90U, 90U};
    uint8_t mode_index;
    uint32_t i;
    uint32_t now_ms;

    for (mode_index = 0U; mode_index < 3U; ++mode_index) {
        now_ms = 0U;
        assert(h456_mission_init(&mission,
            &g_h456_mission_default_config) == ML_STATUS_OK);
        assert(h456_mission_start(&mission, modes[mode_index],
            0.0f, 0.0f, now_ms) == ML_STATUS_OK);
        for (i = 0U; i < settle_cycles[mode_index]; ++i) {
            now_ms += 20U;
            assert(h456_mission_update(&mission, 0.0f,
                expected[mode_index], expected[mode_index], 0.0f,
                now_ms, false, &output) == ML_STATUS_OK);
        }
        assert(output.state == H456_MISSION_RUNNING);
        assert(fabsf(output.linear_mm_s - expected[mode_index]) < 0.01f);
    }
}

static void test_h4_soft_launch_acceleration(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    uint32_t now_ms = 0U;
    uint32_t i;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_4,
        0.0f, 0.0f, now_ms) == ML_STATUS_OK);

    for (i = 0U; i < 60U; ++i) {
        now_ms += 20U;
        assert(h456_mission_update(&mission, 0.0f,
            0.0f, 0.0f, 0.0f, now_ms, false, &output) ==
            ML_STATUS_OK);
    }
    assert(fabsf(output.linear_mm_s - 120.0f) < 0.01f);

    now_ms += 20U;
    assert(h456_mission_update(&mission, 0.0f,
        0.0f, 0.0f, 0.0f, now_ms, false, &output) == ML_STATUS_OK);
    assert(fabsf(output.linear_mm_s - 123.0f) < 0.01f);
}

static void test_h5_h6_keep_normal_acceleration(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    h456_mode_t modes[2] = {H456_MODE_5, H456_MODE_6};
    uint8_t mode_index;
    uint32_t now_ms;

    for (mode_index = 0U; mode_index < 2U; ++mode_index) {
        now_ms = 0U;
        assert(h456_mission_init(&mission,
            &g_h456_mission_default_config) == ML_STATUS_OK);
        assert(h456_mission_start(&mission, modes[mode_index],
            0.0f, 0.0f, now_ms) == ML_STATUS_OK);
        now_ms += 20U;
        assert(h456_mission_update(&mission, 0.0f,
            0.0f, 0.0f, 0.0f, now_ms, false, &output) ==
            ML_STATUS_OK);
        assert(fabsf(output.linear_mm_s - 3.0f) < 0.01f);
    }
}

static void test_h4_soft_launch_still_reaches_b_with_time_margin(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    uint32_t now_ms = 0U;
    float distance_mm = 0.0f;
    uint16_t guard;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_4,
        0.0f, 0.0f, now_ms) == ML_STATUS_OK);
    for (guard = 0U; guard < 500U; ++guard) {
        now_ms += 20U;
        assert(h456_mission_update(&mission, distance_mm,
            mission.commanded_speed_mm_s, mission.commanded_speed_mm_s,
            0.0f, now_ms, false, &output) == ML_STATUS_OK);
        distance_mm += output.linear_mm_s *
            ((float) mission.config.control_period_ms / 1000.0f);
        if (output.score_point_passed) {
            break;
        }
    }
    assert(output.score_point_passed);
    assert(output.score_elapsed_ms < 8000U);
}

static void establish_lap_heading_gate(h456_mission_t *mission,
    h456_mission_output_t *output, uint32_t *now_ms)
{
    uint8_t i;

    for (i = 0U; i < 3U; ++i) {
        *now_ms += 20U;
        assert(h456_mission_update(mission, 5800.0f,
            240.0f, 240.0f, 360.0f, *now_ms, false,
            output) == ML_STATUS_OK);
    }
    assert(output->heading_gate_met);
}

static void test_h5_h6_lap_gate_and_score_time(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;
    uint32_t now_ms = 25000U;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_5,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    establish_lap_heading_gate(&mission, &output, &now_ms);
    now_ms = 26400U;
    assert(h456_mission_update(&mission, 5932.0f,
        240.0f, 240.0f, 360.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.state == H456_MISSION_BRAKING);
    assert(output.score_elapsed_ms == 26400U);
    assert(output.score_point_passed);
    stop_after_braking(&mission, &output, 6120.0f, 360.0f, &now_ms);
    assert(output.result == H456_MISSION_RESULT_PASS);

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_6,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    now_ms = 29500U;
    establish_lap_heading_gate(&mission, &output, &now_ms);
    assert(h456_mission_update(&mission, 5932.0f,
        240.0f, 240.0f, 360.0f, 30060U, false,
        &output) == ML_STATUS_OK);
    assert(output.score_point_passed);
    assert(output.score_elapsed_ms == 30060U);
    now_ms = 30060U;
    stop_after_braking(&mission, &output, 6120.0f, 360.0f, &now_ms);
    assert(output.result == H456_MISSION_RESULT_TIME_LIMIT);
}

static void test_lap_overrun_without_heading_gate(void)
{
    h456_mission_t mission;
    h456_mission_output_t output;

    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, H456_MODE_5,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    assert(h456_mission_update(&mission, 5982.0f,
        240.0f, 240.0f, 330.0f, 25000U, false,
        &output) == ML_STATUS_OK);
    assert(output.state == H456_MISSION_FAULT_LAP_GATE);
    assert(output.result == H456_MISSION_RESULT_FAULT);
    assert(!output.score_point_passed);
}

static void test_invalid_inputs(void)
{
    h456_mission_t mission;
    h456_mission_config_t config = g_h456_mission_default_config;

    assert(h456_mission_init(0, &config) ==
        ML_STATUS_INVALID_ARGUMENT);
    config.cruise_speed_mm_s = 0.0f;
    assert(h456_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT);
    config = g_h456_mission_default_config;
    config.h4_cruise_speed_mm_s = 0.0f;
    assert(h456_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT);
    config = g_h456_mission_default_config;
    config.h4_launch_acceleration_mm_s2 = 0.0f;
    assert(h456_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT);
    config = g_h456_mission_default_config;
    config.h4_launch_acceleration_ms = 0U;
    assert(h456_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(h456_mission_init(&mission,
        &g_h456_mission_default_config) == ML_STATUS_OK);
    assert(h456_mission_start(&mission, (h456_mode_t) 7,
        0.0f, 0.0f, 0U) == ML_STATUS_BUSY);
}

int main(void)
{
    test_invalid_inputs();
    test_mode_specific_cruise_speeds();
    test_h4_soft_launch_acceleration();
    test_h5_h6_keep_normal_acceleration();
    test_h4_soft_launch_still_reaches_b_with_time_margin();
    test_h4_pass_time_and_line_only_braking();
    test_h4_timeout_and_emergency();
    test_h5_h6_lap_gate_and_score_time();
    test_lap_overrun_without_heading_gate();
    printf("H456 mission tests passed\n");
    return 0;
}
