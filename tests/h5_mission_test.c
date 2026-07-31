#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "h5_mission.h"
#include "h5_config.h"

static void test_curve_speed_and_finish(void)
{
    h5_mission_t mission;
    h5_mission_output_t output;
    uint32_t now_ms = 0U;
    uint32_t i;

    assert(h5_mission_init(&mission,
        &g_h5_mission_default_config) == ML_STATUS_OK);
    assert(h5_mission_start(&mission, H5_MODE_5,
        0.0f, 0.0f, now_ms) == ML_STATUS_OK);

    for (i = 0U; i < 100U; ++i) {
        now_ms += mission.config.control_period_ms;
        assert(h5_mission_update(&mission, 1000.0f,
            240.0f, 240.0f, 0.0f, now_ms, false,
            &output) == ML_STATUS_OK);
    }
    assert(fabsf(output.linear_mm_s - 240.0f) < 0.01f);

    now_ms += mission.config.control_period_ms;
    assert(h5_mission_update(&mission, 2000.0f,
        210.0f, 210.0f, 90.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.linear_mm_s < 240.0f);
    assert(output.route_feedforward_rad_s > 0.0f);
    for (i = 0U; i < 20U; ++i) {
        now_ms += mission.config.control_period_ms;
        assert(h5_mission_update(&mission, 2000.0f,
            210.0f, 210.0f, 90.0f, now_ms, false,
            &output) == ML_STATUS_OK);
    }
    assert(fabsf(output.linear_mm_s - 210.0f) < 0.01f);

    for (i = 0U; i < 3U; ++i) {
        now_ms += mission.config.control_period_ms;
        assert(h5_mission_update(&mission, 5580.0f,
            210.0f, 210.0f, 360.0f, now_ms, false,
            &output) == ML_STATUS_OK);
    }
    assert(output.heading_gate_met);

    now_ms += mission.config.control_period_ms;
    assert(h5_mission_update(&mission, 5932.0f,
        210.0f, 210.0f, 360.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.score_point_passed);
    assert(output.state == H5_MISSION_BRAKING);
    assert(output.score_elapsed_ms <= H5_TIME_LIMIT_MS);

    for (i = 0U; i < 200U && !output.finished; ++i) {
        now_ms += mission.config.control_period_ms;
        assert(h5_mission_update(&mission, 5932.0f,
            0.0f, 0.0f, 360.0f, now_ms, false,
            &output) == ML_STATUS_OK);
    }
    assert(output.finished);
    assert(output.result == H5_MISSION_RESULT_PASS);
}

static void test_timeout_and_emergency(void)
{
    h5_mission_t mission;
    h5_mission_output_t output;

    assert(h5_mission_init(&mission,
        &g_h5_mission_default_config) == ML_STATUS_OK);
    assert(h5_mission_start(&mission, H5_MODE_5,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    assert(h5_mission_update(&mission, 100.0f,
        100.0f, 100.0f, 0.0f, H5_TIME_LIMIT_MS,
        false, &output) == ML_STATUS_OK);
    assert(output.result == H5_MISSION_RESULT_TIME_LIMIT ||
        output.state == H5_MISSION_BRAKING);

    assert(h5_mission_init(&mission,
        &g_h5_mission_default_config) == ML_STATUS_OK);
    assert(h5_mission_start(&mission, H5_MODE_5,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    assert(h5_mission_update(&mission, 100.0f,
        100.0f, 100.0f, 0.0f, 20U,
        true, &output) == ML_STATUS_OK);
    assert(output.result == H5_MISSION_RESULT_FAULT);
    assert(output.finished);
}

static void test_marker_can_confirm_after_heading_gate(void)
{
    h5_mission_t mission;
    h5_mission_output_t output;
    uint32_t now_ms = 0U;
    uint32_t i;

    assert(h5_mission_init(&mission,
        &g_h5_mission_default_config) == ML_STATUS_OK);
    assert(h5_mission_start(&mission, H5_MODE_5,
        0.0f, 0.0f, 0U) == ML_STATUS_OK);
    for (i = 0U; i < 3U; ++i) {
        now_ms += mission.config.control_period_ms;
        assert(h5_mission_update(&mission, 5580.0f,
            210.0f, 210.0f, 360.0f, now_ms, false,
            &output) == ML_STATUS_OK);
    }
    h5_mission_mark_a_line(&mission);
    now_ms += mission.config.control_period_ms;
    assert(h5_mission_update(&mission, 5800.0f,
        210.0f, 210.0f, 360.0f, now_ms, false,
        &output) == ML_STATUS_OK);
    assert(output.score_point_passed);
}

int main(void)
{
    test_curve_speed_and_finish();
    test_timeout_and_emergency();
    test_marker_can_confirm_after_heading_gate();
    puts("H5 mission tests passed");
    return 0;
}
