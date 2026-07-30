#include "chassis_track_mission.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI (3.14159265358979323846f)

#ifndef CHASSIS_TRACK_SPEED_STAGE
#define CHASSIS_TRACK_SPEED_STAGE (0U)
#endif

#if CHASSIS_TRACK_SPEED_STAGE == 0U
#define TEST_STRAIGHT_CRUISE_MM_S (360.0f)
#define TEST_CURVE_CRUISE_MM_S    (360.0f)
#define TEST_ACCELERATION_MM_S2   (400.0f)
#define TEST_LAP_TIME_LIMIT_S     (20.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == 1U
#define TEST_STRAIGHT_CRUISE_MM_S (380.0f)
#define TEST_CURVE_CRUISE_MM_S    (360.0f)
#define TEST_ACCELERATION_MM_S2   (400.0f)
#define TEST_LAP_TIME_LIMIT_S     (20.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == 2U
#define TEST_STRAIGHT_CRUISE_MM_S (400.0f)
#define TEST_CURVE_CRUISE_MM_S    (360.0f)
#define TEST_ACCELERATION_MM_S2   (400.0f)
#define TEST_LAP_TIME_LIMIT_S     (20.0f)
#else
#error "test speed stage must be 0, 1, or 2"
#endif

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

static void start_mission(chassis_track_mission_t *mission,
    float start_distance_mm, float start_heading_deg)
{
    check(chassis_track_mission_init(mission,
              &g_chassis_track_default_config) == ML_STATUS_OK,
        "mission initializes");
    check(chassis_track_mission_start(mission, start_distance_mm,
              start_heading_deg, 100U) == ML_STATUS_OK,
        "center key start records encoder and fused-heading baselines");
}

static void test_start_and_route_geometry(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float route = chassis_track_route_length(
        &g_chassis_track_default_config);

    check(near_value(route, 6141.5928f, 0.01f),
        "capsule route length is 6141.6 mm");
    check(near_value(g_chassis_track_default_config.
              finish_reference_progress_mm, 5932.0f, 0.001f) &&
        near_value(g_chassis_track_default_config.
              finish_stop_lead_mm, 13.0f, 0.001f),
        "finish reference and low-speed stop lead are calibrated separately");
    check(near_value(g_chassis_track_default_config.
              finish_reference_progress_mm -
              g_chassis_track_default_config.approach_distance_mm,
              5742.0f, 0.001f) &&
        near_value(g_chassis_track_default_config.
              finish_reference_progress_mm -
              g_chassis_track_default_config.finish_stop_lead_mm,
              5919.0f, 0.001f),
        "calibrated approach and braking thresholds are exact");
    check(near_value(g_chassis_track_default_config.
              finish_alignment_tolerance_deg, 1.0f, 0.001f) &&
        near_value(g_chassis_track_default_config.
              finish_alignment_heading_bias_deg, 37.0f, 0.001f) &&
        near_value(g_chassis_track_default_config.
              finish_alignment_max_start_error_deg, 45.0f, 0.001f) &&
        g_chassis_track_default_config.finish_alignment_timeout_ms ==
            3000U &&
        g_chassis_track_default_config.finish_alignment_confirm_cycles ==
            3U,
        "finish alignment uses the calibrated one-degree safety window");
    check(near_value(g_chassis_track_default_config.
              straight_cruise_speed_mm_s,
              TEST_STRAIGHT_CRUISE_MM_S, 0.001f) &&
        near_value(g_chassis_track_default_config.
              curve_cruise_speed_mm_s,
              TEST_CURVE_CRUISE_MM_S, 0.001f) &&
        near_value(g_chassis_track_default_config.
              acceleration_mm_s2,
              TEST_ACCELERATION_MM_S2, 0.001f),
        "selected stage sets the intended cruise speeds and acceleration");
    start_mission(&mission, 1234.0f, 47.0f);
    (void) chassis_track_mission_update(&mission,
        1334.0f, 0.0f, 0.0f, 57.0f, 120U, false, &output);
    check(near_value(output.progress_mm, 100.0f, 0.001f) &&
        near_value(output.heading_progress_deg, 10.0f, 0.001f),
        "progress is relative to the encoder and heading start snapshot");
}

static void test_expected_heading_profile(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float half_curve = TEST_PI * 500.0f;
    float second_straight_end = 3000.0f + half_curve;

    start_mission(&mission, 0.0f, 0.0f);
    (void) chassis_track_mission_update(&mission,
        750.0f, 0.0f, 0.0f, 0.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_AB &&
        near_value(output.expected_heading_deg, 0.0f, 0.01f),
        "AB holds zero degrees");
    (void) chassis_track_mission_update(&mission,
        1500.0f + half_curve * 0.5f,
        0.0f, 0.0f, 90.0f, 140U, false, &output);
    check(output.state == CHASSIS_TRACK_BC &&
        near_value(output.expected_heading_deg, 90.0f, 0.02f) &&
        output.angular_rad_s > 0.0f,
        "BC advances clockwise from zero to 180 degrees");
    (void) chassis_track_mission_update(&mission,
        1500.0f + half_curve + 500.0f,
        0.0f, 0.0f, 180.0f, 160U, false, &output);
    check(output.state == CHASSIS_TRACK_CD &&
        near_value(output.expected_heading_deg, 180.0f, 0.02f) &&
        near_value(output.angular_rad_s, 0.0f, 0.001f),
        "CD holds 180 degrees");
    (void) chassis_track_mission_update(&mission,
        second_straight_end + half_curve * 0.5f,
        0.0f, 0.0f, 270.0f, 180U, false, &output);
    check(output.state == CHASSIS_TRACK_DA &&
        near_value(output.expected_heading_deg, 270.0f, 0.02f) &&
        output.angular_rad_s > 0.0f,
        "DA advances clockwise from 180 to 360 degrees");
}

static void test_heading_feedback(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float quarter_curve = TEST_PI * 500.0f * 0.5f;
    float nominal_omega;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = TEST_CURVE_CRUISE_MM_S;
    (void) chassis_track_mission_update(&mission,
        1500.0f + quarter_curve,
        0.0f, 0.0f, 80.0f, 120U, false, &output);
    nominal_omega = output.linear_mm_s / 500.0f;
    check(near_value(output.angular_rad_s - nominal_omega,
              0.35f, 0.001f) &&
        near_value(output.route_feedforward_rad_s,
              nominal_omega, 0.001f) &&
        near_value(output.heading_feedback_rad_s, 0.35f, 0.001f) &&
        near_value(output.heading_error_deg, 10.0f, 0.02f),
        "lagging fused heading gets limited positive correction");
    (void) chassis_track_mission_update(&mission,
        1500.0f + quarter_curve,
        0.0f, 0.0f, 100.0f, 140U, false, &output);
    nominal_omega = output.linear_mm_s / 500.0f;
    check(near_value(output.angular_rad_s - nominal_omega,
              -0.35f, 0.001f) &&
        near_value(output.route_feedforward_rad_s,
              nominal_omega, 0.001f) &&
        near_value(output.heading_feedback_rad_s, -0.35f, 0.001f) &&
        near_value(output.heading_error_deg, -10.0f, 0.02f),
        "leading fused heading gets limited negative correction");
}

static void test_second_curve_fusion_replay(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = TEST_CURVE_CRUISE_MM_S;
    (void) chassis_track_mission_update(&mission,
        5455.8f, 0.0f, 0.0f, 294.32f, 120U, false, &output);

    check(output.state == CHASSIS_TRACK_DA &&
        near_value(output.expected_heading_deg, 281.41f, 0.02f),
        "second-curve replay uses encoder progress for expected heading");
    check(near_value(output.linear_mm_s,
              TEST_CURVE_CRUISE_MM_S, 0.001f) &&
        near_value(output.route_feedforward_rad_s,
              TEST_CURVE_CRUISE_MM_S / 500.0f, 0.001f) &&
        near_value(output.heading_feedback_rad_s, -0.35f, 0.001f) &&
        near_value(output.angular_rad_s,
              TEST_CURVE_CRUISE_MM_S / 500.0f - 0.35f,
              0.001f),
        "second-curve replay exposes feedforward and negative heading damping");
}

static void test_dual_finish_gate(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;
    float brake_distance = finish_reference -
        g_chassis_track_default_config.finish_stop_lead_mm;
    float heading_arm_start = 2.0f *
        g_chassis_track_default_config.straight_length_mm +
        TEST_PI * g_chassis_track_default_config.curve_radius_mm;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = 100.0f;
    (void) chassis_track_mission_update(&mission,
        heading_arm_start - 0.1f, 100.0f, 100.0f,
        360.0f, 120U, false, &output);
    (void) chassis_track_mission_update(&mission,
        heading_arm_start - 0.1f, 100.0f, 100.0f,
        360.0f, 140U, false, &output);
    (void) chassis_track_mission_update(&mission,
        heading_arm_start - 0.1f, 100.0f, 100.0f,
        360.0f, 160U, false, &output);
    check(!output.heading_gate_met,
        "finish heading is ignored before the second curve");
    (void) chassis_track_mission_update(&mission,
        brake_distance - 1.0f, 100.0f, 100.0f,
        360.0f, 180U, false, &output);
    (void) chassis_track_mission_update(&mission,
        brake_distance - 1.0f, 100.0f, 100.0f,
        360.0f, 200U, false, &output);
    (void) chassis_track_mission_update(&mission,
        brake_distance - 1.0f, 100.0f, 100.0f,
        360.0f, 220U, false, &output);
    check(!output.command_stop && !output.distance_gate_met &&
        output.heading_gate_met,
        "heading gate alone cannot stop the car");
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f,
        389.0f, 240U, false, &output);
    check(output.state == CHASSIS_TRACK_BRAKING &&
        output.command_stop && output.distance_gate_met &&
        output.heading_gate_met,
        "latched heading confirmation survives 389 degrees and permits braking");

    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 389.0f, 260U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 30.0f, 389.0f, 280U, false, &output);
    check(!output.finished && mission.stopped_cycles == 0U,
        "either wheel above 20 mm/s resets the stopped-cycle gate");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 389.0f, 300U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 389.0f, 320U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 389.0f, 340U, false, &output);
    check(!output.finished && output.state == CHASSIS_TRACK_ALIGNING &&
        !output.command_stop && output.angular_rad_s > 0.0f &&
        mission.stop_time_ms == 0U,
        "third braking stop cycle starts positive alignment from 389 degrees");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 396.0f, 360U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.0f, 380U, false, &output);
    check(!output.finished && mission.alignment_confirm_cycles == 2U,
        "alignment requires three new in-window stopped cycles");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f, 400U, false, &output);
    check(output.finished && output.state == CHASSIS_TRACK_COMPLETE &&
        output.command_stop && mission.stop_time_ms == 400U &&
        near_value(output.elapsed_s, 0.300f, 0.0001f) &&
        near_value(output.stop_error_mm, 0.0f, 0.001f),
        "completion time and stop error are recorded after final alignment");
}

static void test_finish_heading_window_bounds(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float brake_distance = g_chassis_track_default_config.
        finish_reference_progress_mm -
        g_chassis_track_default_config.finish_stop_lead_mm;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = 100.0f;
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f, 354.9f, 120U, false, &output);
    check(!output.heading_gate_met,
        "heading below 355 degrees is outside the finish window");
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f, 355.0f, 140U, false, &output);
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f, 365.0f, 160U, false, &output);
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f, 365.0f, 180U, false, &output);
    check(output.state == CHASSIS_TRACK_BRAKING &&
        output.heading_gate_met,
        "355 through 365 degrees are inclusive after three cycles");
}

static void enter_alignment(chassis_track_mission_t *mission,
    float heading_deg, chassis_track_output_t *output)
{
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    start_mission(mission, 0.0f, 0.0f);
    mission->state = CHASSIS_TRACK_BRAKING;
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, heading_deg,
        120U, false, output);
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, heading_deg,
        140U, false, output);
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, heading_deg,
        160U, false, output);
}

static void enter_alignment_from_start(chassis_track_mission_t *mission,
    float start_heading_deg, float current_heading_deg,
    chassis_track_output_t *output)
{
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    start_mission(mission, 0.0f, start_heading_deg);
    mission->state = CHASSIS_TRACK_BRAKING;
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, current_heading_deg,
        120U, false, output);
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, current_heading_deg,
        140U, false, output);
    (void) chassis_track_mission_update(mission,
        finish_reference, 0.0f, 0.0f, current_heading_deg,
        160U, false, output);
}

static void test_nonzero_start_heading_reference(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    enter_alignment_from_start(&mission, 47.0f, 456.77f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        output.angular_rad_s < 0.0f &&
        near_value(output.heading_error_deg, -12.77f, 0.02f),
        "nonzero start heading drives past the biased target negatively");

    enter_alignment_from_start(&mission, 47.0f, 434.0f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        output.angular_rad_s > 0.0f &&
        near_value(output.heading_error_deg, 10.0f, 0.02f),
        "nonzero start heading drives toward the biased target positively");

    enter_alignment_from_start(&mission, 47.0f, 443.0f, &output);
    check(mission.alignment_confirm_cycles == 1U &&
        near_value(output.heading_error_deg, 1.0f, 0.02f),
        "biased target negative one-degree boundary is included");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 445.0f,
        180U, false, &output);
    check(mission.alignment_confirm_cycles == 2U &&
        near_value(output.heading_error_deg, -1.0f, 0.02f),
        "biased target positive one-degree boundary is included");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 444.0f,
        200U, false, &output);
    check(output.finished &&
        strcmp(chassis_track_state_text(CHASSIS_TRACK_ALIGNING),
            "ALIGN TO START  ") == 0,
        "alignment completes at the bias-compensated start orientation");

    start_mission(&mission, 0.0f, 47.0f);
    (void) chassis_track_mission_update(&mission,
        100.0f, 0.0f, 0.0f, 47.0f,
        120U, false, &output);
    check(output.state == CHASSIS_TRACK_AB &&
        !output.command_stop && !output.finished &&
        !output.distance_gate_met && !output.heading_gate_met,
        "matching the start orientation cannot bypass the lap gates");
}

static void test_alignment_direction_window_and_stop_confirmation(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    enter_alignment(&mission, 360.0f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        !output.command_stop &&
        near_value(output.angular_rad_s, 0.35f, 0.001f),
        "360 degrees starts limited positive biased alignment");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 396.0f,
        180U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.0f,
        200U, false, &output);
    check(mission.alignment_confirm_cycles == 2U &&
        near_value(output.angular_rad_s, 0.0f, 0.001f),
        "396 and 398 degree boundaries are inside the alignment window");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 20.0f, 397.0f,
        220U, false, &output);
    check(mission.alignment_confirm_cycles == 0U,
        "either wheel at 20 mm/s resets alignment stop confirmation");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f,
        240U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 395.99f,
        260U, false, &output);
    check(mission.alignment_confirm_cycles == 0U &&
        output.angular_rad_s > 0.0f,
        "395.99 degrees restarts positive heading correction");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.01f,
        280U, false, &output);
    check(mission.alignment_confirm_cycles == 0U &&
        output.angular_rad_s < 0.0f,
        "398.01 degrees restarts negative heading correction");
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 396.0f,
        300U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.0f,
        320U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f,
        340U, false, &output);
    check(output.finished && output.state == CHASSIS_TRACK_COMPLETE &&
        mission.stop_time_ms == 340U &&
        near_value(output.elapsed_s, 0.240f, 0.0001f),
        "third aligned stopped cycle records the final lap time");
}

static void test_alignment_faults_and_emergency(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    enter_alignment(&mission, 352.0f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        output.angular_rad_s > 0.0f,
        "alignment accepts the inclusive 45-degree start boundary");

    enter_alignment(&mission, 351.9f, &output);
    check(output.state == CHASSIS_TRACK_FAULT_ALIGNMENT &&
        output.command_stop,
        "alignment rejects a start error greater than 45 degrees");

    enter_alignment(&mission, 360.0f, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 360.0f,
        3160U, false, &output);
    check(output.state == CHASSIS_TRACK_FAULT_ALIGNMENT &&
        output.command_stop,
        "alignment timeout locks the motors at 3000 ms");

    enter_alignment(&mission, 360.0f, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 360.0f,
        180U, true, &output);
    check(output.state == CHASSIS_TRACK_FAULT_EMERGENCY &&
        output.command_stop,
        "center-key emergency remains active during alignment");
}

static void test_alignment_time_over_pass_limit(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    start_mission(&mission, 0.0f, 0.0f);
    mission.state = CHASSIS_TRACK_ALIGNING;
    mission.alignment_start_time_ms = 20000U;
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f,
        20120U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f,
        20140U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.0f,
        20160U, false, &output);
    check(output.finished && !output.passed &&
        mission.stop_time_ms - mission.start_time_ms == 20060U &&
        near_value(output.elapsed_s, 20.06f, 0.001f),
        "alignment time is included and laps over 20 seconds fail");
}

static void test_latest_finish_replay(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = TEST_CURVE_CRUISE_MM_S;
    (void) chassis_track_mission_update(&mission,
        5584.0f, 100.0f, 100.0f, 359.23f,
        120U, false, &output);
    (void) chassis_track_mission_update(&mission,
        5591.0f, 100.0f, 100.0f, 360.17f,
        140U, false, &output);
    (void) chassis_track_mission_update(&mission,
        5598.0f, 100.0f, 100.0f, 361.11f,
        160U, false, &output);
    check(output.heading_gate_met && !output.distance_gate_met &&
        !output.command_stop,
        "latest lap heading samples latch before the distance gate");
    (void) chassis_track_mission_update(&mission,
        5742.0f, 300.0f, 300.0f, 378.30f,
        180U, false, &output);
    check(output.state == CHASSIS_TRACK_FINAL_APPROACH,
        "latest lap enters the rear-reference approach at 5742 mm");
    (void) chassis_track_mission_update(&mission,
        5918.9f, 100.0f, 100.0f, 389.8f,
        200U, false, &output);
    check(!output.distance_gate_met && !output.command_stop,
        "5918.9 mm remains below the rear-reference distance gate");
    (void) chassis_track_mission_update(&mission,
        5919.0f, 100.0f, 100.0f, 390.0f,
        220U, false, &output);
    check(output.state == CHASSIS_TRACK_BRAKING &&
        output.distance_gate_met && output.heading_gate_met,
        "5919 mm brakes after the latched heading advances to 390 degrees");
}

static void test_latest_alignment_bias_replay(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    enter_alignment_from_start(&mission, 0.03f, 387.06f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        output.angular_rad_s > 0.0f &&
        near_value(output.heading_error_deg, 9.97f, 0.02f),
        "latest CSV first stop turns right about ten degrees");

    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 359.04f,
        180U, false, &output);
    check(!output.finished &&
        mission.alignment_confirm_cycles == 0U &&
        near_value(output.heading_error_deg, 37.99f, 0.02f) &&
        output.angular_rad_s > 0.0f,
        "old 359-degree finish cannot complete the biased alignment");

    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 396.03f,
        200U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.03f,
        220U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.03f,
        240U, false, &output);
    check(output.finished && output.command_stop &&
        near_value(output.heading_progress_deg, 397.0f, 0.02f),
        "396 through 398 degrees complete after three stopped cycles");
}

static void test_latest_tight_alignment_replay(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    enter_alignment_from_start(&mission, 0.01f, 385.12f, &output);
    check(output.state == CHASSIS_TRACK_ALIGNING &&
        output.angular_rad_s > 0.0f &&
        near_value(output.heading_error_deg, 11.89f, 0.02f),
        "latest CSV first stop still turns right toward 397 degrees");

    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.71f,
        180U, false, &output);
    check(!output.finished &&
        mission.alignment_confirm_cycles == 0U &&
        near_value(output.heading_error_deg, -1.70f, 0.02f) &&
        output.angular_rad_s < 0.0f,
        "latest 398.71-degree overshoot must correct left");

    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 396.01f,
        200U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 398.01f,
        220U, false, &output);
    (void) chassis_track_mission_update(&mission,
        finish_reference, 0.0f, 0.0f, 397.01f,
        240U, false, &output);
    check(output.finished && output.command_stop &&
        near_value(output.heading_progress_deg, 397.0f, 0.02f),
        "latest replay completes only inside the one-degree window");
}

static void test_finish_reference_validation(void)
{
    chassis_track_config_t config = g_chassis_track_default_config;
    chassis_track_mission_t mission = {0};

    config.finish_reference_progress_mm =
        config.approach_distance_mm;
    check(chassis_track_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT,
        "finish reference must leave room for the approach distance");
    config = g_chassis_track_default_config;
    config.finish_alignment_tolerance_deg = 0.0f;
    check(chassis_track_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT,
        "alignment tolerance must be positive");
    config = g_chassis_track_default_config;
    config.finish_alignment_heading_bias_deg = 180.0f;
    check(chassis_track_mission_init(&mission, &config) ==
        ML_STATUS_INVALID_ARGUMENT,
        "alignment heading bias must have an unambiguous wrapped direction");
}

static void test_distance_limited_final_approach(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;
    float brake_distance = finish_reference -
        g_chassis_track_default_config.finish_stop_lead_mm;
    float approach_start = finish_reference -
        g_chassis_track_default_config.approach_distance_mm;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = TEST_CURVE_CRUISE_MM_S;
    (void) chassis_track_mission_update(&mission,
        approach_start - 0.1f, 360.0f, 360.0f,
        340.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_DA,
        "rear finish calibration does not change route stage geometry");
    (void) chassis_track_mission_update(&mission,
        brake_distance - 100.0f, 360.0f, 360.0f,
        340.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_FINAL_APPROACH &&
        output.linear_mm_s < TEST_CURVE_CRUISE_MM_S &&
        output.linear_mm_s > 100.0f,
        "remaining distance begins the smooth final speed limit");
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f,
        349.0f, 140U, false, &output);
    check(output.state == CHASSIS_TRACK_FINISH_CHECK &&
        near_value(output.linear_mm_s, 100.0f, 0.001f),
        "distance gate immediately caps the request at approach speed");
}

static void test_stage_zero_lap_time_budget(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output = {0};
    float distance_mm = 0.0f;
    float measured_speed = 0.0f;
    float heading_deg;
    uint32_t now_ms = 100U;
    uint16_t cycles = 0U;

    start_mission(&mission, 0.0f, 0.0f);
    while (!output.finished && (cycles < 1100U)) {
        now_ms += 20U;
        if (!output.command_stop) {
            distance_mm += output.linear_mm_s * 0.02f;
            measured_speed = output.linear_mm_s;
        } else {
            measured_speed = 0.0f;
        }
        if (mission.state == CHASSIS_TRACK_ALIGNING) {
            heading_deg = 397.0f;
        } else {
            heading_deg = distance_mm >=
                g_chassis_track_default_config.
                    finish_reference_progress_mm ?
                360.0f : 360.0f * distance_mm /
                g_chassis_track_default_config.
                    finish_reference_progress_mm;
        }
        (void) chassis_track_mission_update(&mission,
            distance_mm, measured_speed, measured_speed,
            heading_deg, now_ms, false, &output);
        ++cycles;
    }
    check(output.finished && output.elapsed_s <= TEST_LAP_TIME_LIMIT_S,
        "selected speed stage meets its nominal lap-time budget");
}

static void test_lap_mismatch_and_emergency(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float finish_reference = g_chassis_track_default_config.
        finish_reference_progress_mm;

    start_mission(&mission, 0.0f, 0.0f);
    (void) chassis_track_mission_update(&mission,
        finish_reference + 50.0f, 100.0f, 100.0f,
        349.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_FAULT_LAP_CHECK &&
        output.command_stop,
        "finish reference plus 50 mm without both gates locks the motors");

    start_mission(&mission, 0.0f, 0.0f);
    (void) chassis_track_mission_update(&mission,
        10.0f, 0.0f, 0.0f, 0.0f, 120U, true, &output);
    check(output.state == CHASSIS_TRACK_FAULT_EMERGENCY &&
        output.command_stop,
        "center-key emergency remains latched");
}

int main(void)
{
    test_start_and_route_geometry();
    test_expected_heading_profile();
    test_heading_feedback();
    test_second_curve_fusion_replay();
    test_dual_finish_gate();
    test_finish_heading_window_bounds();
    test_nonzero_start_heading_reference();
    test_alignment_direction_window_and_stop_confirmation();
    test_alignment_faults_and_emergency();
    test_alignment_time_over_pass_limit();
    test_latest_finish_replay();
    test_latest_alignment_bias_replay();
    test_latest_tight_alignment_replay();
    test_finish_reference_validation();
    test_distance_limited_final_approach();
    test_stage_zero_lap_time_budget();
    test_lap_mismatch_and_emergency();
    if (g_failures == 0) {
        puts("chassis track mission tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
