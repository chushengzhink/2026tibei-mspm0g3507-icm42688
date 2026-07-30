#include "chassis_track_mission.h"

#include <math.h>
#include <stdio.h>

#define TEST_PI (3.14159265358979323846f)

#ifndef CHASSIS_TRACK_SPEED_STAGE
#define CHASSIS_TRACK_SPEED_STAGE (0U)
#endif

#if CHASSIS_TRACK_SPEED_STAGE == 0U
#define TEST_STRAIGHT_CRUISE_MM_S (360.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == 1U
#define TEST_STRAIGHT_CRUISE_MM_S (380.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == 2U
#define TEST_STRAIGHT_CRUISE_MM_S (400.0f)
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
              straight_cruise_speed_mm_s,
              TEST_STRAIGHT_CRUISE_MM_S, 0.001f) &&
        near_value(g_chassis_track_default_config.
              curve_cruise_speed_mm_s, 360.0f, 0.001f),
        "selected stage changes only straight cruise speed");
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
    mission.commanded_speed_mm_s = 360.0f;
    (void) chassis_track_mission_update(&mission,
        1500.0f + quarter_curve,
        0.0f, 0.0f, 80.0f, 120U, false, &output);
    nominal_omega = output.linear_mm_s / 500.0f;
    check(near_value(output.angular_rad_s - nominal_omega,
              0.35f, 0.001f),
        "lagging fused heading gets limited positive correction");
    (void) chassis_track_mission_update(&mission,
        1500.0f + quarter_curve,
        0.0f, 0.0f, 100.0f, 140U, false, &output);
    nominal_omega = output.linear_mm_s / 500.0f;
    check(near_value(output.angular_rad_s - nominal_omega,
              -0.35f, 0.001f),
        "leading fused heading gets limited negative correction");
}

static void test_dual_finish_gate(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float route = chassis_track_route_length(
        &g_chassis_track_default_config);
    float brake_distance = route - 15.0f;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = 100.0f;
    (void) chassis_track_mission_update(&mission,
        brake_distance - 1.0f, 100.0f, 100.0f,
        350.0f, 120U, false, &output);
    check(!output.command_stop && !output.distance_gate_met &&
        output.heading_gate_met,
        "heading gate alone cannot stop the car");
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f,
        349.0f, 140U, false, &output);
    check(!output.command_stop && output.distance_gate_met &&
        !output.heading_gate_met &&
        near_value(output.linear_mm_s, 100.0f, 0.001f),
        "distance gate alone continues at approach speed");
    (void) chassis_track_mission_update(&mission,
        brake_distance, 100.0f, 100.0f,
        350.0f, 160U, false, &output);
    check(output.state == CHASSIS_TRACK_BRAKING &&
        output.command_stop && output.distance_gate_met &&
        output.heading_gate_met,
        "distance and 350 degree gates together command braking");

    (void) chassis_track_mission_update(&mission,
        route, 0.0f, 0.0f, 351.0f, 180U, false, &output);
    (void) chassis_track_mission_update(&mission,
        route, 0.0f, 30.0f, 351.0f, 200U, false, &output);
    check(!output.finished && mission.stopped_cycles == 0U,
        "either wheel above 20 mm/s resets the stopped-cycle gate");
    (void) chassis_track_mission_update(&mission,
        route, 0.0f, 0.0f, 351.0f, 220U, false, &output);
    (void) chassis_track_mission_update(&mission,
        route, 0.0f, 0.0f, 351.0f, 240U, false, &output);
    (void) chassis_track_mission_update(&mission,
        route, 0.0f, 0.0f, 351.0f, 260U, false, &output);
    check(output.finished && output.state == CHASSIS_TRACK_COMPLETE &&
        mission.stop_time_ms == 260U &&
        near_value(output.elapsed_s, 0.160f, 0.0001f),
        "third stopped cycle records the exact completion time");
}

static void test_distance_limited_final_approach(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float route = chassis_track_route_length(
        &g_chassis_track_default_config);
    float brake_distance = route -
        g_chassis_track_default_config.finish_stop_lead_mm;

    start_mission(&mission, 0.0f, 0.0f);
    mission.commanded_speed_mm_s = 360.0f;
    (void) chassis_track_mission_update(&mission,
        brake_distance - 147.0f, 360.0f, 360.0f,
        340.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_FINAL_APPROACH &&
        output.linear_mm_s < 360.0f && output.linear_mm_s > 350.0f,
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
    float route = chassis_track_route_length(
        &g_chassis_track_default_config);
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
        heading_deg = distance_mm >= route ? 360.0f :
            360.0f * distance_mm / route;
        (void) chassis_track_mission_update(&mission,
            distance_mm, measured_speed, measured_speed,
            heading_deg, now_ms, false, &output);
        ++cycles;
    }
    check(output.finished && output.elapsed_s <= 20.0f,
        "stage zero nominal B6 lap completes within the 20 s budget");
}

static void test_lap_mismatch_and_emergency(void)
{
    chassis_track_mission_t mission = {0};
    chassis_track_output_t output;
    float route = chassis_track_route_length(
        &g_chassis_track_default_config);

    start_mission(&mission, 0.0f, 0.0f);
    (void) chassis_track_mission_update(&mission,
        route + 50.0f, 100.0f, 100.0f,
        349.0f, 120U, false, &output);
    check(output.state == CHASSIS_TRACK_FAULT_LAP_CHECK &&
        output.command_stop,
        "route plus 50 mm without both gates locks the motors");

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
    test_dual_finish_gate();
    test_distance_limited_final_approach();
    test_stage_zero_lap_time_budget();
    test_lap_mismatch_and_emergency();
    if (g_failures == 0) {
        puts("chassis track mission tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
