#include "h5_mission.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "h5_config.h"

#define H5_PI (3.14159265358979323846f)

const h5_mission_config_t g_h5_mission_default_config = {
    1500.0f,
    500.0f,
    H5_CRUISE_SPEED_MM_S,
    H5_CRUISE_SPEED_MM_S,
    H5_CURVE_SPEED_MM_S,
    H5_ACCELERATION_MM_S2,
    H5_ACCELERATION_MM_S2,
    100.0f,
    H5_PASS_PROGRESS_MM,
    H5_PASS_GATE_ARM_PROGRESS_MM,
    H5_PASS_MAX_OVERRUN_MM,
    H5_HEADING_GATE_TARGET_DEG,
    H5_HEADING_GATE_TOLERANCE_DEG,
    4.0f,
    0.35f,
    20.0f,
    1200U,
    8000U,
    H5_TIME_LIMIT_MS,
    H5_CONTROL_PERIOD_MS,
    H5_HEADING_GATE_CONFIRM_CYCLES,
    3U
};

static bool h5_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float h5_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float h5_clamp(float value, float limit)
{
    if (value < -limit) {
        return -limit;
    }
    if (value > limit) {
        return limit;
    }
    return value;
}

static float h5_wrap(float angle_rad)
{
    while (angle_rad >= H5_PI) {
        angle_rad -= 2.0f * H5_PI;
    }
    while (angle_rad < -H5_PI) {
        angle_rad += 2.0f * H5_PI;
    }
    return angle_rad;
}

static float h5_ramp(float current, float target, float step)
{
    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

static float h5_route_length(const h5_mission_config_t *config)
{
    return 2.0f * config->straight_length_mm +
        2.0f * H5_PI * config->curve_radius_mm;
}

static bool h5_mode_valid(h5_mode_t mode)
{
    return (mode == H5_MODE_4) || (mode == H5_MODE_5) ||
        (mode == H5_MODE_6);
}

static bool h5_config_valid(const h5_mission_config_t *config)
{
    if (config == 0) {
        return false;
    }
    return h5_finite(config->straight_length_mm) &&
        h5_finite(config->curve_radius_mm) &&
        h5_finite(config->cruise_speed_mm_s) &&
        h5_finite(config->h5_cruise_speed_mm_s) &&
        h5_finite(config->curve_cruise_speed_mm_s) &&
        h5_finite(config->acceleration_mm_s2) &&
        h5_finite(config->lap_acceleration_mm_s2) &&
        h5_finite(config->h5_launch_acceleration_mm_s2) &&
        h5_finite(config->lap_pass_progress_mm) &&
        h5_finite(config->lap_heading_gate_arm_progress_mm) &&
        h5_finite(config->lap_max_overrun_mm) &&
        h5_finite(config->finish_heading_target_deg) &&
        h5_finite(config->finish_heading_tolerance_deg) &&
        h5_finite(config->heading_control_kp) &&
        h5_finite(config->maximum_heading_correction_rad_s) &&
        h5_finite(config->stop_speed_mm_s) &&
        (config->straight_length_mm > 0.0f) &&
        (config->curve_radius_mm > 0.0f) &&
        (config->cruise_speed_mm_s > 0.0f) &&
        (config->h5_cruise_speed_mm_s > 0.0f) &&
        (config->curve_cruise_speed_mm_s > 0.0f) &&
        (config->acceleration_mm_s2 > 0.0f) &&
        (config->lap_acceleration_mm_s2 > 0.0f) &&
        (config->h5_launch_acceleration_mm_s2 > 0.0f) &&
        (config->lap_pass_progress_mm > config->straight_length_mm) &&
        (config->lap_heading_gate_arm_progress_mm >
         config->straight_length_mm) &&
        (config->lap_heading_gate_arm_progress_mm <=
         config->lap_pass_progress_mm) &&
        (config->lap_max_overrun_mm > 0.0f) &&
        (config->finish_heading_target_deg > 0.0f) &&
        (config->finish_heading_tolerance_deg > 0.0f) &&
        (config->maximum_heading_correction_rad_s > 0.0f) &&
        (config->stop_speed_mm_s > 0.0f) &&
        (config->h5_launch_acceleration_ms > 0U) &&
        (config->h5_time_limit_ms > 0U) &&
        (config->lap_time_limit_ms > 0U) &&
        (config->control_period_ms > 0U) &&
        (config->finish_heading_confirm_cycles > 0U) &&
        (config->stopped_cycles_required > 0U);
}

static bool h5_on_curve(const h5_mission_config_t *config,
    float progress_mm)
{
    float half_curve = H5_PI * config->curve_radius_mm;

    return ((progress_mm >= config->straight_length_mm) &&
            (progress_mm < config->straight_length_mm + half_curve)) ||
        ((progress_mm >= 2.0f * config->straight_length_mm +
          half_curve) &&
         (progress_mm < h5_route_length(config)));
}

static float h5_expected_heading_rad(
    const h5_mission_config_t *config, float progress_mm)
{
    float half_curve = H5_PI * config->curve_radius_mm;
    float second_straight_end =
        2.0f * config->straight_length_mm + half_curve;
    float route = h5_route_length(config);

    if (progress_mm <= config->straight_length_mm) {
        return 0.0f;
    }
    if (progress_mm < config->straight_length_mm + half_curve) {
        return (progress_mm - config->straight_length_mm) /
            config->curve_radius_mm;
    }
    if (progress_mm < second_straight_end) {
        return H5_PI;
    }
    if (progress_mm < route) {
        return H5_PI +
            (progress_mm - second_straight_end) /
            config->curve_radius_mm;
    }
    return 2.0f * H5_PI;
}

static uint32_t h5_time_limit_ms(const h5_mission_t *mission)
{
    return mission->mode == H5_MODE_4 ?
        mission->config.h5_time_limit_ms :
        mission->config.lap_time_limit_ms;
}

static float h5_cruise_speed_mm_s(const h5_mission_t *mission)
{
    return mission->mode == H5_MODE_4 ?
        mission->config.h5_cruise_speed_mm_s :
        mission->config.cruise_speed_mm_s;
}

static float h5_acceleration_mm_s2(const h5_mission_t *mission)
{
    return mission->mode == H5_MODE_4 ?
        mission->config.acceleration_mm_s2 :
        mission->config.lap_acceleration_mm_s2;
}

static float h5_running_target_speed(const h5_mission_t *mission)
{
    if ((mission->mode != H5_MODE_4) &&
        h5_on_curve(&mission->config, mission->progress_mm)) {
        return mission->config.curve_cruise_speed_mm_s;
    }
    return h5_cruise_speed_mm_s(mission);
}

static float h5_running_acceleration_mm_s2(
    const h5_mission_t *mission, uint32_t elapsed_ms)
{
    if ((mission->mode == H5_MODE_4) &&
        (elapsed_ms <= mission->config.h5_launch_acceleration_ms)) {
        return mission->config.h5_launch_acceleration_mm_s2;
    }
    return h5_acceleration_mm_s2(mission);
}

static void h5_fill_output(const h5_mission_t *mission,
    uint32_t now_ms, h5_mission_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->mode = mission->mode;
    output->state = mission->state;
    output->progress_mm = mission->progress_mm;
    output->expected_heading_deg = mission->expected_heading_deg;
    output->heading_progress_deg = mission->heading_progress_deg;
    output->heading_error_deg = mission->heading_error_deg;
    output->elapsed_ms = now_ms - mission->start_time_ms;
    output->score_elapsed_ms = mission->score_elapsed_ms;
    output->score_point_passed = mission->score_point_passed;
    output->heading_gate_met = mission->heading_gate_met;
    output->finished = (mission->state == H5_MISSION_COMPLETE) ||
        (mission->state == H5_MISSION_FAULT_LAP_GATE) ||
        (mission->state == H5_MISSION_FAULT_EMERGENCY);
    output->command_stop = output->finished;
    if (!output->finished) {
        output->result = H5_MISSION_RESULT_PENDING;
    } else if ((mission->state == H5_MISSION_FAULT_LAP_GATE) ||
               (mission->state == H5_MISSION_FAULT_EMERGENCY)) {
        output->result = H5_MISSION_RESULT_FAULT;
    } else if (mission->time_limit_failed) {
        output->result = H5_MISSION_RESULT_TIME_LIMIT;
    } else {
        output->result = H5_MISSION_RESULT_PASS;
    }
}

ml_status_t h5_mission_init(h5_mission_t *mission,
    const h5_mission_config_t *config)
{
    if ((mission == 0) || !h5_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    memset(mission, 0, sizeof(*mission));
    mission->config = *config;
    mission->mode = H5_MODE_4;
    mission->state = H5_MISSION_READY;
    mission->initialized = true;
    return ML_STATUS_OK;
}

ml_status_t h5_mission_start(h5_mission_t *mission,
    h5_mode_t mode, float center_distance_mm,
    float fused_heading_deg, uint32_t now_ms)
{
    if ((mission == 0) || !mission->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((mission->state != H5_MISSION_READY) ||
        !h5_mode_valid(mode) || !h5_finite(center_distance_mm) ||
        !h5_finite(fused_heading_deg)) {
        return ML_STATUS_BUSY;
    }
    mission->mode = mode;
    mission->state = H5_MISSION_RUNNING;
    mission->start_distance_mm = center_distance_mm;
    mission->start_heading_deg = fused_heading_deg;
    mission->start_time_ms = now_ms;
    mission->progress_mm = 0.0f;
    mission->commanded_speed_mm_s = 0.0f;
    mission->expected_heading_deg = 0.0f;
    mission->heading_progress_deg = 0.0f;
    mission->heading_error_deg = 0.0f;
    mission->score_elapsed_ms = 0U;
    mission->heading_window_cycles = 0U;
    mission->stopped_cycles = 0U;
    mission->score_point_passed = false;
    mission->time_limit_failed = false;
    mission->heading_gate_met = false;
    mission->a_marker_seen = false;
    return ML_STATUS_OK;
}

ml_status_t h5_mission_update(h5_mission_t *mission,
    float center_distance_mm, float measured_left_mm_s,
    float measured_right_mm_s, float fused_heading_deg,
    uint32_t now_ms, bool emergency_stop,
    h5_mission_output_t *output)
{
    float expected_heading_rad;
    float heading_error_rad;
    float step;
    uint32_t elapsed_ms;
    bool pass_now = false;

    if ((mission == 0) || !mission->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((output == 0) || !h5_finite(center_distance_mm) ||
        !h5_finite(measured_left_mm_s) ||
        !h5_finite(measured_right_mm_s) ||
        !h5_finite(fused_heading_deg)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (mission->state == H5_MISSION_READY) {
        return ML_STATUS_BUSY;
    }

    mission->progress_mm = center_distance_mm - mission->start_distance_mm;
    if (mission->progress_mm < 0.0f) {
        mission->progress_mm = 0.0f;
    }
    elapsed_ms = now_ms - mission->start_time_ms;
    mission->heading_progress_deg =
        fused_heading_deg - mission->start_heading_deg;
    expected_heading_rad = h5_expected_heading_rad(
        &mission->config, mission->progress_mm);
    mission->expected_heading_deg =
        expected_heading_rad * 180.0f / H5_PI;
    heading_error_rad = h5_wrap(expected_heading_rad -
        (mission->heading_progress_deg * H5_PI / 180.0f));
    mission->heading_error_deg =
        heading_error_rad * 180.0f / H5_PI;

    if (emergency_stop &&
        ((mission->state == H5_MISSION_RUNNING) ||
         (mission->state == H5_MISSION_BRAKING))) {
        mission->state = H5_MISSION_FAULT_EMERGENCY;
        mission->commanded_speed_mm_s = 0.0f;
    }

    if (mission->state == H5_MISSION_RUNNING) {
        if (mission->mode == H5_MODE_4) {
            pass_now = mission->progress_mm >=
                mission->config.straight_length_mm;
        } else {
            if (!mission->heading_gate_met) {
                if ((mission->progress_mm >=
                     mission->config.lap_heading_gate_arm_progress_mm) &&
                    (h5_abs(mission->heading_progress_deg -
                     mission->config.finish_heading_target_deg) <=
                     mission->config.finish_heading_tolerance_deg)) {
                    if (mission->heading_window_cycles < UINT8_MAX) {
                        ++mission->heading_window_cycles;
                    }
                } else {
                    mission->heading_window_cycles = 0U;
                }
                mission->heading_gate_met =
                    mission->heading_window_cycles >=
                    mission->config.finish_heading_confirm_cycles;
            }
            pass_now = mission->heading_gate_met &&
                ((mission->progress_mm >=
                  mission->config.lap_pass_progress_mm) ||
                 (mission->a_marker_seen &&
                  (mission->progress_mm >=
                   mission->config.lap_heading_gate_arm_progress_mm)));
            if (!pass_now &&
                (mission->progress_mm >=
                 mission->config.lap_pass_progress_mm +
                 mission->config.lap_max_overrun_mm)) {
                mission->state = H5_MISSION_FAULT_LAP_GATE;
                mission->commanded_speed_mm_s = 0.0f;
            }
        }
        if (pass_now) {
            mission->score_point_passed = true;
            mission->score_elapsed_ms = elapsed_ms;
            mission->time_limit_failed =
                elapsed_ms > h5_time_limit_ms(mission);
            mission->state = H5_MISSION_BRAKING;
            mission->stopped_cycles = 0U;
        } else if ((mission->state == H5_MISSION_RUNNING) &&
                   (elapsed_ms >= h5_time_limit_ms(mission))) {
            mission->time_limit_failed = true;
            mission->state = H5_MISSION_BRAKING;
            mission->stopped_cycles = 0U;
        }
    }

    if (mission->state == H5_MISSION_RUNNING) {
        step = h5_running_acceleration_mm_s2(mission, elapsed_ms) *
            ((float) mission->config.control_period_ms / 1000.0f);
        mission->commanded_speed_mm_s = h5_ramp(
            mission->commanded_speed_mm_s,
            h5_running_target_speed(mission), step);
    } else if (mission->state == H5_MISSION_BRAKING) {
        step = h5_acceleration_mm_s2(mission) *
            ((float) mission->config.control_period_ms / 1000.0f);
        mission->commanded_speed_mm_s = h5_ramp(
            mission->commanded_speed_mm_s, 0.0f, step);
        if ((mission->commanded_speed_mm_s == 0.0f) &&
            (h5_abs(measured_left_mm_s) <
             mission->config.stop_speed_mm_s) &&
            (h5_abs(measured_right_mm_s) <
             mission->config.stop_speed_mm_s)) {
            if (mission->stopped_cycles < UINT8_MAX) {
                ++mission->stopped_cycles;
            }
        } else {
            mission->stopped_cycles = 0U;
        }
        if (mission->stopped_cycles >=
            mission->config.stopped_cycles_required) {
            mission->state = H5_MISSION_COMPLETE;
        }
    }

    h5_fill_output(mission, now_ms, output);
    if (mission->state == H5_MISSION_RUNNING) {
        output->linear_mm_s = mission->commanded_speed_mm_s;
        if (h5_on_curve(&mission->config, mission->progress_mm)) {
            output->route_feedforward_rad_s =
                mission->commanded_speed_mm_s /
                mission->config.curve_radius_mm;
        }
        output->heading_feedback_rad_s = h5_clamp(
            mission->config.heading_control_kp * heading_error_rad,
            mission->config.maximum_heading_correction_rad_s);
    } else if (mission->state == H5_MISSION_BRAKING) {
        output->linear_mm_s = mission->commanded_speed_mm_s;
    }
    return ML_STATUS_OK;
}

void h5_mission_mark_a_line(h5_mission_t *mission)
{
    if ((mission != 0) && mission->initialized &&
        (mission->state == H5_MISSION_RUNNING)) {
        mission->a_marker_seen = true;
    }
}

const char *h5_mission_state_text(h5_mission_state_t state)
{
    switch (state) {
        case H5_MISSION_READY: return "H5 READY      ";
        case H5_MISSION_RUNNING: return "H5 RUN        ";
        case H5_MISSION_BRAKING: return "H5 PASS BRAKE ";
        case H5_MISSION_COMPLETE: return "H5 COMPLETE   ";
        case H5_MISSION_FAULT_LAP_GATE: return "H5 LAP FAULT  ";
        case H5_MISSION_FAULT_EMERGENCY: return "H5 EMERGENCY  ";
        default: return "H5 ERROR      ";
    }
}
