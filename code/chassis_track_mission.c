#include "chassis_track_mission.h"

#include <float.h>
#include <math.h>

#define CHASSIS_TRACK_PI (3.14159265358979323846f)

#define CHASSIS_TRACK_SPEED_STAGE_BASELINE (0U)
#define CHASSIS_TRACK_SPEED_STAGE_ONE      (1U)
#define CHASSIS_TRACK_SPEED_STAGE_TWO      (2U)

#ifndef CHASSIS_TRACK_SPEED_STAGE
#define CHASSIS_TRACK_SPEED_STAGE CHASSIS_TRACK_SPEED_STAGE_BASELINE
#endif

#if CHASSIS_TRACK_SPEED_STAGE == CHASSIS_TRACK_SPEED_STAGE_BASELINE
#define CHASSIS_TRACK_STRAIGHT_CRUISE_MM_S (360.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == CHASSIS_TRACK_SPEED_STAGE_ONE
#define CHASSIS_TRACK_STRAIGHT_CRUISE_MM_S (380.0f)
#elif CHASSIS_TRACK_SPEED_STAGE == CHASSIS_TRACK_SPEED_STAGE_TWO
#define CHASSIS_TRACK_STRAIGHT_CRUISE_MM_S (400.0f)
#else
#error "CHASSIS_TRACK_SPEED_STAGE must be 0, 1, or 2"
#endif

#define CHASSIS_TRACK_CURVE_CRUISE_MM_S (360.0f)

const chassis_track_config_t g_chassis_track_default_config = {
    1500.0f,
    500.0f,
    CHASSIS_TRACK_STRAIGHT_CRUISE_MM_S,
    CHASSIS_TRACK_CURVE_CRUISE_MM_S,
    100.0f,
    400.0f,
    190.0f,
    5932.0f,
    50.0f,
    13.0f,
    360.0f,
    5.0f,
    2.0f,
    37.0f,
    45.0f,
    4.0f,
    0.35f,
    20.0f,
    20.0f,
    20.0f,
    20U,
    3000U,
    3U,
    3U,
    3U
};

static bool track_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float track_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float track_clamp(float value, float limit)
{
    if (value < -limit) {
        return -limit;
    }
    if (value > limit) {
        return limit;
    }
    return value;
}

static float track_wrap(float angle_rad)
{
    while (angle_rad >= CHASSIS_TRACK_PI) {
        angle_rad -= 2.0f * CHASSIS_TRACK_PI;
    }
    while (angle_rad < -CHASSIS_TRACK_PI) {
        angle_rad += 2.0f * CHASSIS_TRACK_PI;
    }
    return angle_rad;
}

static float track_ramp(float current, float target, float step)
{
    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

static float track_distance_speed_limit(
    const chassis_track_config_t *config, float progress_mm,
    float brake_distance, float cruise_speed_mm_s)
{
    float remaining_mm = brake_distance - progress_mm;
    float limit_mm_s;

    if (remaining_mm < 0.0f) {
        remaining_mm = 0.0f;
    }
    limit_mm_s = sqrtf(config->approach_speed_mm_s *
        config->approach_speed_mm_s +
        2.0f * config->acceleration_mm_s2 * remaining_mm);
    return limit_mm_s < cruise_speed_mm_s ?
        limit_mm_s : cruise_speed_mm_s;
}

float chassis_track_route_length(const chassis_track_config_t *config)
{
    if (config == 0) {
        return 0.0f;
    }
    return 2.0f * config->straight_length_mm +
        2.0f * CHASSIS_TRACK_PI * config->curve_radius_mm;
}

static bool track_config_valid(const chassis_track_config_t *config)
{
    float route;

    if (config == 0) {
        return false;
    }
    route = chassis_track_route_length(config);
    return track_finite(route) &&
        track_finite(config->straight_cruise_speed_mm_s) &&
        track_finite(config->curve_cruise_speed_mm_s) &&
        track_finite(config->approach_speed_mm_s) &&
        track_finite(config->acceleration_mm_s2) &&
        track_finite(config->approach_distance_mm) &&
        track_finite(config->finish_reference_progress_mm) &&
        track_finite(config->finish_max_overrun_mm) &&
        track_finite(config->finish_stop_lead_mm) &&
        track_finite(config->finish_heading_target_deg) &&
        track_finite(config->finish_heading_tolerance_deg) &&
        track_finite(config->finish_alignment_tolerance_deg) &&
        track_finite(config->finish_alignment_heading_bias_deg) &&
        track_finite(config->finish_alignment_max_start_error_deg) &&
        track_finite(config->heading_control_kp) &&
        track_finite(config->maximum_heading_correction_rad_s) &&
        (config->straight_length_mm > 0.0f) &&
        (config->curve_radius_mm > 0.0f) &&
        (config->straight_cruise_speed_mm_s > 0.0f) &&
        (config->curve_cruise_speed_mm_s > 0.0f) &&
        (config->approach_speed_mm_s > 0.0f) &&
        (config->approach_speed_mm_s <=
         config->straight_cruise_speed_mm_s) &&
        (config->approach_speed_mm_s <=
         config->curve_cruise_speed_mm_s) &&
        (config->acceleration_mm_s2 > 0.0f) &&
        (config->approach_distance_mm > 0.0f) &&
        (config->approach_distance_mm <
         config->finish_reference_progress_mm) &&
        (config->finish_max_overrun_mm > 0.0f) &&
        (config->finish_stop_lead_mm >= 0.0f) &&
        (config->finish_stop_lead_mm < config->approach_distance_mm) &&
        (config->finish_heading_target_deg > 0.0f) &&
        (config->finish_heading_target_deg <= 360.0f) &&
        (config->finish_heading_tolerance_deg > 0.0f) &&
        (config->finish_heading_tolerance_deg <
         config->finish_heading_target_deg) &&
        (config->finish_alignment_tolerance_deg > 0.0f) &&
        (config->finish_alignment_tolerance_deg <=
         config->finish_heading_tolerance_deg) &&
        (track_abs(config->finish_alignment_heading_bias_deg) < 180.0f) &&
        (config->finish_alignment_max_start_error_deg >
         config->finish_alignment_tolerance_deg) &&
        (config->finish_alignment_max_start_error_deg < 180.0f) &&
        (config->heading_control_kp >= 0.0f) &&
        (config->maximum_heading_correction_rad_s > 0.0f) &&
        (config->control_period_ms > 0U) &&
        (config->finish_alignment_timeout_ms > 0U) &&
        (config->finish_heading_confirm_cycles > 0U) &&
        (config->finish_alignment_confirm_cycles > 0U) &&
        (config->stopped_cycles_required > 0U);
}

ml_status_t chassis_track_mission_init(chassis_track_mission_t *mission,
    const chassis_track_config_t *config)
{
    if ((mission == 0) || !track_config_valid(config)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    mission->config = *config;
    mission->state = CHASSIS_TRACK_READY;
    mission->start_distance_mm = 0.0f;
    mission->start_heading_deg = 0.0f;
    mission->progress_mm = 0.0f;
    mission->commanded_speed_mm_s = 0.0f;
    mission->stop_error_mm = 0.0f;
    mission->elapsed_s = 0.0f;
    mission->heading_progress_deg = 0.0f;
    mission->expected_heading_deg = 0.0f;
    mission->route_feedforward_rad_s = 0.0f;
    mission->heading_feedback_rad_s = 0.0f;
    mission->heading_error_deg = 0.0f;
    mission->start_time_ms = 0U;
    mission->stop_time_ms = 0U;
    mission->alignment_start_time_ms = 0U;
    mission->heading_window_cycles = 0U;
    mission->alignment_confirm_cycles = 0U;
    mission->stopped_cycles = 0U;
    mission->distance_gate_met = false;
    mission->heading_gate_met = false;
    mission->initialized = true;
    return ML_STATUS_OK;
}

ml_status_t chassis_track_mission_start(chassis_track_mission_t *mission,
    float center_distance_mm, float fused_heading_deg, uint32_t now_ms)
{
    if ((mission == 0) || !mission->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((mission->state != CHASSIS_TRACK_READY) ||
        !track_finite(center_distance_mm) ||
        !track_finite(fused_heading_deg)) {
        return ML_STATUS_BUSY;
    }
    mission->state = CHASSIS_TRACK_AB;
    mission->start_distance_mm = center_distance_mm;
    mission->start_heading_deg = fused_heading_deg;
    mission->start_time_ms = now_ms;
    mission->progress_mm = 0.0f;
    mission->commanded_speed_mm_s = 0.0f;
    mission->stop_error_mm = 0.0f;
    mission->elapsed_s = 0.0f;
    mission->heading_progress_deg = 0.0f;
    mission->expected_heading_deg = 0.0f;
    mission->route_feedforward_rad_s = 0.0f;
    mission->heading_feedback_rad_s = 0.0f;
    mission->heading_error_deg = 0.0f;
    mission->stop_time_ms = 0U;
    mission->alignment_start_time_ms = 0U;
    mission->heading_window_cycles = 0U;
    mission->alignment_confirm_cycles = 0U;
    mission->stopped_cycles = 0U;
    mission->distance_gate_met = false;
    mission->heading_gate_met = false;
    return ML_STATUS_OK;
}

static chassis_track_state_t track_running_state(
    const chassis_track_config_t *config, float progress_mm)
{
    float half_curve = CHASSIS_TRACK_PI * config->curve_radius_mm;
    float brake_distance = config->finish_reference_progress_mm -
        config->finish_stop_lead_mm;

    if (progress_mm >= brake_distance) {
        return CHASSIS_TRACK_FINISH_CHECK;
    }
    if (progress_mm >= config->finish_reference_progress_mm -
        config->approach_distance_mm) {
        return CHASSIS_TRACK_FINAL_APPROACH;
    }
    if (progress_mm < config->straight_length_mm) {
        return CHASSIS_TRACK_AB;
    }
    if (progress_mm < config->straight_length_mm + half_curve) {
        return CHASSIS_TRACK_BC;
    }
    if (progress_mm < 2.0f * config->straight_length_mm + half_curve) {
        return CHASSIS_TRACK_CD;
    }
    return CHASSIS_TRACK_DA;
}

static bool track_state_running(chassis_track_state_t state)
{
    return (state >= CHASSIS_TRACK_AB) &&
        (state <= CHASSIS_TRACK_FINISH_CHECK);
}

static bool track_state_active(chassis_track_state_t state)
{
    return track_state_running(state) ||
        (state == CHASSIS_TRACK_BRAKING) ||
        (state == CHASSIS_TRACK_ALIGNING);
}

static bool track_state_commands_motion(chassis_track_state_t state)
{
    return track_state_running(state) ||
        (state == CHASSIS_TRACK_ALIGNING);
}

static bool track_progress_on_curve(
    const chassis_track_config_t *config, float progress_mm)
{
    float half_curve = CHASSIS_TRACK_PI * config->curve_radius_mm;

    return ((progress_mm >= config->straight_length_mm) &&
            (progress_mm < config->straight_length_mm + half_curve)) ||
        (progress_mm >= 2.0f * config->straight_length_mm + half_curve);
}

static float track_expected_heading_rad(
    const chassis_track_config_t *config, float progress_mm)
{
    float half_curve = CHASSIS_TRACK_PI * config->curve_radius_mm;
    float second_straight_end =
        2.0f * config->straight_length_mm + half_curve;
    float route = chassis_track_route_length(config);

    if (progress_mm <= config->straight_length_mm) {
        return 0.0f;
    }
    if (progress_mm < config->straight_length_mm + half_curve) {
        return (progress_mm - config->straight_length_mm) /
            config->curve_radius_mm;
    }
    if (progress_mm < second_straight_end) {
        return CHASSIS_TRACK_PI;
    }
    if (progress_mm < route) {
        return CHASSIS_TRACK_PI +
            (progress_mm - second_straight_end) /
            config->curve_radius_mm;
    }
    return 2.0f * CHASSIS_TRACK_PI;
}

static void track_fill_output(const chassis_track_mission_t *mission,
    chassis_track_output_t *output)
{
    output->progress_mm = mission->progress_mm;
    output->elapsed_s = mission->elapsed_s;
    output->stop_error_mm = mission->stop_error_mm;
    output->heading_progress_deg = mission->heading_progress_deg;
    output->expected_heading_deg = mission->expected_heading_deg;
    output->route_feedforward_rad_s =
        mission->route_feedforward_rad_s;
    output->heading_feedback_rad_s =
        mission->heading_feedback_rad_s;
    output->heading_error_deg = mission->heading_error_deg;
    output->state = mission->state;
    output->finished = mission->state == CHASSIS_TRACK_COMPLETE;
    output->command_stop = !track_state_commands_motion(mission->state);
    output->passed = output->finished &&
        (mission->elapsed_s <= mission->config.pass_time_s) &&
        (track_abs(mission->stop_error_mm) <=
         mission->config.pass_error_mm);
    output->distance_gate_met = mission->distance_gate_met;
    output->heading_gate_met = mission->heading_gate_met;
}

ml_status_t chassis_track_mission_update(chassis_track_mission_t *mission,
    float center_distance_mm, float measured_left_mm_s,
    float measured_right_mm_s, float fused_heading_deg,
    uint32_t now_ms, bool emergency_stop,
    chassis_track_output_t *output)
{
    float dt_s;
    float target_speed;
    float expected_heading_rad;
    float actual_heading_rad;
    float heading_error_rad;
    float omega_ff = 0.0f;
    float omega_heading;
    float brake_distance;
    float heading_arm_progress;
    float alignment_error_rad;

    if ((mission == 0) || !mission->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((output == 0) || !track_finite(center_distance_mm) ||
        !track_finite(measured_left_mm_s) ||
        !track_finite(measured_right_mm_s) ||
        !track_finite(fused_heading_deg)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    output->linear_mm_s = 0.0f;
    output->angular_rad_s = 0.0f;
    brake_distance = mission->config.finish_reference_progress_mm -
        mission->config.finish_stop_lead_mm;
    heading_arm_progress = 2.0f * mission->config.straight_length_mm +
        CHASSIS_TRACK_PI * mission->config.curve_radius_mm;
    mission->progress_mm = center_distance_mm - mission->start_distance_mm;
    if (mission->progress_mm < 0.0f) {
        mission->progress_mm = 0.0f;
    }
    mission->elapsed_s = (float) ((uint32_t) (now_ms -
        mission->start_time_ms)) / 1000.0f;
    mission->heading_progress_deg =
        fused_heading_deg - mission->start_heading_deg;
    expected_heading_rad = track_expected_heading_rad(
        &mission->config, mission->progress_mm);
    mission->expected_heading_deg =
        expected_heading_rad * 180.0f / CHASSIS_TRACK_PI;
    actual_heading_rad = mission->heading_progress_deg *
        CHASSIS_TRACK_PI / 180.0f;
    heading_error_rad = track_wrap(
        expected_heading_rad - actual_heading_rad);
    mission->heading_error_deg = heading_error_rad *
        180.0f / CHASSIS_TRACK_PI;
    mission->route_feedforward_rad_s = 0.0f;
    mission->heading_feedback_rad_s = 0.0f;

    if (emergency_stop && track_state_active(mission->state)) {
        mission->state = CHASSIS_TRACK_FAULT_EMERGENCY;
    }
    if (track_state_running(mission->state)) {
        mission->distance_gate_met =
            mission->progress_mm >= brake_distance;
        if (!mission->heading_gate_met) {
            if ((mission->progress_mm >= heading_arm_progress) &&
                (track_abs(mission->heading_progress_deg -
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
        if (mission->distance_gate_met && mission->heading_gate_met) {
            mission->state = CHASSIS_TRACK_BRAKING;
            mission->commanded_speed_mm_s = 0.0f;
        } else if (mission->progress_mm >=
                   mission->config.finish_reference_progress_mm +
                   mission->config.finish_max_overrun_mm) {
            mission->state = CHASSIS_TRACK_FAULT_LAP_CHECK;
            mission->commanded_speed_mm_s = 0.0f;
        }
    }

    if (track_state_running(mission->state)) {
        mission->state = track_running_state(&mission->config,
            mission->progress_mm);
        if (track_progress_on_curve(&mission->config,
            mission->progress_mm)) {
            target_speed = mission->config.curve_cruise_speed_mm_s;
        } else {
            target_speed = mission->config.straight_cruise_speed_mm_s;
        }
        target_speed = track_distance_speed_limit(&mission->config,
            mission->progress_mm, brake_distance, target_speed);
        dt_s = (float) mission->config.control_period_ms / 1000.0f;
        if (mission->commanded_speed_mm_s > target_speed) {
            mission->commanded_speed_mm_s = target_speed;
        } else {
            mission->commanded_speed_mm_s = track_ramp(
                mission->commanded_speed_mm_s, target_speed,
                mission->config.acceleration_mm_s2 * dt_s);
        }
        if (track_progress_on_curve(&mission->config,
            mission->progress_mm)) {
            omega_ff = mission->commanded_speed_mm_s /
                mission->config.curve_radius_mm;
        }
        omega_heading = track_clamp(
            mission->config.heading_control_kp * heading_error_rad,
            mission->config.maximum_heading_correction_rad_s);
        mission->route_feedforward_rad_s = omega_ff;
        mission->heading_feedback_rad_s = omega_heading;
        output->linear_mm_s = mission->commanded_speed_mm_s;
        output->angular_rad_s = omega_ff + omega_heading;
    } else if (mission->state == CHASSIS_TRACK_BRAKING) {
        if ((track_abs(measured_left_mm_s) <
             mission->config.stop_speed_mm_s) &&
            (track_abs(measured_right_mm_s) <
             mission->config.stop_speed_mm_s)) {
            if (mission->stopped_cycles < UINT8_MAX) {
                ++mission->stopped_cycles;
            }
        } else {
            mission->stopped_cycles = 0U;
        }
        if (mission->stopped_cycles >=
            mission->config.stopped_cycles_required) {
            alignment_error_rad = track_wrap(
                (mission->start_heading_deg +
                 mission->config.finish_alignment_heading_bias_deg -
                 fused_heading_deg) *
                    CHASSIS_TRACK_PI / 180.0f);
            mission->heading_error_deg = alignment_error_rad *
                180.0f / CHASSIS_TRACK_PI;
            mission->stopped_cycles = 0U;
            mission->alignment_confirm_cycles = 0U;
            if (track_abs(mission->heading_error_deg) >
                mission->config.finish_alignment_max_start_error_deg +
                    0.0001f) {
                mission->state = CHASSIS_TRACK_FAULT_ALIGNMENT;
            } else {
                mission->state = CHASSIS_TRACK_ALIGNING;
                mission->alignment_start_time_ms = now_ms;
            }
        }
    }
    if (mission->state == CHASSIS_TRACK_ALIGNING) {
        mission->expected_heading_deg =
            mission->config.finish_heading_target_deg;
        alignment_error_rad = track_wrap(
            (mission->start_heading_deg +
             mission->config.finish_alignment_heading_bias_deg -
             fused_heading_deg) *
                CHASSIS_TRACK_PI / 180.0f);
        mission->heading_error_deg = alignment_error_rad *
            180.0f / CHASSIS_TRACK_PI;
        mission->route_feedforward_rad_s = 0.0f;
        mission->heading_feedback_rad_s = 0.0f;
        output->linear_mm_s = 0.0f;
        output->angular_rad_s = 0.0f;

        if ((uint32_t) (now_ms - mission->alignment_start_time_ms) >=
            mission->config.finish_alignment_timeout_ms) {
            mission->state = CHASSIS_TRACK_FAULT_ALIGNMENT;
        } else if (track_abs(mission->heading_error_deg) <=
                   mission->config.finish_alignment_tolerance_deg +
                       0.0001f) {
            if ((track_abs(measured_left_mm_s) <
                 mission->config.stop_speed_mm_s) &&
                (track_abs(measured_right_mm_s) <
                 mission->config.stop_speed_mm_s)) {
                if (mission->alignment_confirm_cycles < UINT8_MAX) {
                    ++mission->alignment_confirm_cycles;
                }
            } else {
                mission->alignment_confirm_cycles = 0U;
            }
            if (mission->alignment_confirm_cycles >=
                mission->config.finish_alignment_confirm_cycles) {
                mission->state = CHASSIS_TRACK_COMPLETE;
                mission->stop_time_ms = now_ms;
                mission->elapsed_s = (float) ((uint32_t) (now_ms -
                    mission->start_time_ms)) / 1000.0f;
                mission->stop_error_mm = mission->progress_mm -
                    mission->config.finish_reference_progress_mm;
            }
        } else {
            mission->alignment_confirm_cycles = 0U;
            omega_heading = track_clamp(
                mission->config.heading_control_kp * alignment_error_rad,
                mission->config.maximum_heading_correction_rad_s);
            mission->heading_feedback_rad_s = omega_heading;
            output->angular_rad_s = omega_heading;
        }
    }
    track_fill_output(mission, output);
    return ML_STATUS_OK;
}

const char *chassis_track_state_text(chassis_track_state_t state)
{
    switch (state) {
        case CHASSIS_TRACK_READY: return "RACE FUSION     ";
        case CHASSIS_TRACK_AB: return "RUN AB STRAIGHT ";
        case CHASSIS_TRACK_BC: return "RUN BC CW CURVE ";
        case CHASSIS_TRACK_CD: return "RUN CD STRAIGHT ";
        case CHASSIS_TRACK_DA: return "RUN DA CW CURVE ";
        case CHASSIS_TRACK_FINAL_APPROACH: return "FINAL APPROACH  ";
        case CHASSIS_TRACK_FINISH_CHECK: return "ENC+IMU CHECK   ";
        case CHASSIS_TRACK_BRAKING: return "BRAKING         ";
        case CHASSIS_TRACK_ALIGNING: return "ALIGN TO START  ";
        case CHASSIS_TRACK_COMPLETE: return "LAP COMPLETE    ";
        case CHASSIS_TRACK_FAULT_LAP_CHECK: return "FAULT LAP CHECK ";
        case CHASSIS_TRACK_FAULT_ALIGNMENT: return "FAULT ALIGN     ";
        case CHASSIS_TRACK_FAULT_EMERGENCY: return "EMERGENCY STOP  ";
        default: return "TRACK ERROR     ";
    }
}
