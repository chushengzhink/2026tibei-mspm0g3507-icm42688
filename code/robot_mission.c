#include "robot_mission.h"

#include "line_sensor.h"
#include "mission_sequence.h"
#include "mission_view.h"
#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "robot_config.h"
#include "robot_input.h"

#define MISSION_DISPLAY_PERIOD_TICKS (50U)

typedef struct {
    mission_sequence_t sequence;
    uint32_t last_processed_tick;
    uint32_t last_display_tick;
} robot_mission_context_t;

static robot_mission_context_t g_robot_mission;

static ml_status_t robot_mission_execute_command(
    const mission_motion_command_t *command)
{
    switch (command->type) {
        case MISSION_MOTION_COMMAND_NONE:
            return ML_STATUS_OK;
        case MISSION_MOTION_COMMAND_FIND_A:
            return motion_start_find_a(command->speed_mm_s);
        case MISSION_MOTION_COMMAND_STRAIGHT:
            return motion_start_straight(
                command->distance_mm, command->speed_mm_s);
        case MISSION_MOTION_COMMAND_LINE:
            return motion_start_line(
                command->distance_mm, command->speed_mm_s);
        case MISSION_MOTION_COMMAND_LINE_TO_END:
            return motion_start_line_to_end(
                command->distance_mm, command->speed_mm_s);
        case MISSION_MOTION_COMMAND_TURN:
            return motion_start_turn90(
                command->turn_right, command->align_to_line);
        case MISSION_MOTION_COMMAND_CIRCLE:
            return motion_start_circle(command->radius_mm);
        default:
            return ML_STATUS_INVALID_ARGUMENT;
    }
}

static void robot_mission_apply_output(
    mission_sequence_output_t *output)
{
    ml_status_t status;

    if (output->reset_vision) {
        robot_input_reset_vision();
    }
    if (output->stop_motion) {
        motion_emergency_stop();
    }
    if (output->led_off) {
        board_led_off();
    }
    if (output->led_on) {
        board_led_on();
    }
    status = robot_mission_execute_command(&output->motion_command);
    if (status != ML_STATUS_OK) {
        mission_sequence_command_failed(
            &g_robot_mission.sequence, output);
        motion_emergency_stop();
        board_led_on();
    }
}

static void robot_mission_render(void)
{
    motion_status_t motion = motion_get_status();

    mission_view_render(g_robot_mission.sequence.state,
        g_robot_mission.sequence.fault,
        g_robot_mission.sequence.motion_fault,
        g_robot_mission.sequence.radius_cm,
        line_sensor_white_levels(), &motion);
}

ml_status_t robot_mission_init(void)
{
    uint32_t now;
    ml_status_t status;

    status = board_led_init();
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_init();
    }
    if (status == ML_STATUS_OK) {
        status = ml_encoder_init();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_init();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_calibrate_white(64U, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = robot_input_init();
    }
    if (status == ML_STATUS_OK) {
        status = mission_view_init();
    }
    if (status == ML_STATUS_OK) {
        status = motion_init(&g_robot_calibration);
    }

    now = motion_get_uptime_ticks();
    mission_sequence_init(&g_robot_mission.sequence, now);
    g_robot_mission.last_processed_tick = now;
    g_robot_mission.last_display_tick = now;
    if (status != ML_STATUS_OK) {
        g_robot_mission.sequence.fault = MISSION_FAULT_INIT;
        return status;
    }
    robot_mission_render();
    return ML_STATUS_OK;
}

void robot_mission_poll(void)
{
    mission_sequence_input_t input;
    mission_sequence_output_t output;
    uint32_t now = motion_get_uptime_ticks();
    motion_status_t motion;

    robot_input_poll(
        (uint32_t) (now - g_robot_mission.last_processed_tick));
    g_robot_mission.last_processed_tick = now;
    motion = motion_get_status();
    input.now_tick = now;
    input.motion = motion;
    input.radius_cm = 0U;
    input.radius_available =
        robot_input_try_get_radius(&input.radius_cm);
    input.hmi_start_requested = robot_input_take_hmi_start();
    input.center_pressed = robot_input_take_center_press();

    mission_sequence_step(
        &g_robot_mission.sequence, &input, &output);
    robot_mission_apply_output(&output);
    if (output.state_changed) {
        robot_mission_render();
    }
    if ((uint32_t) (now - g_robot_mission.last_display_tick) >=
        MISSION_DISPLAY_PERIOD_TICKS) {
        g_robot_mission.last_display_tick = now;
        mission_view_update_line(
            line_sensor_white_levels(), &motion);
    }
}

mission_state_t robot_mission_get_state(void)
{
    return g_robot_mission.sequence.state;
}

mission_fault_t robot_mission_get_fault(void)
{
    return g_robot_mission.sequence.fault;
}
