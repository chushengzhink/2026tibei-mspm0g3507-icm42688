#include "mission_sequence.h"

#include <stdio.h>

static int g_failures;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static mission_sequence_input_t base_input(uint32_t now)
{
    mission_sequence_input_t input;

    input.now_tick = now;
    input.motion.mode = MOTION_MODE_IDLE;
    input.motion.result = MOTION_RESULT_IDLE;
    input.motion.fault = MOTION_FAULT_NONE;
    input.motion.left_distance_mm = 0.0f;
    input.motion.right_distance_mm = 0.0f;
    input.motion.mean_distance_mm = 0.0f;
    input.motion.target_distance_mm = 0.0f;
    input.motion.line.raw_bits = 0U;
    input.motion.line.black_bits = 0U;
    input.motion.line.black_count = 0U;
    input.motion.line.error_mm = 0.0f;
    input.motion.line.lost = false;
    input.motion.line.transverse = false;
    input.motion.line.centered = true;
    input.motion.uptime_ticks = now;
    input.radius_cm = 0U;
    input.radius_available = false;
    input.hmi_start_requested = false;
    input.center_pressed = false;
    return input;
}

static void complete_step(mission_sequence_t *sequence,
    mission_sequence_input_t *input, mission_sequence_output_t *output)
{
    ++input->now_tick;
    input->motion.result = MOTION_RESULT_COMPLETE;
    mission_sequence_step(sequence, input, output);
}

static void test_complete_sequence(void)
{
    mission_sequence_t sequence;
    mission_sequence_input_t input = base_input(0U);
    mission_sequence_output_t output;

    mission_sequence_init(&sequence, 0U);
    input.hmi_start_requested = true;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_SEEK_A &&
        output.motion_command.type == MISSION_MOTION_COMMAND_FIND_A &&
        output.reset_vision && output.led_off,
        "start event begins A search and resets mission inputs");

    input.hmi_start_requested = false;
    input.radius_available = true;
    input.radius_cm = 40U;
    input.motion.mean_distance_mm = 123.0f;
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_TURN_OUTBOUND &&
        output.motion_command.type == MISSION_MOTION_COMMAND_TURN &&
        output.motion_command.turn_right &&
        output.motion_command.align_to_line,
        "A completion starts aligned outbound turn");

    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_FOLLOW_OUTBOUND &&
        output.motion_command.type == MISSION_MOTION_COMMAND_LINE &&
        output.motion_command.distance_mm == 400.0f,
        "outbound line distance derives from detected radius");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_TURN_TANGENT &&
        output.motion_command.type == MISSION_MOTION_COMMAND_TURN,
        "outbound line completion starts tangent turn");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_DRAW_CIRCLE &&
        output.motion_command.type == MISSION_MOTION_COMMAND_CIRCLE &&
        output.motion_command.radius_mm == 400U,
        "tangent completion starts the requested circle");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_TURN_RETURN,
        "circle completion starts return turn");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_FOLLOW_RETURN &&
        output.motion_command.type ==
            MISSION_MOTION_COMMAND_LINE_TO_END,
        "return turn starts endpoint line tracking");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_TURN_PARK &&
        !output.motion_command.turn_right,
        "return line completion starts left parking turn");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_ENTER_PARK &&
        output.motion_command.type == MISSION_MOTION_COMMAND_STRAIGHT &&
        output.motion_command.distance_mm == 123.0f,
        "parking command reuses measured park-to-A distance");
    complete_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_DONE && output.led_on,
        "final straight completion ends the mission");
}

static void test_faults_and_reset(void)
{
    mission_sequence_t sequence;
    mission_sequence_input_t input = base_input(0U);
    mission_sequence_output_t output;

    mission_sequence_init(&sequence, 0U);
    input.center_pressed = true;
    mission_sequence_step(&sequence, &input, &output);
    input.center_pressed = false;
    input.motion.result = MOTION_RESULT_COMPLETE;
    input.now_tick = 1U;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_WAIT_VISION,
        "mission waits when radius is not ready at A");
    input.now_tick = 1U + ROBOT_VISION_TIMEOUT_TICKS;
    input.motion.result = MOTION_RESULT_IDLE;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.fault == MISSION_FAULT_VISION_TIMEOUT &&
        output.stop_motion && output.led_on,
        "vision wait timeout enters a safe fault");
    input.center_pressed = true;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.state == MISSION_STATE_WAIT_START &&
        sequence.fault == MISSION_FAULT_NONE && output.reset_vision,
        "center press resets a terminal fault");

    input = base_input(10U);
    input.hmi_start_requested = true;
    mission_sequence_step(&sequence, &input, &output);
    input.hmi_start_requested = false;
    input.center_pressed = true;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.fault == MISSION_FAULT_EMERGENCY_STOP,
        "center press while running triggers emergency stop");

    mission_sequence_init(&sequence, 0U);
    input = base_input(0U);
    input.hmi_start_requested = true;
    mission_sequence_step(&sequence, &input, &output);
    input.hmi_start_requested = false;
    input.motion.result = MOTION_RESULT_FAULT;
    input.motion.fault = MOTION_FAULT_ENCODER;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.fault == MISSION_FAULT_MOTION &&
        sequence.motion_fault == MOTION_FAULT_ENCODER,
        "motion faults propagate into mission diagnostics");

    mission_sequence_init(&sequence, 0U);
    mission_sequence_command_failed(&sequence, &output);
    check(sequence.fault == MISSION_FAULT_COMMAND && output.stop_motion,
        "command execution failure enters a safe fault");

    mission_sequence_init(&sequence, 0U);
    input = base_input(0U);
    input.hmi_start_requested = true;
    mission_sequence_step(&sequence, &input, &output);
    input.hmi_start_requested = false;
    input.now_tick = ROBOT_MISSION_TIMEOUT_TICKS;
    mission_sequence_step(&sequence, &input, &output);
    check(sequence.fault == MISSION_FAULT_MISSION_TIMEOUT,
        "whole-mission timeout is enforced");
}

int main(void)
{
    test_complete_sequence();
    test_faults_and_reset();
    if (g_failures == 0) {
        printf("PASS: mission sequence tests\n");
    }
    return g_failures == 0 ? 0 : 1;
}
