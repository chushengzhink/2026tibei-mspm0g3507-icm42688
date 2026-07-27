#include "mission_sequence.h"

#include "robot_config.h"

static bool mission_sequence_is_running(
    const mission_sequence_t *sequence)
{
    return (sequence->state != MISSION_STATE_WAIT_START) &&
        (sequence->state != MISSION_STATE_DONE) &&
        (sequence->state != MISSION_STATE_FAULT);
}

static void mission_sequence_clear_output(
    mission_sequence_output_t *output)
{
    output->motion_command.type = MISSION_MOTION_COMMAND_NONE;
    output->motion_command.distance_mm = 0.0f;
    output->motion_command.speed_mm_s = 0.0f;
    output->motion_command.radius_mm = 0U;
    output->motion_command.turn_right = false;
    output->motion_command.align_to_line = false;
    output->stop_motion = false;
    output->reset_vision = false;
    output->led_on = false;
    output->led_off = false;
    output->state_changed = false;
}

static void mission_sequence_transition(mission_sequence_t *sequence,
    mission_state_t state, uint32_t now_tick,
    mission_sequence_output_t *output)
{
    sequence->state = state;
    sequence->state_start_tick = now_tick;
    output->state_changed = true;
}

static void mission_sequence_fault(mission_sequence_t *sequence,
    mission_fault_t fault, motion_fault_t motion_fault,
    uint32_t now_tick, mission_sequence_output_t *output)
{
    sequence->fault = fault;
    sequence->motion_fault = motion_fault;
    output->stop_motion = true;
    output->led_on = true;
    mission_sequence_transition(
        sequence, MISSION_STATE_FAULT, now_tick, output);
}

static void mission_sequence_begin(mission_sequence_t *sequence,
    uint32_t now_tick, mission_sequence_output_t *output)
{
    sequence->radius_cm = 0U;
    sequence->park_to_a_mm = 0.0f;
    sequence->fault = MISSION_FAULT_NONE;
    sequence->motion_fault = MOTION_FAULT_NONE;
    sequence->mission_start_tick = now_tick;
    output->reset_vision = true;
    output->led_off = true;
    output->motion_command.type = MISSION_MOTION_COMMAND_FIND_A;
    output->motion_command.speed_mm_s = ROBOT_APPROACH_SPEED_MM_S;
    mission_sequence_transition(
        sequence, MISSION_STATE_SEEK_A, now_tick, output);
}

static void mission_sequence_turn(mission_sequence_output_t *output,
    bool turn_right, bool align_to_line)
{
    output->motion_command.type = MISSION_MOTION_COMMAND_TURN;
    output->motion_command.turn_right = turn_right;
    output->motion_command.align_to_line = align_to_line;
}

void mission_sequence_init(
    mission_sequence_t *sequence, uint32_t now_tick)
{
    if (sequence == 0) {
        return;
    }
    sequence->state = MISSION_STATE_WAIT_START;
    sequence->fault = MISSION_FAULT_NONE;
    sequence->motion_fault = MOTION_FAULT_NONE;
    sequence->radius_cm = 0U;
    sequence->park_to_a_mm = 0.0f;
    sequence->state_start_tick = now_tick;
    sequence->mission_start_tick = now_tick;
}

void mission_sequence_step(mission_sequence_t *sequence,
    const mission_sequence_input_t *input,
    mission_sequence_output_t *output)
{
    if ((sequence == 0) || (input == 0) || (output == 0)) {
        return;
    }
    mission_sequence_clear_output(output);

    if (input->center_pressed) {
        if (sequence->state == MISSION_STATE_WAIT_START) {
            mission_sequence_begin(sequence, input->now_tick, output);
        } else if (mission_sequence_is_running(sequence)) {
            mission_sequence_fault(sequence,
                MISSION_FAULT_EMERGENCY_STOP, MOTION_FAULT_NONE,
                input->now_tick, output);
        } else {
            sequence->fault = MISSION_FAULT_NONE;
            sequence->motion_fault = MOTION_FAULT_NONE;
            sequence->radius_cm = 0U;
            output->reset_vision = true;
            mission_sequence_transition(sequence,
                MISSION_STATE_WAIT_START, input->now_tick, output);
        }
        return;
    }

    if (input->hmi_start_requested &&
        (sequence->state == MISSION_STATE_WAIT_START)) {
        mission_sequence_begin(sequence, input->now_tick, output);
        return;
    }

    if (mission_sequence_is_running(sequence) &&
        ((uint32_t) (input->now_tick - sequence->mission_start_tick) >=
         ROBOT_MISSION_TIMEOUT_TICKS)) {
        mission_sequence_fault(sequence,
            MISSION_FAULT_MISSION_TIMEOUT, MOTION_FAULT_NONE,
            input->now_tick, output);
        return;
    }
    if (input->motion.result == MOTION_RESULT_FAULT) {
        mission_sequence_fault(sequence, MISSION_FAULT_MOTION,
            input->motion.fault, input->now_tick, output);
        return;
    }

    switch (sequence->state) {
        case MISSION_STATE_SEEK_A:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                sequence->park_to_a_mm = input->motion.mean_distance_mm;
                if (input->radius_available) {
                    sequence->radius_cm = input->radius_cm;
                    mission_sequence_turn(output, true, true);
                    mission_sequence_transition(sequence,
                        MISSION_STATE_TURN_OUTBOUND,
                        input->now_tick, output);
                } else {
                    mission_sequence_transition(sequence,
                        MISSION_STATE_WAIT_VISION,
                        input->now_tick, output);
                }
            }
            break;

        case MISSION_STATE_WAIT_VISION:
            if (input->radius_available) {
                sequence->radius_cm = input->radius_cm;
                mission_sequence_turn(output, true, true);
                mission_sequence_transition(sequence,
                    MISSION_STATE_TURN_OUTBOUND,
                    input->now_tick, output);
            } else if ((uint32_t) (input->now_tick -
                sequence->state_start_tick) >=
                ROBOT_VISION_TIMEOUT_TICKS) {
                mission_sequence_fault(sequence,
                    MISSION_FAULT_VISION_TIMEOUT, MOTION_FAULT_NONE,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_TURN_OUTBOUND:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                output->motion_command.type = MISSION_MOTION_COMMAND_LINE;
                output->motion_command.distance_mm =
                    (float) sequence->radius_cm * 10.0f;
                output->motion_command.speed_mm_s = ROBOT_LINE_SPEED_MM_S;
                mission_sequence_transition(sequence,
                    MISSION_STATE_FOLLOW_OUTBOUND,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_FOLLOW_OUTBOUND:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                mission_sequence_turn(output, true, false);
                mission_sequence_transition(sequence,
                    MISSION_STATE_TURN_TANGENT,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_TURN_TANGENT:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                output->motion_command.type = MISSION_MOTION_COMMAND_CIRCLE;
                output->motion_command.radius_mm =
                    (uint16_t) sequence->radius_cm * 10U;
                mission_sequence_transition(sequence,
                    MISSION_STATE_DRAW_CIRCLE,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_DRAW_CIRCLE:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                mission_sequence_turn(output, true, true);
                mission_sequence_transition(sequence,
                    MISSION_STATE_TURN_RETURN,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_TURN_RETURN:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                output->motion_command.type =
                    MISSION_MOTION_COMMAND_LINE_TO_END;
                output->motion_command.distance_mm =
                    (float) sequence->radius_cm * 10.0f;
                output->motion_command.speed_mm_s = ROBOT_LINE_SPEED_MM_S;
                mission_sequence_transition(sequence,
                    MISSION_STATE_FOLLOW_RETURN,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_FOLLOW_RETURN:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                mission_sequence_turn(output, false, false);
                mission_sequence_transition(sequence,
                    MISSION_STATE_TURN_PARK,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_TURN_PARK:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                output->motion_command.type =
                    MISSION_MOTION_COMMAND_STRAIGHT;
                output->motion_command.distance_mm = sequence->park_to_a_mm;
                output->motion_command.speed_mm_s = ROBOT_PARK_SPEED_MM_S;
                mission_sequence_transition(sequence,
                    MISSION_STATE_ENTER_PARK,
                    input->now_tick, output);
            }
            break;

        case MISSION_STATE_ENTER_PARK:
            if (input->motion.result == MOTION_RESULT_COMPLETE) {
                output->led_on = true;
                mission_sequence_transition(sequence,
                    MISSION_STATE_DONE, input->now_tick, output);
            }
            break;

        default:
            break;
    }
}

void mission_sequence_command_failed(mission_sequence_t *sequence,
    mission_sequence_output_t *output)
{
    uint32_t now_tick;

    if ((sequence == 0) || (output == 0)) {
        return;
    }
    now_tick = sequence->state_start_tick;
    mission_sequence_clear_output(output);
    mission_sequence_fault(sequence, MISSION_FAULT_COMMAND,
        MOTION_FAULT_NONE, now_tick, output);
}
