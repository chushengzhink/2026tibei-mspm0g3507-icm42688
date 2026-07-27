#ifndef MISSION_SEQUENCE_H
#define MISSION_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_mission.h"

typedef enum {
    MISSION_MOTION_COMMAND_NONE = 0,
    MISSION_MOTION_COMMAND_FIND_A,
    MISSION_MOTION_COMMAND_STRAIGHT,
    MISSION_MOTION_COMMAND_LINE,
    MISSION_MOTION_COMMAND_LINE_TO_END,
    MISSION_MOTION_COMMAND_TURN,
    MISSION_MOTION_COMMAND_CIRCLE
} mission_motion_command_type_t;

typedef struct {
    mission_motion_command_type_t type;
    float distance_mm;
    float speed_mm_s;
    uint16_t radius_mm;
    bool turn_right;
    bool align_to_line;
} mission_motion_command_t;

typedef struct {
    uint32_t now_tick;
    motion_status_t motion;
    uint8_t radius_cm;
    bool radius_available;
    bool hmi_start_requested;
    bool center_pressed;
} mission_sequence_input_t;

typedef struct {
    mission_motion_command_t motion_command;
    bool stop_motion;
    bool reset_vision;
    bool led_on;
    bool led_off;
    bool state_changed;
} mission_sequence_output_t;

typedef struct {
    mission_state_t state;
    mission_fault_t fault;
    motion_fault_t motion_fault;
    uint8_t radius_cm;
    float park_to_a_mm;
    uint32_t state_start_tick;
    uint32_t mission_start_tick;
} mission_sequence_t;

void mission_sequence_init(
    mission_sequence_t *sequence, uint32_t now_tick);
void mission_sequence_step(mission_sequence_t *sequence,
    const mission_sequence_input_t *input,
    mission_sequence_output_t *output);
void mission_sequence_command_failed(mission_sequence_t *sequence,
    mission_sequence_output_t *output);

#endif
