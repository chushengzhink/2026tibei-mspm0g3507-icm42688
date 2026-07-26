#ifndef ROBOT_MISSION_H
#define ROBOT_MISSION_H

#include <stdint.h>

#include "ml_common.h"
#include "motion_control.h"

typedef enum {
    MISSION_STATE_WAIT_START = 0,
    MISSION_STATE_SEEK_A,
    MISSION_STATE_WAIT_VISION,
    MISSION_STATE_TURN_OUTBOUND,
    MISSION_STATE_FOLLOW_OUTBOUND,
    MISSION_STATE_TURN_TANGENT,
    MISSION_STATE_DRAW_CIRCLE,
    MISSION_STATE_TURN_RETURN,
    MISSION_STATE_FOLLOW_RETURN,
    MISSION_STATE_TURN_PARK,
    MISSION_STATE_ENTER_PARK,
    MISSION_STATE_DONE,
    MISSION_STATE_FAULT
} mission_state_t;

typedef enum {
    MISSION_FAULT_NONE = 0,
    MISSION_FAULT_INIT,
    MISSION_FAULT_COMMAND,
    MISSION_FAULT_VISION_TIMEOUT,
    MISSION_FAULT_MISSION_TIMEOUT,
    MISSION_FAULT_EMERGENCY_STOP,
    MISSION_FAULT_MOTION
} mission_fault_t;

ml_status_t robot_mission_init(void);
void robot_mission_poll(void);
mission_state_t robot_mission_get_state(void);
mission_fault_t robot_mission_get_fault(void);

#endif
