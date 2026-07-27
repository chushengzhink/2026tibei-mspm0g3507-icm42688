#ifndef MISSION_VIEW_H
#define MISSION_VIEW_H

#include <stdint.h>

#include "robot_mission.h"

ml_status_t mission_view_init(void);
void mission_view_render(mission_state_t state,
    mission_fault_t fault, motion_fault_t motion_fault,
    uint8_t radius_cm, uint8_t white_levels,
    const motion_status_t *motion);
void mission_view_update_line(
    uint8_t white_levels, const motion_status_t *motion);

#endif
