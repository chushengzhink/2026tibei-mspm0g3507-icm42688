#ifndef CHASSIS_TRACK_MISSION_BUILD
#define CHASSIS_TRACK_MISSION_BUILD (0)
#endif

#if CHASSIS_TRACK_MISSION_BUILD
#include "chassis_track_app.h"
#else
#include "chassis_self_test.h"
#endif
#include "ml_system.h"

int main(void)
{
    if (system_init() != ML_STATUS_OK) {
        while (1) {
        }
    }

    __enable_irq();
#if CHASSIS_TRACK_MISSION_BUILD
    (void) chassis_track_app_init();
#else
    (void) chassis_self_test_init();
#endif
    while (1) {
#if CHASSIS_TRACK_MISSION_BUILD
        chassis_track_app_poll();
#else
        chassis_self_test_poll();
#endif
    }
}
