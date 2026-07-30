#ifndef CHASSIS_TRACK_MISSION_BUILD
#define CHASSIS_TRACK_MISSION_BUILD (0)
#endif
#ifndef BALL_BALANCE_BUILD
#define BALL_BALANCE_BUILD (0)
#endif

#if BALL_BALANCE_BUILD && CHASSIS_TRACK_MISSION_BUILD
#error BALL_BALANCE_BUILD and CHASSIS_TRACK_MISSION_BUILD are exclusive
#endif

#if BALL_BALANCE_BUILD
#include "ball_balance_app.h"
#elif CHASSIS_TRACK_MISSION_BUILD
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
#if BALL_BALANCE_BUILD
    (void) ball_balance_app_init();
#elif CHASSIS_TRACK_MISSION_BUILD
    (void) chassis_track_app_init();
#else
    (void) chassis_self_test_init();
#endif
    while (1) {
#if BALL_BALANCE_BUILD
        ball_balance_app_poll();
#elif CHASSIS_TRACK_MISSION_BUILD
        chassis_track_app_poll();
#else
        chassis_self_test_poll();
#endif
    }
}
