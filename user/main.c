#ifndef CHASSIS_TRACK_MISSION_BUILD
#define CHASSIS_TRACK_MISSION_BUILD (0)
#endif
#ifndef BALL_BALANCE_BUILD
#define BALL_BALANCE_BUILD (0)
#endif
#ifndef H456_COMBINED_BUILD
#define H456_COMBINED_BUILD (0)
#endif

#if (BALL_BALANCE_BUILD + CHASSIS_TRACK_MISSION_BUILD + \
     H456_COMBINED_BUILD) > 1
#error Application build modes are mutually exclusive
#endif

#if H456_COMBINED_BUILD
#include "h456_app.h"
#elif BALL_BALANCE_BUILD
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
#if H456_COMBINED_BUILD
    (void) h456_app_init();
#elif BALL_BALANCE_BUILD
    (void) ball_balance_app_init();
#elif CHASSIS_TRACK_MISSION_BUILD
    (void) chassis_track_app_init();
#else
    (void) chassis_self_test_init();
#endif
    while (1) {
#if H456_COMBINED_BUILD
        h456_app_poll();
#elif BALL_BALANCE_BUILD
        ball_balance_app_poll();
#elif CHASSIS_TRACK_MISSION_BUILD
        chassis_track_app_poll();
#else
        chassis_self_test_poll();
#endif
    }
}
