#ifndef BALL_DEMO_H
#define BALL_DEMO_H

#include "ml_common.h"

ml_status_t ball_demo_init(void);
void ball_demo_process(void);
ml_status_t ball_demo_short_press(void);
ml_status_t ball_demo_long_press(void);

#endif
