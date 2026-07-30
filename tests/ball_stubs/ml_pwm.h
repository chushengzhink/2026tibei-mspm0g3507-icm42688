#ifndef TEST_BALL_ML_PWM_H
#define TEST_BALL_ML_PWM_H

#include "ml_board.h"

ml_status_t pwm_init(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t frequency_hz);
ml_status_t pwm_update(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint32_t duty);

#endif
