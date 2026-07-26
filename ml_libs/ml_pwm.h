#ifndef ML_PWM_H
#define ML_PWM_H

#include "ml_board.h"

#define MAX_DUTY ML_PWM_DUTY_MAX

ml_status_t pwm_init(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t frequency_hz);
ml_status_t pwm_update(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint32_t duty);
ml_status_t pwm_pin_init(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint16_t frequency_hz, uint8_t *prescale, uint32_t *period);

#endif
