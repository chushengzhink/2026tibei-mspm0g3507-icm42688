#ifndef TEST_ML_TIM_H
#define TEST_ML_TIM_H

#include <stdint.h>

#include "ml_common.h"

typedef struct GPTIMER_Regs GPTIMER_Regs;
typedef void (*tim_callback_t)(void *context);

ml_status_t tim_interrupt_ms_init(
    GPTIMER_Regs *timer, uint32_t time_ms, uint8_t priority);
ml_status_t tim_interrupt_ms_init_ex(GPTIMER_Regs *timer, uint32_t time_ms,
    uint8_t priority, tim_callback_t callback, void *context);
void tim_irq_dispatch(GPTIMER_Regs *timer);

#endif
