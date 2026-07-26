#include "ml_tim.h"

typedef struct {
    GPTIMER_Regs *timer;
    IRQn_Type irq;
    uint32_t divided_clock_hz;
    tim_callback_t callback;
    void *context;
} tim_state_t;

static tim_state_t g_timers[] = {
    {TIMG0, TIMG0_INT_IRQn, 5000000UL, 0, 0},
    {TIMG6, TIMG6_INT_IRQn, 10000000UL, 0, 0},
    {TIMG7, TIMG7_INT_IRQn, 10000000UL, 0, 0},
    {TIMG8, TIMG8_INT_IRQn, 5000000UL, 0, 0},
    {TIMG12, TIMG12_INT_IRQn, 10000000UL, 0, 0}
};

static tim_state_t *tim_find_state(GPTIMER_Regs *timer)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(g_timers) / sizeof(g_timers[0])); ++i) {
        if (g_timers[i].timer == timer) {
            return &g_timers[i];
        }
    }
    return 0;
}

ml_status_t tim_interrupt_ms_init(
    GPTIMER_Regs *timer, uint32_t time_ms, uint8_t priority)
{
    return tim_interrupt_ms_init_ex(timer, time_ms, priority, 0, 0);
}

ml_status_t tim_interrupt_ms_init_ex(GPTIMER_Regs *timer, uint32_t time_ms,
    uint8_t priority, tim_callback_t callback, void *context)
{
    tim_state_t *state = tim_find_state(timer);
    uint64_t total_counts;
    uint32_t divider;
    uint32_t period;
    DL_TimerG_ClockConfig clock_config;
    DL_TimerG_TimerConfig timer_config;

    if ((state == 0) || (time_ms == 0U) ||
        (priority > ML_NVIC_PRIORITY_MAX)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    total_counts = ((uint64_t) state->divided_clock_hz * time_ms) / 1000U;
    if ((total_counts == 0U) ||
        (total_counts > ((uint64_t) 256U * 65536U))) {
        return ML_STATUS_UNSUPPORTED;
    }

    divider = (uint32_t) ((total_counts + 65535U) / 65536U);
    period = (uint32_t) (total_counts / divider);
    if (period == 0U) {
        return ML_STATUS_UNSUPPORTED;
    }

    clock_config.clockSel = DL_TIMER_CLOCK_BUSCLK;
    clock_config.divideRatio = DL_TIMER_CLOCK_DIVIDE_8;
    clock_config.prescale = divider - 1U;

    timer_config.period = period - 1U;
    timer_config.timerMode = DL_TIMER_TIMER_MODE_PERIODIC_UP;
    timer_config.startTimer = DL_TIMER_START;

    DL_TimerG_reset(timer);
    DL_TimerG_enablePower(timer);
    DL_TimerG_setClockConfig(timer, &clock_config);
    DL_TimerG_initTimerMode(timer, &timer_config);
    DL_TimerG_enableInterrupt(timer, DL_TIMERG_INTERRUPT_LOAD_EVENT);

    state->callback = callback;
    state->context = context;

    NVIC_ClearPendingIRQ(state->irq);
    NVIC_SetPriority(state->irq, priority);
    NVIC_EnableIRQ(state->irq);
    DL_TimerG_enableClock(timer);

    return ML_STATUS_OK;
}

void tim_irq_dispatch(GPTIMER_Regs *timer)
{
    tim_state_t *state = tim_find_state(timer);

    if ((state != 0) &&
        (DL_TimerG_getPendingInterrupt(timer) == DL_TIMER_IIDX_LOAD) &&
        (state->callback != 0)) {
        state->callback(state->context);
    }
}
