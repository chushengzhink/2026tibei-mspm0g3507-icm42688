#include "ml_delay.h"

void SysTick_Init(void)
{
    /* Retained for source compatibility. Delays no longer reserve SysTick. */
}

void SysTick_Wait(uint32_t delay)
{
    if (delay != 0U) {
        delay_cycles(delay);
    }
}

void delay_ms(uint32_t delay)
{
    while (delay-- != 0U) {
        delay_cycles(ML_CPU_CLOCK_HZ / 1000U);
    }
}

void delay_us(uint32_t delay)
{
    while (delay-- != 0U) {
        delay_cycles(ML_CPU_CLOCK_HZ / 1000000U);
    }
}
