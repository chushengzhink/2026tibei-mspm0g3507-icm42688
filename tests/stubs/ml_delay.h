#ifndef TEST_ML_DELAY_H
#define TEST_ML_DELAY_H

#include <stdint.h>

void SysTick_Init(void);
void SysTick_Wait(uint32_t delay);
void delay_ms(uint32_t delay);
void delay_us(uint32_t delay);

#endif
