#ifndef TEST_Q3_ML_BOARD_H
#define TEST_Q3_ML_BOARD_H

#include "ml_common.h"

typedef struct { uint8_t unused; } UART_Regs;
typedef struct GPTIMER_Regs { uint8_t unused; } GPTIMER_Regs;
typedef enum { DL_TIMER_CC_0_INDEX = 0, DL_TIMER_CC_1_INDEX = 1 }
    DL_TIMER_CC_INDEX;

extern UART_Regs g_test_uart2;
extern GPTIMER_Regs g_test_tima1;
extern GPTIMER_Regs g_test_timg6;

#define UART2 (&g_test_uart2)
#define TIMA1 (&g_test_tima1)
#define TIMG6 (&g_test_timg6)
#define ML_PWM_DUTY_MAX (50000UL)

#endif
