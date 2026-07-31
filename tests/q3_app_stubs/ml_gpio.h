#ifndef TEST_Q3_APP_ML_GPIO_H
#define TEST_Q3_APP_ML_GPIO_H

#include "ml_board.h"

typedef enum { OUT = 0, IN_UP, IN_DOWN, IN_FLOAT } GPIO_Mode_enum;
typedef uint32_t GPIOn_enum;

ml_status_t gpio_init(GPIO_Regs *gpio, uint32_t pins,
    GPIOn_enum gpion, GPIO_Mode_enum mode);
uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins);

#endif
