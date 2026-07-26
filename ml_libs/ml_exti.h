#ifndef ML_EXTI_H
#define ML_EXTI_H

#include "ml_gpio.h"

typedef enum {
    RISING = 0,
    FALLING
} EXTI_MODE_enum;

/* Values 0-27 encode GPIOA; values 32-59 encode GPIOB. */
typedef enum {
    EXTI_PA0 = 0,
    EXTI_PA1,
    EXTI_PA2,
    EXTI_PA3,
    EXTI_PA4,
    EXTI_PA5,
    EXTI_PA6,
    EXTI_PA7,
    EXTI_PA8,
    EXTI_PA9,
    EXTI_PA10,
    EXTI_PA11,
    EXTI_PA12,
    EXTI_PA13,
    EXTI_PA14,
    EXTI_PA15,
    EXTI_PA16,
    EXTI_PA17,
    EXTI_PA18,
    EXTI_PA19,
    EXTI_PA20,
    EXTI_PA21,
    EXTI_PA22,
    EXTI_PA23,
    EXTI_PA24,
    EXTI_PA25,
    EXTI_PA26,
    EXTI_PA27,
    EXTI_PB0 = 32,
    EXTI_PB1,
    EXTI_PB2,
    EXTI_PB3,
    EXTI_PB4,
    EXTI_PB5,
    EXTI_PB6,
    EXTI_PB7,
    EXTI_PB8,
    EXTI_PB9,
    EXTI_PB10,
    EXTI_PB11,
    EXTI_PB12,
    EXTI_PB13,
    EXTI_PB14,
    EXTI_PB15,
    EXTI_PB16,
    EXTI_PB17,
    EXTI_PB18,
    EXTI_PB19,
    EXTI_PB20,
    EXTI_PB21,
    EXTI_PB22,
    EXTI_PB23,
    EXTI_PB24,
    EXTI_PB25,
    EXTI_PB26,
    EXTI_PB27
} EXTI_GPIO_enum;

typedef void (*exti_callback_t)(uint32_t pin, void *context);

ml_status_t exti_init(
    EXTI_GPIO_enum exti_gpio, EXTI_MODE_enum mode, uint8_t priority);
ml_status_t exti_init_ex(EXTI_GPIO_enum exti_gpio, EXTI_MODE_enum mode,
    uint8_t priority, exti_callback_t callback, void *context);
ml_status_t exti_pin_init(EXTI_GPIO_enum exti_gpio, GPIOn_enum *gpion);
void exti_irq_dispatch(void);

#endif
