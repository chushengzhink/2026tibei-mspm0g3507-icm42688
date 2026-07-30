#ifndef TEST_ML_BOARD_H
#define TEST_ML_BOARD_H

#include "ml_common.h"

typedef struct {
    uint8_t unused;
} UART_Regs;

typedef struct GPTIMER_Regs {
    uint8_t unused;
} GPTIMER_Regs;

typedef struct GPIO_Regs {
    uint8_t identifier;
    uint32_t DIN31_0;
    uint32_t DOUTSET31_0;
    uint32_t DOUTCLR31_0;
    uint32_t DOUTTGL31_0;
} GPIO_Regs;

extern UART_Regs g_test_uart0;
extern GPTIMER_Regs g_test_timg0;
extern GPIO_Regs g_test_gpioa;
extern GPIO_Regs g_test_gpiob;

#define UART0 (&g_test_uart0)
#define TIMG0 (&g_test_timg0)
#define GPIOA (&g_test_gpioa)
#define GPIOB (&g_test_gpiob)
#define DL_GPIO_PIN_31 (UINT32_C(1) << 31)
#define DL_GPIO_PIN_12 (UINT32_C(1) << 12)
#define DL_GPIO_PIN_8  (UINT32_C(1) << 8)
#define DL_GPIO_PIN_10 (UINT32_C(1) << 10)
#define DL_GPIO_PIN_27 (UINT32_C(1) << 27)
#define IOMUX_PINCM60 59U
#define PA27 IOMUX_PINCM60
#define ML_C1_PORT GPIOA
#define ML_C1_PIN DL_GPIO_PIN_31
#define ML_C1_IOMUX 6U
#define ML_C2_PORT GPIOA
#define ML_C2_PIN DL_GPIO_PIN_12
#define ML_C2_IOMUX 34U
#define ML_C3_PORT GPIOB
#define ML_C3_PIN DL_GPIO_PIN_8
#define ML_C3_IOMUX 25U
#define ML_C8_PORT GPIOA
#define ML_C8_PIN DL_GPIO_PIN_27
#define ML_C8_IOMUX PA27
#define ML_PWM_DUTY_MAX     (50000UL)
#define ML_MOTOR_DUTY_LIMIT (20000UL)

#define DL_GPIO_INVERSION_DISABLE 0U
#define DL_GPIO_RESISTOR_NONE 0U
#define DL_GPIO_RESISTOR_PULL_UP 1U
#define DL_GPIO_RESISTOR_PULL_DOWN 2U
#define DL_GPIO_HYSTERESIS_DISABLE 0U
#define DL_GPIO_HYSTERESIS_ENABLE 1U
#define DL_GPIO_WAKEUP_DISABLE 0U

void DL_GPIO_initDigitalOutput(uint32_t pincm_index);
void DL_GPIO_enableOutput(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_disableOutput(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_initDigitalInputFeatures(uint32_t pincm_index,
    uint32_t inversion, uint32_t resistor,
    uint32_t hysteresis, uint32_t wakeup);

static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

#endif
