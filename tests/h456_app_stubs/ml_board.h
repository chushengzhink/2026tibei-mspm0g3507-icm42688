#ifndef ML_BOARD_H
#define ML_BOARD_H

#include "ml_common.h"

typedef struct GPIO_Regs { uint8_t id; } GPIO_Regs;
typedef struct UART_Regs { uint8_t id; } UART_Regs;
typedef struct GPTIMER_Regs { uint8_t id; } GPTIMER_Regs;

extern GPIO_Regs g_test_gpioa;
extern GPIO_Regs g_test_gpiob;
extern UART_Regs g_test_uart0;
extern GPTIMER_Regs g_test_timg8;

#define GPIOA (&g_test_gpioa)
#define GPIOB (&g_test_gpiob)
#define UART0 (&g_test_uart0)
#define TIMG8 (&g_test_timg8)

#define ML_KEY_ACTIVE_LEVEL  (0U)
#define ML_KEY_UP_PORT       GPIOA
#define ML_KEY_UP_PIN        (1UL << 14U)
#define ML_KEY_UP_IOMUX      (36U)
#define ML_KEY_LEFT_PORT     GPIOA
#define ML_KEY_LEFT_PIN      (1UL << 15U)
#define ML_KEY_LEFT_IOMUX    (37U)
#define ML_KEY_DOWN_PORT     GPIOA
#define ML_KEY_DOWN_PIN      (1UL << 24U)
#define ML_KEY_DOWN_IOMUX    (54U)
#define ML_KEY_CENTER_PORT   GPIOB
#define ML_KEY_CENTER_PIN    (1UL << 24U)
#define ML_KEY_CENTER_IOMUX  (52U)
#define ML_KEY_RIGHT_PORT    GPIOB
#define ML_KEY_RIGHT_PIN     (1UL << 25U)
#define ML_KEY_RIGHT_IOMUX   (56U)

typedef enum { ML_BOARD_RESOURCE_PB24 = 0 } ml_board_resource_t;
typedef enum { ML_BOARD_OWNER_KEY = 0 } ml_board_owner_t;

ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner);

#endif
