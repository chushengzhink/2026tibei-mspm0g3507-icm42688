#ifndef TEST_Q3_APP_ML_BOARD_H
#define TEST_Q3_APP_ML_BOARD_H

#include "ml_common.h"

typedef struct GPIO_Regs { uint8_t id; } GPIO_Regs;
typedef struct UART_Regs { uint8_t id; } UART_Regs;

extern GPIO_Regs g_test_gpioa;
extern GPIO_Regs g_test_gpiob;
extern UART_Regs g_test_uart0;

#define GPIOA (&g_test_gpioa)
#define GPIOB (&g_test_gpiob)
#define UART0 (&g_test_uart0)

#define ML_KEY_ACTIVE_LEVEL  (0U)
#define ML_KEY_CENTER_PORT   GPIOB
#define ML_KEY_CENTER_PIN    (1UL << 24U)
#define ML_KEY_CENTER_IOMUX  (52U)

typedef enum { ML_BOARD_RESOURCE_PB24 = 0 } ml_board_resource_t;
typedef enum { ML_BOARD_OWNER_KEY = 0 } ml_board_owner_t;

ml_status_t board_led_init(void);
void board_led_off(void);
void board_led_toggle(void);
ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner);
void board_resource_release(
    ml_board_resource_t resource, ml_board_owner_t owner);

#endif
