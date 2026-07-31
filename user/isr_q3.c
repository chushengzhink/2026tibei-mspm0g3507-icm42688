#include "ml_tim.h"
#include "ml_uart.h"

void TIMG6_IRQHandler(void)
{
    tim_irq_dispatch(TIMG6);
}

void UART0_IRQHandler(void)
{
    uart_irq_dispatch(UART0);
}

void UART2_IRQHandler(void)
{
    uart_irq_dispatch(UART2);
}
