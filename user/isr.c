#include "ml_exti.h"
#include "ml_tim.h"
#include "ml_uart.h"

void TIMG0_IRQHandler(void)
{
    tim_irq_dispatch(TIMG0);
}

void TIMG6_IRQHandler(void)
{
    tim_irq_dispatch(TIMG6);
}

void TIMG7_IRQHandler(void)
{
    tim_irq_dispatch(TIMG7);
}

void TIMG8_IRQHandler(void)
{
    tim_irq_dispatch(TIMG8);
}

void TIMG12_IRQHandler(void)
{
    tim_irq_dispatch(TIMG12);
}

void UART0_IRQHandler(void)
{
    uart_irq_dispatch(UART0);
}

void UART1_IRQHandler(void)
{
    uart_irq_dispatch(UART1);
}

void UART2_IRQHandler(void)
{
    uart_irq_dispatch(UART2);
}

void UART3_IRQHandler(void)
{
    uart_irq_dispatch(UART3);
}

void GROUP1_IRQHandler(void)
{
    exti_irq_dispatch();
}
