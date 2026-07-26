#ifndef ML_UART_H
#define ML_UART_H

#include "ml_board.h"

typedef void (*uart_rx_callback_t)(
    UART_Regs *uart, uint8_t byte, void *context);

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority);
ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte);
uint8_t uart_getbyte(UART_Regs *uart);
ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte);
ml_status_t uart_set_rx_callback(
    UART_Regs *uart, uart_rx_callback_t callback, void *context);
uint32_t uart_get_rx_overflow_count(UART_Regs *uart);
void uart_irq_dispatch(UART_Regs *uart);

#endif
