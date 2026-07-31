#ifndef ML_UART_H
#define ML_UART_H

#include "ml_board.h"

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority);
ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte);
ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte);

#endif
