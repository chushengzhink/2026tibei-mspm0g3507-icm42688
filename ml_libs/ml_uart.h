#ifndef ML_UART_H
#define ML_UART_H

#include "ml_board.h"

typedef void (*uart_rx_callback_t)(
    UART_Regs *uart, uint8_t byte, void *context);

#define ML_UART_RX_ERROR_OVERRUN (1U << 0)
#define ML_UART_RX_ERROR_BREAK   (1U << 1)
#define ML_UART_RX_ERROR_PARITY  (1U << 2)
#define ML_UART_RX_ERROR_FRAMING (1U << 3)
#define ML_UART_RX_ERROR_NOISE   (1U << 4)

typedef struct {
    uint32_t tx_bytes;
    uint32_t tx_timeouts;
    uint32_t rx_bytes;
    uint32_t rx_queue_overflows;
    uint32_t rx_overrun_errors;
    uint32_t rx_break_errors;
    uint32_t rx_parity_errors;
    uint32_t rx_framing_errors;
    uint32_t rx_noise_errors;
    uint16_t integer_divisor;
    uint8_t fractional_divisor;
    uint8_t last_rx_byte;
    uint8_t last_rx_errors;
    bool last_rx_valid;
} uart_diagnostics_t;

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority);
ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte);
uint8_t uart_getbyte(UART_Regs *uart);
ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte);
ml_status_t uart_set_rx_callback(
    UART_Regs *uart, uart_rx_callback_t callback, void *context);
uint32_t uart_get_rx_overflow_count(UART_Regs *uart);
ml_status_t uart_get_diagnostics(
    UART_Regs *uart, uart_diagnostics_t *diagnostics);
void uart_irq_dispatch(UART_Regs *uart);

#endif
