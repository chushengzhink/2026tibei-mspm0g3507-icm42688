#ifndef CHASSIS_TELEMETRY_UART_H
#define CHASSIS_TELEMETRY_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

ml_status_t chassis_telemetry_uart0_handle_byte(
    uint8_t byte, bool chassis_stopped);
ml_status_t chassis_uart0_send_busy(void);
ml_status_t chassis_uart0_send_diagnostic_banner(uint32_t sequence);
ml_status_t chassis_uart0_send_diagnostic_rx(
    uint8_t byte, uint8_t error_flags);

#endif
