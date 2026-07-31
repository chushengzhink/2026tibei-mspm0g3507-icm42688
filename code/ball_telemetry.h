#ifndef BALL_TELEMETRY_H
#define BALL_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_balance.h"

#define BALL_TELEMETRY_CAPACITY       (600U)
#define BALL_TELEMETRY_PERIOD_MS      (10U)
#define BALL_TELEMETRY_UART_BAUD      (115200UL)
#define BALL_TELEMETRY_UART_PRIORITY  (3U)

ml_status_t ball_telemetry_init(void);
void ball_telemetry_session_start(void);
ml_status_t ball_telemetry_record(const ball_balance_status_t *status);
void ball_telemetry_session_finish(const ball_balance_status_t *status);
bool ball_telemetry_session_active(void);
uint16_t ball_telemetry_count(void);
bool ball_telemetry_full(void);
ml_status_t ball_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms);

#endif
