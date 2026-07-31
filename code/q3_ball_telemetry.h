#ifndef Q3_BALL_TELEMETRY_H
#define Q3_BALL_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "q3_ball.h"

#define Q3_TELEMETRY_CAPACITY (512U)

ml_status_t q3_telemetry_init(void);
void q3_telemetry_session_start(void);
void q3_telemetry_session_finish(const q3_ball_status_t *status);
ml_status_t q3_telemetry_record(const q3_ball_status_t *status);
bool q3_telemetry_session_active(void);
uint16_t q3_telemetry_count(void);
ml_status_t q3_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms);

#endif
