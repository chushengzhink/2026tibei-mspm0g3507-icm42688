#include "q3_ball_telemetry.h"

#include <limits.h>
#include <string.h>

#include "ml_uart.h"
#include "q3_ball_config.h"

typedef struct {
    uint32_t time_ms;
    int16_t position_milli_cm;
    int16_t velocity_centi_cm_s;
    int16_t target_milli_cm;
    int16_t control_deci_us;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    int16_t neutral_deci_us;
    uint16_t response_scale_milli;
    int16_t predicted_stop_milli_cm;
    int16_t stall_progress_milli_cm;
    uint16_t vision_age_ms;
    uint16_t score_milli;
    uint8_t state;
    uint8_t flags;
    uint8_t profile_index;
    uint8_t rescue;
    uint16_t sequence_elapsed_ms;
    uint16_t frame_interval_ms;
    int16_t raw_x_px;
    int16_t raw_y_px;
} q3_telemetry_record_t;

typedef char q3_telemetry_record_size_must_be_40[
    (sizeof(q3_telemetry_record_t) == 40U) ? 1 : -1];

static q3_telemetry_record_t g_records[Q3_TELEMETRY_CAPACITY];
static uint16_t g_record_count;
static bool g_session_active;
static bool g_export_active;
static bool g_busy_sent;
static uint32_t g_last_busy_ms;

static int16_t q3_to_i16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= (float) INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled <= (float) INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t) (scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static uint16_t q3_to_u16(float value, float scale)
{
    float scaled = value * scale;

    if ((scaled <= 0.0f) || (scaled != scaled)) {
        return 0U;
    }
    if (scaled >= (float) UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t) (scaled + 0.5f);
}

static uint16_t q3_u32_to_u16(uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t) value;
}

static void q3_pack(q3_telemetry_record_t *record,
    const q3_ball_status_t *status)
{
    uint8_t flags = 0U;

    record->time_ms = status->uptime_ms;
    record->position_milli_cm = q3_to_i16(status->position_cm, 1000.0f);
    record->velocity_centi_cm_s =
        q3_to_i16(status->velocity_cm_per_s, 100.0f);
    record->target_milli_cm = q3_to_i16(status->target_cm, 1000.0f);
    record->control_deci_us = q3_to_i16(status->control_output_us, 10.0f);
    record->servo_target_us = status->servo_target_us;
    record->servo_current_us = status->servo_current_us;
    record->neutral_deci_us = q3_to_i16(status->neutral_us, 10.0f);
    record->response_scale_milli =
        q3_to_u16(status->response_scale, 1000.0f);
    record->predicted_stop_milli_cm =
        q3_to_i16(status->predicted_stop_cm, 1000.0f);
    record->stall_progress_milli_cm =
        q3_to_i16(status->stall_progress_cm, 1000.0f);
    record->vision_age_ms = q3_u32_to_u16(status->vision_age_ms);
    record->score_milli = q3_to_u16(status->raw_score, 1000.0f);
    if (status->vision_ready) {
        flags |= 0x01U;
    }
    if (status->brake_active) {
        flags |= 0x02U;
    }
    if (status->plus_captured) {
        flags |= 0x04U;
    }
    if (status->final_captured) {
        flags |= 0x08U;
    }
    if (status->sequence_completed) {
        flags |= 0x10U;
    }
    if (status->axis_sign < 0) {
        flags |= 0x20U;
    }
    record->state = (uint8_t) status->state;
    record->flags = flags;
    record->profile_index = status->profile_index;
    record->rescue = (uint8_t) status->rescue_stage |
        (uint8_t) (status->rescue_attempts << 4U);
    record->sequence_elapsed_ms =
        q3_u32_to_u16(status->sequence_elapsed_ms);
    record->frame_interval_ms =
        q3_u32_to_u16(status->vision_frame_interval_ms);
    record->raw_x_px = status->raw_center_x_px;
    record->raw_y_px = status->raw_center_y_px;
}

static uint16_t q3_append_char(char *line, uint16_t index, char value)
{
    if (index < 252U) {
        line[index++] = value;
    }
    return index;
}

static uint16_t q3_append_u32(char *line, uint16_t index, uint32_t value)
{
    char reversed[11];
    uint8_t count = 0U;
    uint8_t i;

    do {
        reversed[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(reversed)));
    for (i = 0U; i < count; ++i) {
        index = q3_append_char(line, index, reversed[count - 1U - i]);
    }
    return index;
}

static uint16_t q3_append_i32(char *line, uint16_t index, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        index = q3_append_char(line, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return q3_append_u32(line, index, magnitude);
}

static uint16_t q3_append_fixed(char *line, uint16_t index,
    int32_t scaled, uint32_t divisor)
{
    uint32_t magnitude;
    uint32_t remainder;
    uint8_t width = 0U;
    uint32_t value;

    if (scaled < 0) {
        index = q3_append_char(line, index, '-');
        magnitude = (uint32_t) (-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t) scaled;
    }
    index = q3_append_u32(line, index, magnitude / divisor);
    index = q3_append_char(line, index, '.');
    value = divisor / 10U;
    while (value > 0U) {
        ++width;
        value /= 10U;
    }
    remainder = magnitude % divisor;
    for (value = divisor / 10U; value > 0U; value /= 10U) {
        index = q3_append_char(line, index,
            (char) ('0' + ((remainder / value) % 10U)));
    }
    (void) width;
    return index;
}

static ml_status_t q3_uart_write(const char *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        if (uart_sendbyte(Q3_TELEMETRY_UART,
                (uint8_t) data[index]) != ML_STATUS_OK) {
            return ML_STATUS_TIMEOUT;
        }
    }
    return ML_STATUS_OK;
}

static ml_status_t q3_export_csv(void)
{
    static const char header[] =
        "time_ms,state,position_cm,velocity_cm_s,target_cm,control_us,"
        "servo_target_us,servo_current_us,neutral_us,response_scale,"
        "predicted_stop_cm,stall_progress_cm,vision_age_ms,score,flags,"
        "profile_index,rescue_stage,rescue_attempts,sequence_elapsed_ms,"
        "frame_interval_ms,raw_x_px,raw_y_px\r\n";
    char line[256];
    uint16_t i;
    uint16_t index;
    const q3_telemetry_record_t *record;

    if (q3_uart_write(header, (uint16_t) (sizeof(header) - 1U)) !=
        ML_STATUS_OK) {
        return ML_STATUS_TIMEOUT;
    }
    for (i = 0U; i < g_record_count; ++i) {
        record = &g_records[i];
        index = 0U;
#define Q3_COMMA() do { index = q3_append_char(line, index, ','); } while (0)
        index = q3_append_u32(line, index, record->time_ms); Q3_COMMA();
        index = q3_append_u32(line, index, record->state); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->position_milli_cm, 1000U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->velocity_centi_cm_s, 100U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->target_milli_cm, 1000U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->control_deci_us, 10U); Q3_COMMA();
        index = q3_append_u32(line, index, record->servo_target_us);
        Q3_COMMA();
        index = q3_append_u32(line, index, record->servo_current_us);
        Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->neutral_deci_us, 10U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->response_scale_milli, 1000U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->predicted_stop_milli_cm, 1000U); Q3_COMMA();
        index = q3_append_fixed(line, index,
            record->stall_progress_milli_cm, 1000U); Q3_COMMA();
        index = q3_append_u32(line, index, record->vision_age_ms);
        Q3_COMMA();
        index = q3_append_fixed(line, index, record->score_milli, 1000U);
        Q3_COMMA();
        index = q3_append_u32(line, index, record->flags); Q3_COMMA();
        index = q3_append_u32(line, index, record->profile_index);
        Q3_COMMA();
        index = q3_append_u32(line, index, record->rescue & 0x0FU);
        Q3_COMMA();
        index = q3_append_u32(line, index, record->rescue >> 4U);
        Q3_COMMA();
        index = q3_append_u32(line, index,
            record->sequence_elapsed_ms); Q3_COMMA();
        index = q3_append_u32(line, index, record->frame_interval_ms);
        Q3_COMMA();
        index = q3_append_i32(line, index, record->raw_x_px); Q3_COMMA();
        index = q3_append_i32(line, index, record->raw_y_px);
        index = q3_append_char(line, index, '\r');
        index = q3_append_char(line, index, '\n');
        if (q3_uart_write(line, index) != ML_STATUS_OK) {
            return ML_STATUS_TIMEOUT;
        }
#undef Q3_COMMA
    }
    return ML_STATUS_OK;
}

void q3_telemetry_storage_init(void)
{
    memset(g_records, 0, sizeof(g_records));
    g_record_count = 0U;
    g_session_active = false;
    g_export_active = false;
    g_busy_sent = false;
    g_last_busy_ms = 0U;
}

ml_status_t q3_telemetry_uart0_init(void)
{
    return uart_init(Q3_TELEMETRY_UART, Q3_TELEMETRY_UART_BAUD,
        Q3_TELEMETRY_UART_PRIORITY);
}

ml_status_t q3_telemetry_init(void)
{
    q3_telemetry_storage_init();
    return q3_telemetry_uart0_init();
}

void q3_telemetry_session_start(void)
{
    g_record_count = 0U;
    g_session_active = true;
}

void q3_telemetry_session_finish(const q3_ball_status_t *status)
{
    if (g_session_active && (status != 0)) {
        (void) q3_telemetry_record(status);
    }
    g_session_active = false;
}

ml_status_t q3_telemetry_record(const q3_ball_status_t *status)
{
    if ((status == 0) || !g_session_active) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if ((g_record_count > 0U) &&
        (status->uptime_ms == g_records[g_record_count - 1U].time_ms)) {
        q3_pack(&g_records[g_record_count - 1U], status);
        return ML_STATUS_OK;
    }
    if (g_record_count >= Q3_TELEMETRY_CAPACITY) {
        return ML_STATUS_BUFFER_FULL;
    }
    q3_pack(&g_records[g_record_count++], status);
    return ML_STATUS_OK;
}

bool q3_telemetry_session_active(void)
{
    return g_session_active;
}

uint16_t q3_telemetry_count(void)
{
    return g_record_count;
}

ml_status_t q3_telemetry_uart0_handle_byte(uint8_t byte,
    bool export_allowed, uint32_t now_ms)
{
    bool is_export = (byte == (uint8_t) 'D') ||
        (byte == (uint8_t) 'd');
    bool is_clear = (byte == (uint8_t) 'C') ||
        (byte == (uint8_t) 'c');

    if (!is_export && !is_clear) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!export_allowed || g_session_active) {
        static const char busy[] = "BUSY\r\n";

        if (!g_busy_sent || ((now_ms - g_last_busy_ms) >= 1000U)) {
            g_busy_sent = true;
            g_last_busy_ms = now_ms;
            (void) q3_uart_write(busy,
                (uint16_t) (sizeof(busy) - 1U));
        }
        return ML_STATUS_BUSY;
    }
    g_busy_sent = false;
    if (is_clear) {
        g_record_count = 0U;
        return ML_STATUS_OK;
    }
    if (g_record_count == 0U) {
        static const char empty[] = "EMPTY\r\n";

        (void) q3_uart_write(empty, (uint16_t) (sizeof(empty) - 1U));
        return ML_STATUS_BUFFER_EMPTY;
    }
    if (g_export_active) {
        return ML_STATUS_BUSY;
    }
    g_export_active = true;
    {
        ml_status_t status = q3_export_csv();

        g_export_active = false;
        return status;
    }
}
