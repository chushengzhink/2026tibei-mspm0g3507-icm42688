#include "ball_telemetry.h"

#include <limits.h>
#include <string.h>

#include "ml_board.h"
#include "ml_uart.h"

#define BALL_TELEMETRY_LINE_SIZE (224U)
#define BALL_TELEMETRY_STATE_MASK (0x0FU)
#define BALL_TELEMETRY_SEQUENCE_SHIFT (4U)
#define BALL_TELEMETRY_BREAKAWAY_ACTIVE_MASK (0x01U)
#define BALL_TELEMETRY_BRAKE_ACTIVE_MASK (0x02U)

typedef struct {
    uint32_t time_ms;
    int16_t target_milli_cm;
    int16_t position_milli_cm;
    int16_t error_milli_cm;
    int16_t integral_milli_cm_s;
    int16_t target_velocity_centi_cm_s;
    int16_t velocity_centi_cm_s;
    int16_t speed_error_centi_cm_s;
    int16_t control_deci_us;
    int16_t breakaway_boost_deci_us;
    int16_t raw_x_px;
    int16_t raw_y_px;
    uint16_t score_milli;
    uint16_t vision_age_ms;
    uint16_t frame_interval_ms;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    uint16_t sequence_elapsed_ms;
    uint8_t state_sequence;
    uint8_t control_mode;
    uint8_t enabled;
    uint8_t vision_ready;
    uint8_t breakaway_flags;
    uint8_t breakaway_fault;
} ball_telemetry_record_t;

typedef char ball_telemetry_record_must_be_44_bytes[
    (sizeof(ball_telemetry_record_t) == 44U) ? 1 : -1];
typedef char ball_telemetry_state_must_fit_nibble[
    (BALL_BALANCE_BREAKAWAY_FAULT <= BALL_TELEMETRY_STATE_MASK) ? 1 : -1];
typedef char ball_telemetry_sequence_must_fit_nibble[
    (BALL_SEQUENCE_VISION_LOST <= BALL_TELEMETRY_STATE_MASK) ? 1 : -1];

static ball_telemetry_record_t
    g_records[BALL_TELEMETRY_CAPACITY];
static uint16_t g_record_count;
static bool g_session_active;
static bool g_buffer_full;
static bool g_export_active;
static bool g_busy_sent;
static uint32_t g_last_busy_ms;

static int16_t ball_telemetry_float_to_i16(float value, float scale)
{
    float scaled = value * scale;

    if ((value != value) || (scaled >= (float) INT16_MAX)) {
        return INT16_MAX;
    }
    if (scaled <= (float) INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t) (scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static uint16_t ball_telemetry_float_to_u16(float value, float scale)
{
    float scaled = value * scale;

    if ((value != value) || (scaled <= 0.0f)) {
        return 0U;
    }
    if (scaled >= (float) UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t) (scaled + 0.5f);
}

static uint16_t ball_telemetry_u32_to_u16(uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t) value;
}

static void ball_telemetry_pack(ball_telemetry_record_t *record,
    const ball_balance_status_t *status)
{
    record->time_ms = status->uptime_ms;
    record->target_milli_cm = ball_telemetry_float_to_i16(
        status->target_cm, 1000.0f);
    record->position_milli_cm = ball_telemetry_float_to_i16(
        status->position_cm, 1000.0f);
    record->error_milli_cm = ball_telemetry_float_to_i16(
        status->error_cm, 1000.0f);
    record->integral_milli_cm_s = ball_telemetry_float_to_i16(
        status->integral_cm_s, 1000.0f);
    record->target_velocity_centi_cm_s = ball_telemetry_float_to_i16(
        status->target_velocity_cm_per_s, 100.0f);
    record->velocity_centi_cm_s = ball_telemetry_float_to_i16(
        status->velocity_cm_per_s, 100.0f);
    record->speed_error_centi_cm_s = ball_telemetry_float_to_i16(
        status->speed_error_cm_per_s, 100.0f);
    record->control_deci_us = ball_telemetry_float_to_i16(
        status->control_output_us, 10.0f);
    record->breakaway_boost_deci_us = ball_telemetry_float_to_i16(
        status->breakaway_boost_us, 10.0f);
    record->raw_x_px = status->raw_center_x_px;
    record->raw_y_px = status->raw_center_y_px;
    record->score_milli = ball_telemetry_float_to_u16(
        status->raw_score, 1000.0f);
    record->vision_age_ms = ball_telemetry_u32_to_u16(
        status->vision_age_ms);
    record->frame_interval_ms = ball_telemetry_u32_to_u16(
        status->vision_frame_interval_ms);
    record->servo_target_us = status->servo_target_us;
    record->servo_current_us = status->servo_current_us;
    record->sequence_elapsed_ms = ball_telemetry_u32_to_u16(
        status->sequence_elapsed_ms);
    record->state_sequence =
        ((uint8_t) status->state & BALL_TELEMETRY_STATE_MASK) |
        (uint8_t) (((uint8_t) status->sequence_state &
                    BALL_TELEMETRY_STATE_MASK) <<
                   BALL_TELEMETRY_SEQUENCE_SHIFT);
    record->control_mode = (uint8_t) status->control_mode;
    record->enabled = status->enabled ? 1U : 0U;
    record->vision_ready = status->vision_ready ? 1U : 0U;
    record->breakaway_flags =
        (status->breakaway_active ?
         BALL_TELEMETRY_BREAKAWAY_ACTIVE_MASK : 0U) |
        (status->brake_active ?
         BALL_TELEMETRY_BRAKE_ACTIVE_MASK : 0U);
    record->breakaway_fault = status->breakaway_fault ? 1U : 0U;
}

static uint16_t ball_telemetry_append_char(
    char *buffer, uint16_t index, char value)
{
    if (index < BALL_TELEMETRY_LINE_SIZE) {
        buffer[index++] = value;
    }
    return index;
}

static uint16_t ball_telemetry_append_u32(
    char *buffer, uint16_t index, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));
    while (count > 0U) {
        index = ball_telemetry_append_char(
            buffer, index, digits[--count]);
    }
    return index;
}

static uint16_t ball_telemetry_append_i32(
    char *buffer, uint16_t index, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        index = ball_telemetry_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return ball_telemetry_append_u32(buffer, index, magnitude);
}

static uint16_t ball_telemetry_append_fixed(
    char *buffer, uint16_t index, int32_t value,
    uint32_t scale, uint8_t decimals)
{
    uint32_t magnitude;
    uint32_t divisor;
    uint8_t digit;

    if (value < 0) {
        index = ball_telemetry_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    index = ball_telemetry_append_u32(
        buffer, index, magnitude / scale);
    index = ball_telemetry_append_char(buffer, index, '.');
    divisor = scale / 10U;
    for (digit = 0U; digit < decimals; ++digit) {
        index = ball_telemetry_append_char(buffer, index,
            (char) ('0' + ((magnitude / divisor) % 10U)));
        divisor /= 10U;
    }
    return index;
}

static ml_status_t ball_telemetry_uart_write(
    const char *data, uint16_t length)
{
    uint16_t index;
    ml_status_t status = ML_STATUS_OK;

    for (index = 0U; (index < length) &&
         (status == ML_STATUS_OK); ++index) {
        status = uart_sendbyte(UART0, (uint8_t) data[index]);
    }
    return status;
}

static ml_status_t ball_telemetry_export_csv(void)
{
    static const char header[] =
        "time_ms,state,sequence_state,sequence_elapsed_ms,control_mode,"
        "enabled,vision_ready,target_cm,"
        "position_cm,error_cm,integral_cm_s,target_velocity_cm_s,"
        "velocity_cm_s,speed_error_cm_s,control_output_us,"
        "breakaway_active,brake_active,breakaway_boost_us,"
        "breakaway_fault,"
        "servo_target_us,"
        "servo_current_us,raw_x_px,raw_y_px,score,vision_age_ms,"
        "frame_interval_ms\r\n";
    char line[BALL_TELEMETRY_LINE_SIZE];
    uint16_t record_index;
    ml_status_t status;

    status = ball_telemetry_uart_write(
        header, (uint16_t) (sizeof(header) - 1U));
    for (record_index = 0U; (record_index < g_record_count) &&
         (status == ML_STATUS_OK); ++record_index) {
        const ball_telemetry_record_t *record =
            &g_records[record_index];
        uint16_t index = 0U;

#define BALL_CSV_COMMA() \
        do { index = ball_telemetry_append_char(line, index, ','); } while (0)
        index = ball_telemetry_append_u32(line, index, record->time_ms);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(line, index,
            record->state_sequence & BALL_TELEMETRY_STATE_MASK);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(line, index,
            (record->state_sequence >> BALL_TELEMETRY_SEQUENCE_SHIFT) &
            BALL_TELEMETRY_STATE_MASK);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->sequence_elapsed_ms);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(line, index, record->control_mode);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(line, index, record->enabled);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(line, index, record->vision_ready);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->target_milli_cm, 1000U, 3U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->position_milli_cm, 1000U, 3U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->error_milli_cm, 1000U, 3U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->integral_milli_cm_s, 1000U, 3U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(line, index,
            record->target_velocity_centi_cm_s, 100U, 2U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(line, index,
            record->velocity_centi_cm_s, 100U, 2U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(line, index,
            record->speed_error_centi_cm_s, 100U, 2U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->control_deci_us, 10U, 1U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->breakaway_flags &
            BALL_TELEMETRY_BREAKAWAY_ACTIVE_MASK);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index,
            (record->breakaway_flags &
             BALL_TELEMETRY_BRAKE_ACTIVE_MASK) ? 1U : 0U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->breakaway_boost_deci_us, 10U, 1U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->breakaway_fault);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->servo_target_us);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->servo_current_us);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_i32(line, index, record->raw_x_px);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_i32(line, index, record->raw_y_px);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_fixed(
            line, index, record->score_milli, 1000U, 3U);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->vision_age_ms);
        BALL_CSV_COMMA();
        index = ball_telemetry_append_u32(
            line, index, record->frame_interval_ms);
        index = ball_telemetry_append_char(line, index, '\r');
        index = ball_telemetry_append_char(line, index, '\n');
#undef BALL_CSV_COMMA
        status = ball_telemetry_uart_write(line, index);
    }
    return status;
}

ml_status_t ball_telemetry_init(void)
{
    memset(g_records, 0, sizeof(g_records));
    g_record_count = 0U;
    g_session_active = false;
    g_buffer_full = false;
    g_export_active = false;
    g_busy_sent = false;
    g_last_busy_ms = 0U;
    return uart_init(UART0, BALL_TELEMETRY_UART_BAUD,
        BALL_TELEMETRY_UART_PRIORITY);
}

void ball_telemetry_session_start(void)
{
    g_record_count = 0U;
    g_buffer_full = false;
    g_session_active = true;
}

ml_status_t ball_telemetry_record(const ball_balance_status_t *status)
{
    ball_telemetry_record_t *record;

    if (status == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_session_active) {
        return ML_STATUS_BUSY;
    }
    if (g_record_count > 0U) {
        uint32_t previous_time = g_records[g_record_count - 1U].time_ms;

        if (status->uptime_ms < previous_time) {
            return ML_STATUS_INVALID_ARGUMENT;
        }
        if (status->uptime_ms == previous_time) {
            ball_telemetry_pack(
                &g_records[g_record_count - 1U], status);
            return ML_STATUS_OK;
        }
        if ((status->uptime_ms - previous_time) <
            BALL_TELEMETRY_PERIOD_MS) {
            return ML_STATUS_BUSY;
        }
    }
    if (g_record_count >= BALL_TELEMETRY_CAPACITY) {
        g_buffer_full = true;
        return ML_STATUS_BUFFER_FULL;
    }
    record = &g_records[g_record_count++];
    ball_telemetry_pack(record, status);
    if (g_record_count >= BALL_TELEMETRY_CAPACITY) {
        g_buffer_full = true;
    }
    return ML_STATUS_OK;
}

void ball_telemetry_session_finish(const ball_balance_status_t *status)
{
    if (g_session_active && (status != 0)) {
        (void) ball_telemetry_record(status);
    }
    g_session_active = false;
}

bool ball_telemetry_session_active(void)
{
    return g_session_active;
}

uint16_t ball_telemetry_count(void)
{
    return g_record_count;
}

bool ball_telemetry_full(void)
{
    return g_buffer_full;
}

ml_status_t ball_telemetry_uart0_handle_byte(
    uint8_t byte, bool export_allowed, uint32_t now_ms)
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

        if (!g_busy_sent ||
            ((now_ms - g_last_busy_ms) >= 1000U)) {
            g_busy_sent = true;
            g_last_busy_ms = now_ms;
            (void) ball_telemetry_uart_write(
                busy, (uint16_t) (sizeof(busy) - 1U));
        }
        return ML_STATUS_BUSY;
    }
    g_busy_sent = false;
    if (is_clear) {
        g_record_count = 0U;
        g_buffer_full = false;
        return ML_STATUS_OK;
    }
    if (g_record_count == 0U) {
        static const char empty[] = "EMPTY\r\n";

        (void) ball_telemetry_uart_write(
            empty, (uint16_t) (sizeof(empty) - 1U));
        return ML_STATUS_BUFFER_EMPTY;
    }
    if (g_export_active) {
        return ML_STATUS_BUSY;
    }
    g_export_active = true;
    {
        ml_status_t status = ball_telemetry_export_csv();

        g_export_active = false;
        return status;
    }
}
