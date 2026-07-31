#include "h456_telemetry.h"

#include <float.h>
#include <limits.h>
#include <string.h>

#include "ml_uart.h"

#define H456_TELEMETRY_LINE_SIZE (288U)
#define H456_LINE_USABLE_MASK    (0x01U)
#define H456_LINE_RECOVERING_MASK (0x02U)
#define H456_BALL_VISION_MASK    (0x01U)
#define H456_BALL_ENABLED_MASK   (0x02U)
#define H456_BALL_VIOLATION_MASK (0x04U)
#define H456_BALL_BREAKAWAY_MASK (0x08U)
#define H456_SCORE_PASSED_MASK   (0x10U)

typedef struct {
    uint32_t time_ms;
    uint16_t progress_mm;
    uint16_t fused_heading_cdeg;
    int16_t target_center_dmm_s;
    int16_t actual_center_dmm_s;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    uint8_t line_bits;
    uint8_t line_flags;
    int8_t line_correction_mm_s;
    int8_t final_steering_bias_mm_s;
    int16_t ball_target_milli_cm;
    int16_t ball_position_milli_cm;
    int8_t ball_error_min_deci_cm;
    int8_t ball_error_max_deci_cm;
    int16_t ball_velocity_centi_cm_s;
    int16_t ball_control_deci_us;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    uint16_t vision_age_ms;
    uint16_t frame_interval_ms;
    uint8_t ball_flags;
    uint8_t mode_state;
} h456_telemetry_record_t;

typedef char h456_telemetry_record_must_be_40_bytes[
    (sizeof(h456_telemetry_record_t) == H456_TELEMETRY_RECORD_BYTES) ?
    1 : -1];

static h456_telemetry_record_t g_records[H456_TELEMETRY_CAPACITY];
static uint16_t g_record_count;
static uint16_t g_period_ms;
static uint32_t g_start_time_ms;
static uint32_t g_score_elapsed_ms;
static int16_t g_maximum_ball_error_milli_cm;
static bool g_ball_score_passed;
static bool g_session_active;
static bool g_buffer_full;
static bool g_export_active;
static bool g_busy_sent;
static uint32_t g_last_busy_ms;

static bool h456_telemetry_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static int32_t h456_telemetry_round(float value)
{
    return (int32_t) (value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int16_t h456_telemetry_i16(float value, float scale)
{
    float scaled = value * scale;

    if (!h456_telemetry_float_valid(value) ||
        (scaled >= (float) INT16_MAX)) {
        return INT16_MAX;
    }
    if (scaled <= (float) INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t) h456_telemetry_round(scaled);
}

static int8_t h456_telemetry_i8(float value, float scale)
{
    float scaled = value * scale;

    if (!h456_telemetry_float_valid(value) ||
        (scaled >= (float) INT8_MAX)) {
        return INT8_MAX;
    }
    if (scaled <= (float) INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t) h456_telemetry_round(scaled);
}

static uint16_t h456_telemetry_u16(float value, float scale)
{
    float scaled = value * scale;

    if (!h456_telemetry_float_valid(value) || (scaled <= 0.0f)) {
        return 0U;
    }
    if (scaled >= (float) UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t) (scaled + 0.5f);
}

static uint16_t h456_telemetry_u32_to_u16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t) value;
}

static void h456_telemetry_pack(h456_telemetry_record_t *record,
    const h456_telemetry_sample_t *sample)
{
    record->time_ms = sample->timestamp_ms - g_start_time_ms;
    record->progress_mm = h456_telemetry_u16(sample->progress_mm, 1.0f);
    record->fused_heading_cdeg =
        h456_telemetry_u16(sample->fused_heading_deg, 100.0f);
    record->target_center_dmm_s =
        h456_telemetry_i16(sample->target_center_mm_s, 10.0f);
    record->actual_center_dmm_s =
        h456_telemetry_i16(sample->actual_center_mm_s, 10.0f);
    record->pwm_left_count = sample->pwm_left_count;
    record->pwm_right_count = sample->pwm_right_count;
    record->line_bits = sample->line_bits;
    record->line_flags =
        (sample->line_usable ? H456_LINE_USABLE_MASK : 0U) |
        (sample->line_recovering ? H456_LINE_RECOVERING_MASK : 0U);
    record->line_correction_mm_s =
        h456_telemetry_i8(sample->line_correction_mm_s, 1.0f);
    record->final_steering_bias_mm_s =
        h456_telemetry_i8(sample->final_steering_bias_mm_s, 1.0f);
    record->ball_target_milli_cm =
        h456_telemetry_i16(sample->ball_target_cm, 1000.0f);
    record->ball_position_milli_cm =
        h456_telemetry_i16(sample->ball_position_cm, 1000.0f);
    record->ball_error_min_deci_cm =
        h456_telemetry_i8(sample->ball_error_min_cm, 10.0f);
    record->ball_error_max_deci_cm =
        h456_telemetry_i8(sample->ball_error_max_cm, 10.0f);
    record->ball_velocity_centi_cm_s =
        h456_telemetry_i16(sample->ball_velocity_cm_s, 100.0f);
    record->ball_control_deci_us =
        h456_telemetry_i16(sample->ball_control_output_us, 10.0f);
    record->servo_target_us = sample->servo_target_us;
    record->servo_current_us = sample->servo_current_us;
    record->vision_age_ms =
        h456_telemetry_u32_to_u16(sample->vision_age_ms);
    record->frame_interval_ms =
        h456_telemetry_u32_to_u16(sample->frame_interval_ms);
    record->ball_flags =
        (sample->vision_ready ? H456_BALL_VISION_MASK : 0U) |
        (sample->ball_enabled ? H456_BALL_ENABLED_MASK : 0U) |
        (sample->ball_violation ? H456_BALL_VIOLATION_MASK : 0U) |
        (sample->breakaway_fault ? H456_BALL_BREAKAWAY_MASK : 0U) |
        (sample->score_point_passed ? H456_SCORE_PASSED_MASK : 0U);
    record->mode_state = ((uint8_t) sample->mode & 0x0FU) |
        (uint8_t) (((uint8_t) sample->mission_state & 0x0FU) << 4U);
}

static uint16_t h456_append_char(
    char *buffer, uint16_t index, char value)
{
    if (index < H456_TELEMETRY_LINE_SIZE) {
        buffer[index++] = value;
    }
    return index;
}

static uint16_t h456_append_u32(
    char *buffer, uint16_t index, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));
    while (count > 0U) {
        index = h456_append_char(buffer, index, digits[--count]);
    }
    return index;
}

static uint16_t h456_append_i32(
    char *buffer, uint16_t index, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        index = h456_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return h456_append_u32(buffer, index, magnitude);
}

static uint16_t h456_append_fixed(char *buffer, uint16_t index,
    int32_t value, uint32_t scale, uint8_t decimals)
{
    uint32_t magnitude;
    uint32_t divisor;
    uint8_t digit;

    if (value < 0) {
        index = h456_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    index = h456_append_u32(buffer, index, magnitude / scale);
    index = h456_append_char(buffer, index, '.');
    divisor = scale / 10U;
    for (digit = 0U; digit < decimals; ++digit) {
        index = h456_append_char(buffer, index,
            (char) ('0' + ((magnitude / divisor) % 10U)));
        divisor /= 10U;
    }
    return index;
}

static ml_status_t h456_uart_write(const char *data, uint16_t length)
{
    uint16_t index;
    ml_status_t status = ML_STATUS_OK;

    for (index = 0U; (index < length) &&
         (status == ML_STATUS_OK); ++index) {
        status = uart_sendbyte(UART0, (uint8_t) data[index]);
    }
    return status;
}

static ml_status_t h456_export_csv(void)
{
    static const char header[] =
        "time_ms,mode,mission_state,score_elapsed_ms,ball_score_passed,"
        "maximum_ball_error_cm,score_point_passed,progress_mm,"
        "fused_heading_deg,target_center_mm_s,actual_center_mm_s,"
        "left_pwm_count,right_pwm_count,line_bits,line_usable,"
        "line_recovering,line_correction_mm_s,final_steering_bias_mm_s,"
        "ball_target_cm,ball_position_cm,ball_error_min_cm,"
        "ball_error_max_cm,ball_velocity_cm_s,ball_control_output_us,"
        "servo_target_us,servo_current_us,vision_age_ms,"
        "frame_interval_ms,vision_ready,ball_enabled,ball_violation,"
        "breakaway_fault\r\n";
    char line[H456_TELEMETRY_LINE_SIZE];
    uint16_t record_index;
    ml_status_t status;

    status = h456_uart_write(header,
        (uint16_t) (sizeof(header) - 1U));
    for (record_index = 0U; (record_index < g_record_count) &&
         (status == ML_STATUS_OK); ++record_index) {
        const h456_telemetry_record_t *record =
            &g_records[record_index];
        uint16_t index = 0U;

#define H456_CSV_COMMA() \
        do { index = h456_append_char(line, index, ','); } while (0)
        index = h456_append_u32(line, index, record->time_ms);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            record->mode_state & 0x0FU);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->mode_state >> 4U) & 0x0FU);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, g_score_elapsed_ms);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            g_ball_score_passed ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            g_maximum_ball_error_milli_cm, 1000U, 3U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->ball_flags & H456_SCORE_PASSED_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->progress_mm);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->fused_heading_cdeg, 100U, 2U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->target_center_dmm_s, 10U, 1U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->actual_center_dmm_s, 10U, 1U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->pwm_left_count);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->pwm_right_count);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->line_bits);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->line_flags & H456_LINE_USABLE_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->line_flags & H456_LINE_RECOVERING_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_i32(line, index,
            record->line_correction_mm_s);
        H456_CSV_COMMA();
        index = h456_append_i32(line, index,
            record->final_steering_bias_mm_s);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_target_milli_cm, 1000U, 3U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_position_milli_cm, 1000U, 3U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_error_min_deci_cm, 10U, 1U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_error_max_deci_cm, 10U, 1U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_velocity_centi_cm_s, 100U, 2U);
        H456_CSV_COMMA();
        index = h456_append_fixed(line, index,
            record->ball_control_deci_us, 10U, 1U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->servo_target_us);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->servo_current_us);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->vision_age_ms);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index, record->frame_interval_ms);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->ball_flags & H456_BALL_VISION_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->ball_flags & H456_BALL_ENABLED_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->ball_flags & H456_BALL_VIOLATION_MASK) ? 1U : 0U);
        H456_CSV_COMMA();
        index = h456_append_u32(line, index,
            (record->ball_flags & H456_BALL_BREAKAWAY_MASK) ? 1U : 0U);
        index = h456_append_char(line, index, '\r');
        index = h456_append_char(line, index, '\n');
#undef H456_CSV_COMMA
        status = h456_uart_write(line, index);
    }
    return status;
}

ml_status_t h456_telemetry_init(void)
{
    memset(g_records, 0, sizeof(g_records));
    g_record_count = 0U;
    g_period_ms = H456_TELEMETRY_H4_PERIOD_MS;
    g_start_time_ms = 0U;
    g_score_elapsed_ms = 0U;
    g_maximum_ball_error_milli_cm = 0;
    g_ball_score_passed = false;
    g_session_active = false;
    g_buffer_full = false;
    g_export_active = false;
    g_busy_sent = false;
    g_last_busy_ms = 0U;
    return ML_STATUS_OK;
}

void h456_telemetry_session_start(
    h456_mode_t mode, uint32_t start_time_ms)
{
    g_record_count = 0U;
    g_buffer_full = false;
    g_start_time_ms = start_time_ms;
    g_period_ms = mode == H456_MODE_4 ?
        H456_TELEMETRY_H4_PERIOD_MS :
        H456_TELEMETRY_LAP_PERIOD_MS;
    g_score_elapsed_ms = 0U;
    g_maximum_ball_error_milli_cm = 0;
    g_ball_score_passed = false;
    g_session_active = true;
}

ml_status_t h456_telemetry_record(
    const h456_telemetry_sample_t *sample, bool force)
{
    h456_telemetry_record_t packed;
    uint32_t relative_time;

    if (sample == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_session_active) {
        return ML_STATUS_BUSY;
    }
    if (sample->timestamp_ms < g_start_time_ms) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    relative_time = sample->timestamp_ms - g_start_time_ms;
    if (g_record_count > 0U) {
        uint32_t previous_time =
            g_records[g_record_count - 1U].time_ms;

        if (relative_time < previous_time) {
            return ML_STATUS_INVALID_ARGUMENT;
        }
        if (!force && ((relative_time - previous_time) < g_period_ms)) {
            return ML_STATUS_BUSY;
        }
    }
    h456_telemetry_pack(&packed, sample);
    if (g_record_count >= H456_TELEMETRY_CAPACITY) {
        g_buffer_full = true;
        if (!force) {
            return ML_STATUS_BUFFER_FULL;
        }
        g_records[H456_TELEMETRY_CAPACITY - 1U] = packed;
        return ML_STATUS_OK;
    }
    g_records[g_record_count++] = packed;
    if (g_record_count >= H456_TELEMETRY_CAPACITY) {
        g_buffer_full = true;
    }
    return ML_STATUS_OK;
}

void h456_telemetry_session_finish(
    const h456_telemetry_sample_t *sample)
{
    if (g_session_active && (sample != 0)) {
        (void) h456_telemetry_record(sample, true);
    }
    g_session_active = false;
}

void h456_telemetry_set_result(uint32_t score_elapsed_ms,
    float maximum_ball_error_cm, bool ball_score_passed)
{
    g_score_elapsed_ms = score_elapsed_ms;
    g_maximum_ball_error_milli_cm =
        h456_telemetry_i16(maximum_ball_error_cm, 1000.0f);
    g_ball_score_passed = ball_score_passed;
}

bool h456_telemetry_session_active(void)
{
    return g_session_active;
}

uint16_t h456_telemetry_count(void)
{
    return g_record_count;
}

bool h456_telemetry_full(void)
{
    return g_buffer_full;
}

uint32_t h456_telemetry_storage_bytes(void)
{
    return (uint32_t) sizeof(g_records);
}

ml_status_t h456_telemetry_uart0_handle_byte(
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

        if (!g_busy_sent || ((now_ms - g_last_busy_ms) >= 1000U)) {
            g_busy_sent = true;
            g_last_busy_ms = now_ms;
            (void) h456_uart_write(
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

        (void) h456_uart_write(
            empty, (uint16_t) (sizeof(empty) - 1U));
        return ML_STATUS_BUFFER_EMPTY;
    }
    if (g_export_active) {
        return ML_STATUS_BUSY;
    }
    g_export_active = true;
    {
        ml_status_t status = h456_export_csv();

        g_export_active = false;
        return status;
    }
}
