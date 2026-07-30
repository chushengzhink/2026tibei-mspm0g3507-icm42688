#include "chassis_telemetry.h"

#include <float.h>

typedef char chassis_telemetry_record_must_be_52_bytes[
    (sizeof(chassis_telemetry_record_t) == 52U) ? 1 : -1];

typedef struct {
    uint32_t timestamp_ms;
    uint8_t left_total_ticks_s24[3];
    uint8_t right_total_ticks_s24[3];
    int16_t x_quarter_mm;
    int16_t y_quarter_mm;
    uint8_t encoder_heading_cdeg_s24[3];
    uint8_t fused_heading_cdeg_s24[3];
    int16_t imu_yaw_cdeg;
    int16_t fused_yaw_rate_cdps;
    int16_t target_center_dmm_s;
    int16_t actual_center_dmm_s;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    uint8_t fusion_active;
    uint8_t line_bits;
    uint8_t line_state_flags;
    int8_t line_correction_mm_s;
    uint16_t mission_progress_mm;
    uint16_t expected_heading_cdeg;
    int8_t route_feedforward_bias_mm_s;
    int8_t heading_feedback_bias_mm_s;
    uint8_t line_weight_pct;
    int8_t final_steering_bias_mm_s;
} chassis_telemetry_storage_record_t;

typedef char chassis_telemetry_storage_record_must_be_44_bytes[
    (sizeof(chassis_telemetry_storage_record_t) == 44U) ? 1 : -1];

static chassis_telemetry_storage_record_t
    g_records[CHASSIS_TELEMETRY_CAPACITY];
static uint16_t g_record_count;
static bool g_overflowed;
static uint32_t g_session_start_ms;
static uint32_t g_session_total_ms;
static uint8_t g_line_bits;
static uint8_t g_line_state_flags;
static int8_t g_line_correction_mm_s;
static uint16_t g_mission_progress_mm;
static uint16_t g_expected_heading_cdeg;
static int8_t g_route_feedforward_bias_mm_s;
static int8_t g_heading_feedback_bias_mm_s;
static uint8_t g_line_weight_pct;
static int8_t g_final_steering_bias_mm_s;
static bool g_session_started;
static bool g_session_finished;

static bool telemetry_float_valid(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static int32_t telemetry_heading_to_cdeg(float degrees)
{
    if (degrees >= 21474800.0f) {
        return INT32_MAX;
    }
    if (degrees <= -21474800.0f) {
        return INT32_MIN;
    }
    return (int32_t) (degrees * 100.0f);
}

static int16_t telemetry_mm_to_quarter_mm(float value)
{
    if (value >= 8191.75f) {
        return INT16_MAX;
    }
    if (value <= -8192.0f) {
        return INT16_MIN;
    }
    return (int16_t) (value * 4.0f);
}

static void telemetry_int24_encode(uint8_t bytes[3], int32_t value)
{
    uint32_t encoded;

    if (value > 8388607) {
        value = 8388607;
    } else if (value < -8388608) {
        value = -8388608;
    }
    encoded = ((uint32_t) value) & 0x00FFFFFFU;
    bytes[0] = (uint8_t) encoded;
    bytes[1] = (uint8_t) (encoded >> 8U);
    bytes[2] = (uint8_t) (encoded >> 16U);
}

static int32_t telemetry_int24_decode(const uint8_t bytes[3])
{
    uint32_t encoded = (uint32_t) bytes[0] |
        ((uint32_t) bytes[1] << 8U) |
        ((uint32_t) bytes[2] << 16U);

    if ((encoded & 0x00800000U) != 0U) {
        return -(int32_t) (((~encoded) + 1U) & 0x00FFFFFFU);
    }
    return (int32_t) encoded;
}

static void telemetry_expand_record(
    const chassis_telemetry_storage_record_t *stored,
    chassis_telemetry_record_t *record)
{
    record->timestamp_ms = stored->timestamp_ms;
    record->left_total_ticks = telemetry_int24_decode(
        stored->left_total_ticks_s24);
    record->right_total_ticks = telemetry_int24_decode(
        stored->right_total_ticks_s24);
    record->x_mm = (float) stored->x_quarter_mm * 0.25f;
    record->y_mm = (float) stored->y_quarter_mm * 0.25f;
    record->encoder_heading_cdeg = telemetry_int24_decode(
        stored->encoder_heading_cdeg_s24);
    record->fused_heading_cdeg = telemetry_int24_decode(
        stored->fused_heading_cdeg_s24);
    record->imu_yaw_cdeg = stored->imu_yaw_cdeg;
    record->fused_yaw_rate_cdps = stored->fused_yaw_rate_cdps;
    record->target_center_dmm_s = stored->target_center_dmm_s;
    record->actual_center_dmm_s = stored->actual_center_dmm_s;
    record->pwm_left_count = stored->pwm_left_count;
    record->pwm_right_count = stored->pwm_right_count;
    record->fusion_active = stored->fusion_active;
    record->line_bits = stored->line_bits;
    record->line_state_flags = stored->line_state_flags;
    record->line_correction_mm_s = stored->line_correction_mm_s;
    record->mission_progress_mm = stored->mission_progress_mm;
    record->expected_heading_cdeg = stored->expected_heading_cdeg;
    record->route_feedforward_bias_mm_s =
        stored->route_feedforward_bias_mm_s;
    record->heading_feedback_bias_mm_s =
        stored->heading_feedback_bias_mm_s;
    record->line_weight_pct = stored->line_weight_pct;
    record->final_steering_bias_mm_s =
        stored->final_steering_bias_mm_s;
}

static int16_t telemetry_wrapped_degrees_to_cdeg(float degrees)
{
    while (degrees >= 180.0f) {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f) {
        degrees += 360.0f;
    }
    return (int16_t) (degrees * 100.0f);
}

static int16_t telemetry_rate_to_cdps(float rate_dps)
{
    if (rate_dps >= 327.67f) {
        return INT16_MAX;
    }
    if (rate_dps <= -327.68f) {
        return INT16_MIN;
    }
    return (int16_t) (rate_dps * 100.0f);
}

static int16_t telemetry_speed_to_dmm_s(float speed_mm_s)
{
    if (speed_mm_s >= 3276.7f) {
        return INT16_MAX;
    }
    if (speed_mm_s <= -3276.8f) {
        return INT16_MIN;
    }
    return (int16_t) (speed_mm_s * 10.0f);
}

static int8_t telemetry_correction_to_mm_s(float correction_mm_s)
{
    if (correction_mm_s >= 127.0f) {
        return INT8_MAX;
    }
    if (correction_mm_s <= -128.0f) {
        return INT8_MIN;
    }
    return (int8_t) correction_mm_s;
}

static uint16_t telemetry_positive_to_uint16(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }
    if (value >= 65535.0f) {
        return UINT16_MAX;
    }
    return (uint16_t) (value + 0.5f);
}

static uint16_t telemetry_positive_heading_to_cdeg(float degrees)
{
    if (degrees <= 0.0f) {
        return 0U;
    }
    if (degrees >= 655.35f) {
        return UINT16_MAX;
    }
    return (uint16_t) (degrees * 100.0f + 0.5f);
}

static uint8_t telemetry_weight_to_percent(float weight)
{
    if (weight <= 0.0f) {
        return 0U;
    }
    if (weight >= 1.0f) {
        return 100U;
    }
    return (uint8_t) (weight * 100.0f + 0.5f);
}

static uint16_t telemetry_append_char(
    char *buffer, uint16_t index, char character)
{
    buffer[index] = character;
    return (uint16_t) (index + 1U);
}

static uint16_t telemetry_append_uint32(
    char *buffer, uint16_t index, uint32_t value)
{
    char reverse[10];
    uint8_t count = 0U;

    do {
        reverse[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) {
        buffer[index++] = reverse[--count];
    }
    return index;
}

static uint16_t telemetry_append_int32(
    char *buffer, uint16_t index, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        index = telemetry_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return telemetry_append_uint32(buffer, index, magnitude);
}

static uint16_t telemetry_append_fixed(char *buffer, uint16_t index,
    int32_t scaled_value, uint32_t scale, uint8_t digits)
{
    uint32_t magnitude;
    uint32_t divisor;

    if (scaled_value < 0) {
        index = telemetry_append_char(buffer, index, '-');
        magnitude = (uint32_t) (-(scaled_value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) scaled_value;
    }
    index = telemetry_append_uint32(buffer, index, magnitude / scale);
    index = telemetry_append_char(buffer, index, '.');
    magnitude %= scale;
    divisor = scale / 10U;
    while (digits-- != 0U) {
        index = telemetry_append_char(buffer, index,
            (char) ('0' + ((magnitude / divisor) % 10U)));
        divisor /= 10U;
    }
    return index;
}

static int32_t telemetry_mm_to_milli(float value)
{
    if (value >= 2147483.0f) {
        return INT32_MAX;
    }
    if (value <= -2147483.0f) {
        return INT32_MIN;
    }
    return (int32_t) (value * 1000.0f);
}

void chassis_telemetry_init(void)
{
    chassis_telemetry_clear();
}

void chassis_telemetry_clear(void)
{
    g_record_count = 0U;
    g_overflowed = false;
    g_session_start_ms = 0U;
    g_session_total_ms = 0U;
    g_line_bits = 0U;
    g_line_state_flags = 0U;
    g_line_correction_mm_s = 0;
    g_mission_progress_mm = 0U;
    g_expected_heading_cdeg = 0U;
    g_route_feedforward_bias_mm_s = 0;
    g_heading_feedback_bias_mm_s = 0;
    g_line_weight_pct = 0U;
    g_final_steering_bias_mm_s = 0;
    g_session_started = false;
    g_session_finished = false;
}

void chassis_telemetry_session_start(uint32_t start_time_ms)
{
    chassis_telemetry_clear();
    g_session_start_ms = start_time_ms;
    g_session_started = true;
}

void chassis_telemetry_session_finish(uint32_t stop_time_ms)
{
    if (g_session_started) {
        g_session_total_ms = (uint32_t) (stop_time_ms -
            g_session_start_ms);
        g_session_finished = true;
    }
}

void chassis_telemetry_set_line_bits(uint8_t line_bits)
{
    g_line_bits = (uint8_t) (line_bits & 0x0FU);
    g_line_state_flags = 0U;
}

void chassis_telemetry_set_line_state(uint8_t line_bits,
    bool line_usable, bool line_recovering, bool pattern_invalid)
{
    g_line_bits = (uint8_t) (line_bits & 0x0FU);
    g_line_state_flags = 0U;
    if (line_usable) {
        g_line_state_flags |= CHASSIS_TELEMETRY_LINE_USABLE;
    }
    if (line_recovering) {
        g_line_state_flags |= CHASSIS_TELEMETRY_LINE_RECOVERING;
    }
    if (pattern_invalid) {
        g_line_state_flags |= CHASSIS_TELEMETRY_LINE_PATTERN_INVALID;
    }
}

void chassis_telemetry_set_line_correction(float correction_mm_s)
{
    if (telemetry_float_valid(correction_mm_s)) {
        g_line_correction_mm_s =
            telemetry_correction_to_mm_s(correction_mm_s);
    } else {
        g_line_correction_mm_s = 0;
    }
}

void chassis_telemetry_set_track_fusion(float mission_progress_mm,
    float expected_heading_deg, float route_feedforward_bias_mm_s,
    float heading_feedback_bias_mm_s, float line_weight,
    float final_steering_bias_mm_s)
{
    if (!telemetry_float_valid(mission_progress_mm) ||
        !telemetry_float_valid(expected_heading_deg) ||
        !telemetry_float_valid(route_feedforward_bias_mm_s) ||
        !telemetry_float_valid(heading_feedback_bias_mm_s) ||
        !telemetry_float_valid(line_weight) ||
        !telemetry_float_valid(final_steering_bias_mm_s)) {
        g_mission_progress_mm = 0U;
        g_expected_heading_cdeg = 0U;
        g_route_feedforward_bias_mm_s = 0;
        g_heading_feedback_bias_mm_s = 0;
        g_line_weight_pct = 0U;
        g_final_steering_bias_mm_s = 0;
        return;
    }
    g_mission_progress_mm = telemetry_positive_to_uint16(
        mission_progress_mm);
    g_expected_heading_cdeg =
        telemetry_positive_heading_to_cdeg(expected_heading_deg);
    g_route_feedforward_bias_mm_s = telemetry_correction_to_mm_s(
        route_feedforward_bias_mm_s);
    g_heading_feedback_bias_mm_s = telemetry_correction_to_mm_s(
        heading_feedback_bias_mm_s);
    g_line_weight_pct = telemetry_weight_to_percent(line_weight);
    g_final_steering_bias_mm_s = telemetry_correction_to_mm_s(
        final_steering_bias_mm_s);
}

ml_status_t chassis_telemetry_record(uint32_t timestamp_ms,
    int32_t left_total_ticks, int32_t right_total_ticks,
    float x_mm, float y_mm, float encoder_heading_deg,
    float fused_heading_deg, float imu_yaw_deg,
    float fused_yaw_rate_dps, float target_center_mm_s,
    float actual_center_mm_s, uint16_t pwm_left_count,
    uint16_t pwm_right_count, bool fusion_active)
{
    chassis_telemetry_storage_record_t *record;

    if (!telemetry_float_valid(x_mm) || !telemetry_float_valid(y_mm) ||
        !telemetry_float_valid(encoder_heading_deg) ||
        !telemetry_float_valid(fused_heading_deg) ||
        !telemetry_float_valid(imu_yaw_deg) ||
        !telemetry_float_valid(fused_yaw_rate_dps) ||
        !telemetry_float_valid(target_center_mm_s) ||
        !telemetry_float_valid(actual_center_mm_s)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if ((g_record_count != 0U) &&
        (timestamp_ms < g_records[g_record_count - 1U].timestamp_ms)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if ((g_record_count != 0U) &&
        (timestamp_ms == g_records[g_record_count - 1U].timestamp_ms)) {
        record = &g_records[g_record_count - 1U];
    } else if (g_record_count >= CHASSIS_TELEMETRY_CAPACITY) {
        g_overflowed = true;
        return ML_STATUS_BUFFER_FULL;
    } else {
        record = &g_records[g_record_count++];
    }
    record->timestamp_ms = timestamp_ms;
    telemetry_int24_encode(record->left_total_ticks_s24,
        left_total_ticks);
    telemetry_int24_encode(record->right_total_ticks_s24,
        right_total_ticks);
    record->x_quarter_mm = telemetry_mm_to_quarter_mm(x_mm);
    record->y_quarter_mm = telemetry_mm_to_quarter_mm(y_mm);
    telemetry_int24_encode(record->encoder_heading_cdeg_s24,
        telemetry_heading_to_cdeg(encoder_heading_deg));
    telemetry_int24_encode(record->fused_heading_cdeg_s24,
        telemetry_heading_to_cdeg(fused_heading_deg));
    record->imu_yaw_cdeg =
        telemetry_wrapped_degrees_to_cdeg(imu_yaw_deg);
    record->fused_yaw_rate_cdps =
        telemetry_rate_to_cdps(fused_yaw_rate_dps);
    record->target_center_dmm_s =
        telemetry_speed_to_dmm_s(target_center_mm_s);
    record->actual_center_dmm_s =
        telemetry_speed_to_dmm_s(actual_center_mm_s);
    record->pwm_left_count = pwm_left_count;
    record->pwm_right_count = pwm_right_count;
    record->fusion_active = fusion_active ? 1U : 0U;
    record->line_bits = g_line_bits;
    record->line_state_flags = g_line_state_flags;
    record->line_correction_mm_s = g_line_correction_mm_s;
    record->mission_progress_mm = g_mission_progress_mm;
    record->expected_heading_cdeg = g_expected_heading_cdeg;
    record->route_feedforward_bias_mm_s =
        g_route_feedforward_bias_mm_s;
    record->heading_feedback_bias_mm_s =
        g_heading_feedback_bias_mm_s;
    record->line_weight_pct = g_line_weight_pct;
    record->final_steering_bias_mm_s =
        g_final_steering_bias_mm_s;
    return ML_STATUS_OK;
}

uint16_t chassis_telemetry_count(void)
{
    return g_record_count;
}

bool chassis_telemetry_overflowed(void)
{
    return g_overflowed;
}

ml_status_t chassis_telemetry_get(
    uint16_t index, chassis_telemetry_record_t *record)
{
    if (record == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (index >= g_record_count) {
        return ML_STATUS_BUFFER_EMPTY;
    }
    telemetry_expand_record(&g_records[index], record);
    return ML_STATUS_OK;
}

ml_status_t chassis_telemetry_export_csv(
    chassis_telemetry_writer_t writer, void *context)
{
    static const char header[] =
        "time_ms,left_ticks,right_ticks,x_mm,y_mm,encoder_heading_deg,"
        "fused_heading_deg,imu_yaw_deg,fused_yaw_rate_dps,"
        "fusion_active,elapsed_ms,lap_total_ms,target_center_mm_s,"
        "actual_center_mm_s,left_pwm_count,right_pwm_count,line_bits,"
        "line_correction_mm_s,line_usable,line_recovering,"
        "line_pattern_invalid,mission_progress_mm,expected_heading_deg,"
        "route_feedforward_bias_mm_s,heading_feedback_bias_mm_s,"
        "line_weight_pct,final_steering_bias_mm_s\r\n";
    char line[256];
    uint16_t line_length;
    uint16_t index;
    int32_t x_milli;
    int32_t y_milli;
    chassis_telemetry_record_t record;
    ml_status_t status;

    if (writer == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = writer(header, (uint16_t) (sizeof(header) - 1U), context);
    for (index = 0U;
         (index < g_record_count) && (status == ML_STATUS_OK); ++index) {
        telemetry_expand_record(&g_records[index], &record);
        x_milli = telemetry_mm_to_milli(record.x_mm);
        y_milli = telemetry_mm_to_milli(record.y_mm);
        line_length = 0U;
        line_length = telemetry_append_uint32(line, line_length,
            record.timestamp_ms);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.left_total_ticks);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.right_total_ticks);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            x_milli, 1000U, 3U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            y_milli, 1000U, 3U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.encoder_heading_cdeg, 100U, 2U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.fused_heading_cdeg, 100U, 2U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.imu_yaw_cdeg, 100U, 2U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.fused_yaw_rate_cdps, 100U, 2U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.fusion_active);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            g_session_started ? (uint32_t)
                (record.timestamp_ms - g_session_start_ms) :
                record.timestamp_ms);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            g_session_finished ? g_session_total_ms : 0U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.target_center_dmm_s, 10U, 1U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.actual_center_dmm_s, 10U, 1U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.pwm_left_count);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.pwm_right_count);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.line_bits);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.line_correction_mm_s);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            (record.line_state_flags &
             CHASSIS_TELEMETRY_LINE_USABLE) != 0U ? 1U : 0U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            (record.line_state_flags &
             CHASSIS_TELEMETRY_LINE_RECOVERING) != 0U ? 1U : 0U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            (record.line_state_flags &
             CHASSIS_TELEMETRY_LINE_PATTERN_INVALID) != 0U ? 1U : 0U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.mission_progress_mm);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_fixed(line, line_length,
            record.expected_heading_cdeg, 100U, 2U);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.route_feedforward_bias_mm_s);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.heading_feedback_bias_mm_s);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_uint32(line, line_length,
            record.line_weight_pct);
        line_length = telemetry_append_char(line, line_length, ',');
        line_length = telemetry_append_int32(line, line_length,
            record.final_steering_bias_mm_s);
        line_length = telemetry_append_char(line, line_length, '\r');
        line_length = telemetry_append_char(line, line_length, '\n');
        status = writer(line, line_length, context);
    }
    return status;
}
