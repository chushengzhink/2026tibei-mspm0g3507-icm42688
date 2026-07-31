#include "q3_ball_app.h"

#include <string.h>

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"
#include "q3_ball.h"
#include "q3_ball_config.h"
#include "q3_ball_telemetry.h"

#define Q3_APP_KEY_DEBOUNCE_MS       (40U)
#define Q3_OLED_INIT_RETRIES          (3U)
#define Q3_OLED_RETRY_PERIOD_MS       (500U)
#define Q3_OLED_RETRY_DELAY_MS        (50U)
#define Q3_OLED_REDRAW_DELAY_MS       (20U)
#define Q3_OLED_TEST_HOLD_MS          (400U)
#define Q3_OLED_DISPLAY_PERIOD_MS     (100U)
#define Q3_OLED_DIAG_REPEAT_MS        (2000U)
#define Q3_OLED_BLINK_PERIOD_MS       (250U)

typedef struct {
    bool raw_pressed;
    bool stable_pressed;
    bool armed;
    bool press_event;
    uint32_t raw_change_ms;
} q3_key_t;

typedef struct {
    q3_key_t center_key;
    q3_state_t last_state;
    bool initialized;
    bool have_state;
    bool oled_ready;
    bool oled_fault;
    bool oled_dirty;
    bool oled_line_valid[OLED_TEXT_LINE_COUNT];
    char oled_line[OLED_TEXT_LINE_COUNT][OLED_TEXT_COLUMN_COUNT + 1U];
    bool led_ready;
    bool map_exported;
    bool telemetry_finished;
    uint8_t oled_attempts;
    uint32_t oled_error;
    uint32_t oled_last_retry_ms;
    uint32_t oled_last_display_ms;
    uint32_t oled_last_diag_ms;
    uint32_t oled_last_diag_error;
    uint32_t led_last_toggle_ms;
} q3_app_context_t;

static q3_app_context_t g_app;

static void q3_send_text(const char *text)
{
    while (*text != '\0') {
        (void) uart_sendbyte(Q3_TELEMETRY_UART, (uint8_t) *text++);
    }
}

static void q3_send_u32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));
    while (count > 0U) {
        (void) uart_sendbyte(Q3_TELEMETRY_UART,
            (uint8_t) digits[--count]);
    }
}

static void q3_send_fixed(int32_t scaled, uint32_t divisor)
{
    uint32_t magnitude;
    uint32_t place;

    if (scaled < 0) {
        (void) uart_sendbyte(Q3_TELEMETRY_UART, (uint8_t) '-');
        magnitude = (uint32_t) (-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t) scaled;
    }
    q3_send_u32(magnitude / divisor);
    (void) uart_sendbyte(Q3_TELEMETRY_UART, (uint8_t) '.');
    for (place = divisor / 10U; place > 0U; place /= 10U) {
        (void) uart_sendbyte(Q3_TELEMETRY_UART,
            (uint8_t) ('0' + ((magnitude / place) % 10U)));
    }
}

static int32_t q3_scaled(float value, float scale)
{
    float result = value * scale;

    return (int32_t) (result + ((result >= 0.0f) ? 0.5f : -0.5f));
}

static bool q3_center_pressed(void)
{
    return gpio_get(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN) ==
        ML_KEY_ACTIVE_LEVEL;
}

static void q3_key_init(q3_key_t *key, bool pressed)
{
    memset(key, 0, sizeof(*key));
    key->raw_pressed = pressed;
    key->stable_pressed = pressed;
    key->armed = !pressed;
}

static void q3_key_update(q3_key_t *key, bool pressed, uint32_t now_ms)
{
    key->press_event = false;
    if (pressed != key->raw_pressed) {
        key->raw_pressed = pressed;
        key->raw_change_ms = now_ms;
    }
    if ((key->stable_pressed != key->raw_pressed) &&
        ((now_ms - key->raw_change_ms) >= Q3_APP_KEY_DEBOUNCE_MS)) {
        key->stable_pressed = key->raw_pressed;
        if (key->stable_pressed) {
            if (key->armed) {
                key->press_event = true;
            }
            key->armed = false;
        } else {
            key->armed = true;
        }
    }
}

static bool q3_state_is_boot_motion(q3_state_t state)
{
    return (state >= Q3_STATE_BOOT_SETTLE) &&
        (state <= Q3_STATE_BOOT_RECENTER);
}

static bool q3_state_is_formal_motion(q3_state_t state)
{
    return (state >= Q3_STATE_PLUS_DRIVE) &&
        (state <= Q3_STATE_COMPLETE);
}

static bool q3_state_is_map_motion(q3_state_t state)
{
    return (state == Q3_STATE_MAP_TO_PLUS) ||
        (state == Q3_STATE_MAP_TO_MINUS) ||
        (state == Q3_STATE_MAP_RETURN_CENTER);
}

static bool q3_state_is_motion(q3_state_t state)
{
    return q3_state_is_boot_motion(state) ||
        q3_state_is_formal_motion(state) ||
        q3_state_is_map_motion(state);
}

static bool q3_state_is_neutral_terminal(q3_state_t state)
{
    return (state == Q3_STATE_TIMEOUT) ||
        (state == Q3_STATE_ABORTED) ||
        (state == Q3_STATE_VISION_FAULT) ||
        (state == Q3_STATE_PROFILE_FAULT) ||
        (state == Q3_STATE_CALIBRATION_FAULT);
}

static const char *q3_state_uart_text(q3_state_t state)
{
    switch (state) {
        case Q3_STATE_WAIT_VISION: return "Q3 WAIT VISION\r\n";
        case Q3_STATE_BOOT_SETTLE: return "Q3 AUTO CAL SETTLE\r\n";
        case Q3_STATE_BOOT_PROBE_PLUS: return "Q3 AUTO CAL PLUS\r\n";
        case Q3_STATE_BOOT_RETURN_PLUS: return "Q3 AUTO CAL RETURN\r\n";
        case Q3_STATE_BOOT_PROBE_MINUS: return "Q3 AUTO CAL MINUS\r\n";
        case Q3_STATE_BOOT_RECENTER: return "Q3 AUTO RECENTER\r\n";
        case Q3_STATE_READY: return "Q3 READY\r\n";
        case Q3_STATE_PLUS_DRIVE: return "Q3 RUN O TO +5\r\n";
        case Q3_STATE_PLUS_BRAKE: return "Q3 BRAKE +5\r\n";
        case Q3_STATE_REVERSAL: return "Q3 REVERSE\r\n";
        case Q3_STATE_MINUS_DRIVE: return "Q3 RUN +5 TO -5\r\n";
        case Q3_STATE_MINUS_BRAKE: return "Q3 BRAKE -5\r\n";
        case Q3_STATE_FINAL_CAPTURE: return "Q3 CAPTURE -5\r\n";
        case Q3_STATE_COMPLETE: return "Q3 COMPLETE HOLD\r\n";
        case Q3_STATE_MAP_ARMED: return "Q3 MAP ARMED\r\n";
        case Q3_STATE_MAP_TO_PLUS: return "Q3 MAP O TO +6\r\n";
        case Q3_STATE_MAP_TO_MINUS: return "Q3 MAP +6 TO -6\r\n";
        case Q3_STATE_MAP_RETURN_CENTER: return "Q3 MAP RETURN O\r\n";
        case Q3_STATE_MAP_COMPLETE: return "Q3 MAP COMPLETE\r\n";
        case Q3_STATE_TIMEOUT: return "Q3 TIMEOUT\r\n";
        case Q3_STATE_ABORTED: return "Q3 ABORTED\r\n";
        case Q3_STATE_VISION_FAULT: return "Q3 VISION FAULT\r\n";
        case Q3_STATE_PROFILE_FAULT: return "Q3 PROFILE FAULT\r\n";
        case Q3_STATE_CALIBRATION_FAULT: return "Q3 CAL FAULT\r\n";
        default: return "Q3 UNKNOWN\r\n";
    }
}

static const char *q3_oled_header(q3_state_t state)
{
    switch (state) {
        case Q3_STATE_WAIT_VISION: return "Q3 PLACE BALL O";
        case Q3_STATE_BOOT_SETTLE:
        case Q3_STATE_BOOT_PROBE_PLUS:
        case Q3_STATE_BOOT_RETURN_PLUS:
        case Q3_STATE_BOOT_PROBE_MINUS: return "Q3 AUTO CAL";
        case Q3_STATE_BOOT_RECENTER: return "Q3 AUTO CENTER";
        case Q3_STATE_READY: return "Q3 READY C=GO";
        case Q3_STATE_PLUS_DRIVE: return "RUN O TO +5";
        case Q3_STATE_PLUS_BRAKE: return "RUN BRAKE +5";
        case Q3_STATE_REVERSAL: return "RUN REVERSAL";
        case Q3_STATE_MINUS_DRIVE: return "RUN +5 TO -5";
        case Q3_STATE_MINUS_BRAKE: return "RUN BRAKE -5";
        case Q3_STATE_FINAL_CAPTURE: return "RUN HOLD -5";
        case Q3_STATE_COMPLETE: return "Q3 PASS HOLD";
        case Q3_STATE_MAP_ARMED: return "MAP CAL ARMED";
        case Q3_STATE_MAP_TO_PLUS: return "MAP O TO +6";
        case Q3_STATE_MAP_TO_MINUS: return "MAP +6 TO -6";
        case Q3_STATE_MAP_RETURN_CENTER: return "MAP RETURN O";
        case Q3_STATE_MAP_COMPLETE: return "MAP COMPLETE";
        case Q3_STATE_TIMEOUT: return "Q3 TIMEOUT";
        case Q3_STATE_ABORTED: return "Q3 STOPPED";
        case Q3_STATE_VISION_FAULT: return "Q3 VISION FAULT";
        case Q3_STATE_PROFILE_FAULT: return "Q3 PROFILE BAD";
        case Q3_STATE_CALIBRATION_FAULT: return "Q3 CAL FAULT";
        default: return "Q3 STATE";
    }
}

static void q3_line_clear(char line[OLED_TEXT_COLUMN_COUNT + 1U])
{
    memset(line, ' ', OLED_TEXT_COLUMN_COUNT);
    line[OLED_TEXT_COLUMN_COUNT] = '\0';
}

static void q3_line_text(char line[OLED_TEXT_COLUMN_COUNT + 1U],
    uint8_t column, const char *text)
{
    while ((*text != '\0') && (column < OLED_TEXT_COLUMN_COUNT)) {
        line[column++] = *text++;
    }
}

static void q3_line_u32(char line[OLED_TEXT_COLUMN_COUNT + 1U],
    uint8_t column, uint32_t value, uint8_t width)
{
    uint8_t index;

    for (index = width; index > 0U; --index) {
        if ((column + index - 1U) < OLED_TEXT_COLUMN_COUNT) {
            line[column + index - 1U] = (char) ('0' + (value % 10U));
        }
        value /= 10U;
    }
}

static void q3_line_signed_tenths(
    char line[OLED_TEXT_COLUMN_COUNT + 1U], uint8_t column,
    float value)
{
    int32_t scaled = q3_scaled(value, 10.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        line[column] = '-';
        magnitude = (uint32_t) (-scaled);
    } else {
        line[column] = '+';
        magnitude = (uint32_t) scaled;
    }
    line[column + 1U] = (char) ('0' + ((magnitude / 100U) % 10U));
    line[column + 2U] = (char) ('0' + ((magnitude / 10U) % 10U));
    line[column + 3U] = '.';
    line[column + 4U] = (char) ('0' + (magnitude % 10U));
}

static void q3_render_page(const q3_ball_status_t *status,
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_COLUMN_COUNT + 1U])
{
    uint8_t line;

    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        q3_line_clear(lines[line]);
    }
    q3_line_text(lines[0], 0U, q3_oled_header(status->state));
    q3_line_text(lines[1], 0U, "X");
    q3_line_signed_tenths(lines[1], 1U, status->position_cm);
    q3_line_text(lines[1], 7U, "V");
    q3_line_signed_tenths(lines[1], 8U, status->velocity_cm_per_s);

    if (status->state == Q3_STATE_MAP_COMPLETE) {
        q3_line_text(lines[2], 0U, "MAP 13 PT READY");
        q3_line_text(lines[3], 0U, "L=EXPORT MAP");
    } else if (q3_state_is_formal_motion(status->state) ||
               q3_state_is_neutral_terminal(status->state)) {
        q3_line_text(lines[2], 0U, "T");
        q3_line_u32(lines[2], 1U,
            status->sequence_elapsed_ms > 9999U ? 9999U :
            status->sequence_elapsed_ms, 4U);
        q3_line_text(lines[2], 6U, "R");
        q3_line_u32(lines[2], 7U, status->rescue_attempts, 1U);
        q3_line_text(lines[2], 9U, "P");
        q3_line_u32(lines[2], 10U, status->profile_index, 2U);
        if (status->state == Q3_STATE_COMPLETE) {
            q3_line_text(lines[3], 0U, "PB24 STOP HOLD");
        } else if (q3_state_is_neutral_terminal(status->state)) {
            q3_line_text(lines[3], 0U, "CSV N");
            q3_line_u32(lines[3], 5U, q3_telemetry_count(), 3U);
            q3_line_text(lines[3], 9U, "D/C");
        } else {
            q3_line_text(lines[3], 0U, "PB24 E-STOP");
        }
    } else {
        q3_line_text(lines[2], 0U, "N");
        q3_line_u32(lines[2], 1U, (uint32_t) status->neutral_us, 4U);
        q3_line_text(lines[2], 6U, "S");
        q3_line_u32(lines[2], 7U,
            (uint32_t) (status->response_scale * 100.0f), 3U);
        q3_line_text(lines[2], 11U, "P");
        q3_line_u32(lines[2], 12U, status->profile_index, 2U);
        if (status->state == Q3_STATE_READY) {
            q3_line_text(lines[3], 0U, "C=GO K=MAP W=WF");
        } else if (status->state == Q3_STATE_MAP_ARMED) {
            q3_line_text(lines[3], 0U, "PB24 START MAP");
        } else if (q3_state_is_boot_motion(status->state)) {
            q3_line_text(lines[3], 0U, "PB24 E-STOP");
        } else {
            q3_line_text(lines[3], 0U, "PLACE BALL AT O");
        }
    }
}

static void q3_oled_invalidate(void)
{
    memset(g_app.oled_line_valid, 0, sizeof(g_app.oled_line_valid));
    g_app.oled_dirty = true;
}

static ml_status_t q3_oled_write_line(uint8_t line, const char *text)
{
    ml_status_t result;

    if (g_app.oled_line_valid[line] &&
        (memcmp(g_app.oled_line[line], text,
            OLED_TEXT_COLUMN_COUNT + 1U) == 0)) {
        return ML_STATUS_OK;
    }
    result = OLED_ShowLine((uint8_t) (line + 1U), text);
    if (result != ML_STATUS_OK) {
        g_app.oled_ready = false;
        g_app.oled_fault = true;
        g_app.oled_error = (uint32_t) result;
        q3_oled_invalidate();
        return result;
    }
    memcpy(g_app.oled_line[line], text,
        OLED_TEXT_COLUMN_COUNT + 1U);
    g_app.oled_line_valid[line] = true;
    return ML_STATUS_OK;
}

static ml_status_t q3_oled_show_page(const q3_ball_status_t *status,
    bool header_only)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_COLUMN_COUNT + 1U];
    uint8_t line;

    if (!g_app.oled_ready) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    q3_render_page(status, lines);
    if (header_only) {
        return q3_oled_write_line(0U, lines[0]);
    }
    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        if (q3_oled_write_line(line, lines[line]) != ML_STATUS_OK) {
            return (ml_status_t) g_app.oled_error;
        }
    }
    g_app.oled_dirty = false;
    g_app.oled_last_display_ms = status->uptime_ms;
    return ML_STATUS_OK;
}

static ml_status_t q3_oled_show_boot(void)
{
    static const char *const line[OLED_TEXT_LINE_COUNT] = {
        "Q3 TERRAIN CTRL", "OLED PB2/PB3", "SERVO 1300-1700",
        "WAIT CORE"
    };
    uint8_t index;

    q3_oled_invalidate();
    for (index = 0U; index < OLED_TEXT_LINE_COUNT; ++index) {
        if (q3_oled_write_line(index, line[index]) != ML_STATUS_OK) {
            return (ml_status_t) g_app.oled_error;
        }
    }
    return ML_STATUS_OK;
}

static void q3_oled_log_failure(uint32_t now_ms, bool force)
{
    if (!force && (g_app.oled_last_diag_error == g_app.oled_error) &&
        ((now_ms - g_app.oled_last_diag_ms) < Q3_OLED_DIAG_REPEAT_MS)) {
        return;
    }
    q3_send_text("Q3 OLED FAIL ERR=");
    q3_send_u32(g_app.oled_error);
    q3_send_text(" TRY=");
    q3_send_u32(g_app.oled_attempts);
    q3_send_text("\r\n");
    g_app.oled_last_diag_error = g_app.oled_error;
    g_app.oled_last_diag_ms = now_ms;
}

static ml_status_t q3_oled_init_once(uint32_t now_ms, bool background)
{
    ml_status_t result;

    ++g_app.oled_attempts;
    result = OLED_Init();
    if (result == ML_STATUS_OK) {
        g_app.oled_ready = true;
        g_app.oled_fault = false;
        g_app.oled_error = 0U;
        result = q3_oled_show_boot();
        if (result == ML_STATUS_OK) {
            /* Force a second complete write after the panel has settled.
             * The cache is intentionally invalidated by q3_oled_show_boot(),
             * so all eight text pages are transmitted again. */
            delay_ms(Q3_OLED_REDRAW_DELAY_MS);
            result = q3_oled_show_boot();
        }
        if (result == ML_STATUS_OK) {
            q3_send_text(background ? "Q3 OLED RECOVER OK\r\n" :
                "Q3 OLED OK\r\n");
            return ML_STATUS_OK;
        }
    }
    g_app.oled_ready = false;
    g_app.oled_fault = true;
    g_app.oled_error = (uint32_t) result;
    q3_oled_log_failure(now_ms, !background);
    return result;
}

static void q3_oled_initialize(void)
{
    uint8_t attempt;

    delay_ms(Q3_OLED_POWER_SETTLE_MS);
    for (attempt = 0U; attempt < Q3_OLED_INIT_RETRIES; ++attempt) {
        if (q3_oled_init_once(0U, false) == ML_STATUS_OK) {
            return;
        }
        if ((attempt + 1U) < Q3_OLED_INIT_RETRIES) {
            delay_ms(Q3_OLED_RETRY_DELAY_MS);
        }
    }
}

static void q3_oled_service(const q3_ball_status_t *status)
{
    bool motion = q3_state_is_motion(status->state);

    if (!g_app.oled_ready && !motion &&
        ((status->uptime_ms - g_app.oled_last_retry_ms) >=
         Q3_OLED_RETRY_PERIOD_MS)) {
        g_app.oled_last_retry_ms = status->uptime_ms;
        (void) q3_oled_init_once(status->uptime_ms, true);
        q3_oled_invalidate();
    }
    if (g_app.oled_ready && !motion &&
        (g_app.oled_dirty ||
         ((status->uptime_ms - g_app.oled_last_display_ms) >=
          Q3_OLED_DISPLAY_PERIOD_MS))) {
        (void) q3_oled_show_page(status, false);
    }
}

static void q3_led_service(const q3_ball_status_t *status)
{
    if (!g_app.led_ready) {
        return;
    }
    if (g_app.oled_fault) {
        if ((status->uptime_ms - g_app.led_last_toggle_ms) >=
            Q3_OLED_BLINK_PERIOD_MS) {
            g_app.led_last_toggle_ms = status->uptime_ms;
            board_led_toggle();
        }
    } else {
        board_led_off();
    }
}

static void q3_report_state(const q3_ball_status_t *status)
{
    if (!g_app.have_state || (g_app.last_state != status->state)) {
        q3_send_text(q3_state_uart_text(status->state));
        g_app.last_state = status->state;
        g_app.have_state = true;
        g_app.oled_dirty = true;
        if (g_app.oled_ready && q3_state_is_motion(status->state)) {
            (void) q3_oled_show_page(status, true);
        }
    }
}

static void q3_export_map(void)
{
    q3_calibration_point_t point;
    uint8_t index;

    q3_send_text("Q3_MAP_BEGIN\r\n");
    q3_send_text("position_cm,balance_us,roll_plus_us,roll_minus_us,"
        "break_plus_us,break_minus_us,accel_plus_cm_s2,"
        "accel_minus_cm_s2,valid_mask\r\n");
    for (index = 0U; index < q3_ball_calibration_count(); ++index) {
        if (q3_ball_get_calibration_point(index, &point) !=
            ML_STATUS_OK) {
            continue;
        }
        q3_send_fixed(q3_scaled(point.position_cm, 1000.0f), 1000U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.balance_command_us, 10.0f), 10U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.rolling_plus_us, 10.0f), 10U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.rolling_minus_us, 10.0f), 10U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.breakaway_plus_us, 10.0f), 10U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.breakaway_minus_us, 10.0f), 10U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.acceleration_plus_cm_s2, 100.0f),
            100U);
        q3_send_text(",");
        q3_send_fixed(q3_scaled(point.acceleration_minus_cm_s2, 100.0f),
            100U);
        q3_send_text(",");
        q3_send_u32(point.valid_mask);
        q3_send_text("\r\n");
    }
    q3_send_text("Q3_MAP_END\r\n");
}

static bool q3_handle_idle_command(uint8_t byte,
    const q3_ball_status_t *status)
{
    if ((byte == (uint8_t) 'K') || (byte == (uint8_t) 'k')) {
        if (q3_ball_arm_map_calibration() == ML_STATUS_OK) {
            q3_send_text("MAP ARMED PRESS PB24\r\n");
        } else {
            q3_send_text("MAP BUSY\r\n");
        }
        return true;
    }
    if ((byte == (uint8_t) 'L') || (byte == (uint8_t) 'l')) {
        if (status->state == Q3_STATE_MAP_COMPLETE) {
            q3_export_map();
        } else {
            q3_send_text("MAP NOT READY\r\n");
        }
        return true;
    }
    if ((byte == (uint8_t) 'A') || (byte == (uint8_t) 'a')) {
        if (q3_ball_set_mode(Q3_MODE_PROFILE) == ML_STATUS_OK) {
            q3_send_text("MODE PROFILE\r\n");
        }
        return true;
    }
    if ((byte == (uint8_t) 'W') || (byte == (uint8_t) 'w')) {
        if (q3_ball_set_mode(Q3_MODE_WAVEFORM) == ML_STATUS_OK) {
            q3_send_text("MODE WAVEFORM\r\n");
        }
        return true;
    }
    return false;
}

ml_status_t q3_ball_app_init(void)
{
    ml_status_t result;

    memset(&g_app, 0, sizeof(g_app));
    result = board_resource_claim(
        ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    if (result != ML_STATUS_OK) {
        return result;
    }
    result = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
        (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    if (result != ML_STATUS_OK) {
        board_resource_release(ML_BOARD_RESOURCE_PB24,
            ML_BOARD_OWNER_KEY);
        return result;
    }
    q3_key_init(&g_app.center_key, q3_center_pressed());
    g_app.led_ready = (board_led_init() == ML_STATUS_OK);
    result = q3_telemetry_init();
    if (result != ML_STATUS_OK) {
        return result;
    }
    q3_send_text("Q3 TERRAIN BOOT\r\n");
#ifndef TEST_Q3_APP_PRELUDE_H
    NVIC_DisableIRQ(UART0_INT_IRQn);
#endif
    q3_oled_initialize();
    if (g_app.oled_ready) {
        delay_ms(Q3_OLED_TEST_HOLD_MS);
    }
#ifndef TEST_Q3_APP_PRELUDE_H
    NVIC_ClearPendingIRQ(UART0_INT_IRQn);
    NVIC_EnableIRQ(UART0_INT_IRQn);
#endif
    result = q3_ball_init();
    if (result != ML_STATUS_OK) {
        q3_send_text("Q3 CORE FAIL\r\n");
        return result;
    }
    q3_send_text("Q3 CORE OK\r\n");
    g_app.initialized = true;
    return ML_STATUS_OK;
}

void q3_ball_app_poll(void)
{
    q3_ball_status_t status;
    q3_ball_status_t preview;
    uint8_t byte;

    if (!g_app.initialized) {
        return;
    }
    q3_ball_process();
    if (q3_ball_get_status(&status) != ML_STATUS_OK) {
        return;
    }
    q3_key_update(&g_app.center_key, q3_center_pressed(),
        status.uptime_ms);
    if (g_app.center_key.press_event) {
        if (q3_state_is_motion(status.state) ||
            (status.state == Q3_STATE_COMPLETE)) {
            (void) q3_ball_abort();
        } else if (status.state == Q3_STATE_READY) {
            preview = status;
            preview.state = Q3_STATE_PLUS_DRIVE;
            if (g_app.oled_ready) {
                (void) q3_oled_show_page(&preview, false);
            }
            (void) q3_ball_start();
        } else if (status.state == Q3_STATE_MAP_ARMED) {
            preview = status;
            preview.state = Q3_STATE_MAP_TO_PLUS;
            if (g_app.oled_ready) {
                (void) q3_oled_show_page(&preview, false);
            }
            (void) q3_ball_start_map_calibration();
        }
        (void) q3_ball_get_status(&status);
    }

    q3_report_state(&status);
    q3_oled_service(&status);
    q3_led_service(&status);

    if ((status.state == Q3_STATE_PLUS_DRIVE) &&
        !q3_telemetry_session_active() && !g_app.telemetry_finished) {
        q3_telemetry_session_start();
    }
    if (q3_telemetry_session_active()) {
        (void) q3_telemetry_record(&status);
        if ((status.state == Q3_STATE_COMPLETE) ||
            q3_state_is_neutral_terminal(status.state)) {
            q3_telemetry_session_finish(&status);
            g_app.telemetry_finished = true;
            g_app.oled_dirty = true;
        }
    }
    if ((status.state == Q3_STATE_MAP_COMPLETE) &&
        !g_app.map_exported) {
        q3_export_map();
        g_app.map_exported = true;
        g_app.oled_dirty = true;
    }

    while (uart_try_read(Q3_TELEMETRY_UART, &byte) == ML_STATUS_OK) {
        bool export_allowed;

        if (q3_handle_idle_command(byte, &status)) {
            (void) q3_ball_get_status(&status);
            continue;
        }
        export_allowed = q3_state_is_neutral_terminal(status.state) &&
            !q3_telemetry_session_active() && status.servo_settled;
        (void) q3_telemetry_uart0_handle_byte(byte,
            export_allowed, status.uptime_ms);
    }
}
