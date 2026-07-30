#include "ball_balance_app.h"

#include <string.h>

#include "ball_balance.h"
#include "ball_balance_config.h"
#include "ball_demo.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_oled.h"

#define BALL_APP_KEY_DEBOUNCE_MS  (60U)
#define BALL_APP_KEY_LONG_PRESS_MS (1000U)
#define BALL_APP_DISPLAY_PERIOD_MS (100U)

typedef struct {
    bool raw_pressed;
    bool stable_pressed;
    bool armed;
    bool eligible_press;
    bool long_reported;
    bool press_event;
    bool short_release_event;
    uint32_t raw_change_ms;
    uint32_t pressed_since_ms;
} ball_app_key_t;

typedef struct {
    ball_app_key_t center_key;
    bool calibration_mode;
    bool initialized;
    uint32_t last_display_ms;
} ball_balance_app_context_t;

static ball_balance_app_context_t g_app;

static bool ball_app_center_pressed(void)
{
    return gpio_get(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN) ==
        ML_KEY_ACTIVE_LEVEL;
}

static ml_status_t ball_app_keys_init(void)
{
    ml_status_t status;

    status = board_resource_claim(
        ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = gpio_init(ML_KEY_UP_PORT, ML_KEY_UP_PIN,
        (GPIOn_enum) ML_KEY_UP_IOMUX, IN_UP);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_LEFT_PORT, ML_KEY_LEFT_PIN,
            (GPIOn_enum) ML_KEY_LEFT_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_DOWN_PORT, ML_KEY_DOWN_PIN,
            (GPIOn_enum) ML_KEY_DOWN_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_RIGHT_PORT, ML_KEY_RIGHT_PIN,
            (GPIOn_enum) ML_KEY_RIGHT_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
            (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    }
    if (status != ML_STATUS_OK) {
        board_resource_release(
            ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    }
    return status;
}

static void ball_app_key_init(ball_app_key_t *key, bool pressed)
{
    memset(key, 0, sizeof(*key));
    key->raw_pressed = pressed;
    key->stable_pressed = pressed;
    key->armed = !pressed;
}

static void ball_app_key_update(
    ball_app_key_t *key, bool pressed, uint32_t now_ms)
{
    key->press_event = false;
    key->short_release_event = false;
    if (pressed != key->raw_pressed) {
        key->raw_pressed = pressed;
        key->raw_change_ms = now_ms;
    }
    if ((key->stable_pressed != key->raw_pressed) &&
        ((now_ms - key->raw_change_ms) >= BALL_APP_KEY_DEBOUNCE_MS)) {
        key->stable_pressed = key->raw_pressed;
        if (key->stable_pressed) {
            key->pressed_since_ms = now_ms;
            key->long_reported = false;
            key->eligible_press = key->armed;
            if (key->eligible_press) {
                key->press_event = true;
            }
        } else {
            if (key->eligible_press && !key->long_reported) {
                key->short_release_event = true;
            }
            key->eligible_press = false;
            key->long_reported = false;
            key->armed = true;
        }
    }
    if (key->stable_pressed && key->eligible_press &&
        !key->long_reported &&
        ((now_ms - key->pressed_since_ms) >=
         BALL_APP_KEY_LONG_PRESS_MS)) {
        key->long_reported = true;
    }
}

static bool ball_app_take_long_press(ball_app_key_t *key)
{
    if (key->long_reported && key->stable_pressed &&
        key->eligible_press) {
        key->eligible_press = false;
        return true;
    }
    return false;
}

static void ball_app_copy_line(char *line, const char *text)
{
    uint8_t index = 0U;

    while ((index < OLED_TEXT_COLUMN_COUNT) && (text[index] != '\0')) {
        line[index] = text[index];
        ++index;
    }
    while (index < OLED_TEXT_COLUMN_COUNT) {
        line[index++] = ' ';
    }
    line[OLED_TEXT_COLUMN_COUNT] = '\0';
}

static uint32_t ball_app_clamp_u32(uint32_t value, uint32_t maximum)
{
    return (value > maximum) ? maximum : value;
}

static int32_t ball_app_round_tenths(float value)
{
    float scaled = value * 10.0f;

    if (scaled > 999.0f) {
        return 999;
    }
    if (scaled < -999.0f) {
        return -999;
    }
    return (int32_t) (scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void ball_app_write_unsigned(
    char *line, uint8_t offset, uint32_t value, uint8_t digits)
{
    uint8_t index;

    for (index = 0U; index < digits; ++index) {
        line[offset + digits - 1U - index] =
            (char) ('0' + (value % 10U));
        value /= 10U;
    }
}

static void ball_app_write_signed_tenths(
    char *line, uint8_t offset, float value)
{
    int32_t tenths = ball_app_round_tenths(value);
    uint32_t magnitude;

    line[offset] = (tenths < 0) ? '-' : '+';
    magnitude = (uint32_t) ((tenths < 0) ? -tenths : tenths);
    ball_app_write_unsigned(line, (uint8_t) (offset + 1U), magnitude, 3U);
}

static const char *ball_app_state_text(
    const ball_balance_status_t *status)
{
    if (g_app.calibration_mode) {
        if (status->enabled) {
            return "CAL CENTER LOOP";
        }
        if (!status->vision_ready) {
            return "CAL WAIT VISION";
        }
        return "CAL C=STEP/HOLD";
    }
    switch (status->sequence_state) {
        case BALL_SEQUENCE_TO_PLUS_5_CM:
            return "BALL RUN TO +5";
        case BALL_SEQUENCE_TO_MINUS_5_CM:
            return "BALL RUN TO -5";
        case BALL_SEQUENCE_COMPLETE:
            return "BALL HOLD -5";
        case BALL_SEQUENCE_TIMEOUT:
            return "BALL TIMEOUT";
        case BALL_SEQUENCE_ABORTED:
            return "BALL ABORTED";
        case BALL_SEQUENCE_VISION_LOST:
            return "BALL VISION LOST";
        default:
            break;
    }
    if (!status->vision_ready) {
        return "BALL WAIT VISION";
    }
#if BALL_BALANCE_ALLOW_SEQUENCE
    if (status->sequence_started_once) {
        return "BALL SEQ USED";
    }
    return "BALL READY C=GO";
#else
    return "BALL SEQ LOCKED";
#endif
}

static void ball_app_display(const ball_balance_status_t *status)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_COLUMN_COUNT + 1U];
    uint32_t age_ms;
    uint32_t error_count;
    uint8_t line;

    ball_app_copy_line(lines[0], ball_app_state_text(status));

    ball_app_copy_line(lines[1], "T+000 P+000");
    ball_app_write_signed_tenths(lines[1], 1U, status->target_cm);
    ball_app_write_signed_tenths(lines[1], 7U, status->position_cm);

    ball_app_copy_line(lines[2], "V+000 U0000");
    ball_app_write_signed_tenths(lines[2], 1U, status->velocity_cm_per_s);
    ball_app_write_unsigned(lines[2], 7U,
        ball_app_clamp_u32(status->servo_current_us, 9999U), 4U);

    ball_app_copy_line(lines[3], "S0000 A000 E00");
    ball_app_write_unsigned(lines[3], 1U,
        ball_app_clamp_u32(status->sequence_elapsed_ms, 9999U), 4U);
    age_ms = (status->vision_age_ms == 0xFFFFFFFFUL) ?
        999U : ball_app_clamp_u32(status->vision_age_ms, 999U);
    ball_app_write_unsigned(lines[3], 7U, age_ms, 3U);
    error_count = status->crc_errors + status->length_errors +
        status->format_errors + status->observer_outliers +
        status->uart_overflows;
    ball_app_write_unsigned(lines[3], 12U,
        ball_app_clamp_u32(error_count, 99U), 2U);

    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        (void) OLED_ShowLine((uint8_t) (line + 1U), lines[line]);
    }
}

ml_status_t ball_balance_app_init(void)
{
    ml_status_t status;
    bool boot_center_pressed;

    memset(&g_app, 0, sizeof(g_app));
    status = OLED_Init();
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = ball_app_keys_init();
    if (status != ML_STATUS_OK) {
        return status;
    }
    boot_center_pressed = ball_app_center_pressed();
    ball_app_key_init(&g_app.center_key, boot_center_pressed);
    g_app.calibration_mode = boot_center_pressed;

    status = ball_balance_init();
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (g_app.calibration_mode) {
        status = ball_demo_init();
        if (status != ML_STATUS_OK) {
            return status;
        }
    }
    g_app.initialized = true;
    (void) OLED_Clear();
    return ML_STATUS_OK;
}

void ball_balance_app_poll(void)
{
    ball_balance_status_t status;
    bool long_press;

    if (!g_app.initialized) {
        return;
    }
    ball_balance_process();
    if (ball_balance_get_status(&status) != ML_STATUS_OK) {
        return;
    }

    ball_app_key_update(&g_app.center_key,
        ball_app_center_pressed(), status.uptime_ms);
    long_press = ball_app_take_long_press(&g_app.center_key);
    if (g_app.calibration_mode) {
        if (long_press) {
            (void) ball_demo_long_press();
        } else if (g_app.center_key.short_release_event) {
            (void) ball_demo_short_press();
        }
        ball_demo_process();
    } else if (g_app.center_key.press_event) {
        if ((status.sequence_state == BALL_SEQUENCE_TO_PLUS_5_CM) ||
            (status.sequence_state == BALL_SEQUENCE_TO_MINUS_5_CM)) {
            (void) ball_balance_abort_sequence();
        } else if ((status.sequence_state == BALL_SEQUENCE_IDLE) &&
                   status.vision_ready &&
                   !status.sequence_started_once) {
            (void) ball_balance_start_pm5_sequence();
        }
    }

    if (ball_balance_get_status(&status) != ML_STATUS_OK) {
        return;
    }
    if ((status.uptime_ms - g_app.last_display_ms) >=
        BALL_APP_DISPLAY_PERIOD_MS) {
        g_app.last_display_ms = status.uptime_ms;
        ball_app_display(&status);
    }
}
