#include "chassis_self_test.h"

#include <string.h>

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_key.h"
#include "chassis_self_test_view.h"
#include "chassis_telemetry.h"
#include "chassis_telemetry_uart.h"
#include "icm42688_service.h"
#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"

#define SELF_TEST_UART_BAUD            (115200UL)
#define SELF_TEST_LIFT_SPEED_MM_S      (60.0f)
#define SELF_TEST_LIFT_BOTH_MM_S       (80.0f)
#define SELF_TEST_LIFT_STAGE_MS        (1000U)
#define SELF_TEST_MIN_STAGE_TICKS      (20)
#define SELF_TEST_GROUND_SPEED_MM_S    (100.0f)
#define SELF_TEST_TURN_SPEED_DEG_S     (60.0f)
#define SELF_TEST_DISPLAY_PERIOD_MS    (100U)
#define SELF_TEST_IMU_ERROR_TIMEOUT_MS (500U)
#define SELF_TEST_IMU_NOTICE_MS        (500U)
#define SELF_TEST_IMU_TIMING_LIMIT     (3U)
#define SELF_TEST_UART_DIAG_PERIOD_MS  (1000U)

typedef enum {
    SELF_TEST_UNINITIALIZED = 0,
    SELF_TEST_CALIBRATING,
    SELF_TEST_MAP_SW6,
    SELF_TEST_UART_DIAGNOSTIC,
    SELF_TEST_ROLL_CALIBRATION,
    SELF_TEST_WAIT_LIFT,
    SELF_TEST_FUSION_CAPTURE,
    SELF_TEST_LEFT_FORWARD,
    SELF_TEST_LEFT_REVERSE,
    SELF_TEST_RIGHT_FORWARD,
    SELF_TEST_RIGHT_REVERSE,
    SELF_TEST_BOTH_FORWARD,
    SELF_TEST_WAIT_GROUND,
    SELF_TEST_RUN_500MM,
    SELF_TEST_WAIT_POSITIVE_90,
    SELF_TEST_RUN_POSITIVE_90,
    SELF_TEST_WAIT_NEGATIVE_90,
    SELF_TEST_RUN_NEGATIVE_90,
    SELF_TEST_COMPLETE,
    SELF_TEST_EMERGENCY,
    SELF_TEST_ERROR
} self_test_state_t;

typedef enum {
    SELF_TEST_IMU_NOTICE_NONE = 0,
    SELF_TEST_IMU_NOTICE_MOVE,
    SELF_TEST_IMU_NOTICE_TIMING
} self_test_imu_notice_t;

typedef struct {
    icm42688_service_t imu_service;
    icm42688_service_output_t imu_output;
    self_test_state_t state;
    uint32_t stage_start_ms;
    uint32_t last_display_ms;
    uint32_t imu_error_start_ms;
    uint32_t imu_notice_until_ms;
    int32_t stage_start_left_ticks;
    int32_t stage_start_right_ticks;
    int32_t roll_start_left_ticks;
    int32_t roll_start_right_ticks;
    uint32_t roll_start_left_bad;
    uint32_t roll_start_right_bad;
    uint32_t uart_diag_next_banner_ms;
    uint32_t uart_diag_banner_count;
    uint8_t imu_timing_reset_streak;
    uint8_t uart_diag_last_byte;
    uint8_t uart_diag_last_errors;
    self_test_imu_notice_t imu_notice;
    chassis_key_t center_key;
    bool imu_error_active;
    bool uart_diag_rx_seen;
    bool display_line_valid[OLED_TEXT_LINE_COUNT];
    char display_line[OLED_TEXT_LINE_COUNT]
        [CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];
    ml_status_t error_status;
    bool initialized;
} self_test_context_t;

static const icm42688_service_config_t g_self_test_imu_config = {
    {
        {
            IMU_ATTITUDE_AXIS_X,
            IMU_ATTITUDE_AXIS_Y,
            IMU_ATTITUDE_AXIS_Z
        },
        {-1, 1, -1},
        0U
    },
    TIMG8,
    1U
};

static self_test_context_t g_self_test;

static const char *self_test_state_text(self_test_state_t state)
{
    switch (state) {
        case SELF_TEST_CALIBRATING:
            return "KEEP STILL      ";
        case SELF_TEST_MAP_SW6:
            return "MAP SW6 INPUTS  ";
        case SELF_TEST_UART_DIAGNOSTIC:
            return "UART DIAG STOP  ";
        case SELF_TEST_ROLL_CALIBRATION:
            return "ROLL C=ZERO     ";
        case SELF_TEST_WAIT_LIFT:
            return "LIFT+PRESS KEY  ";
        case SELF_TEST_FUSION_CAPTURE:
            return "FUSION F/S STOP";
        case SELF_TEST_LEFT_FORWARD:
            return "LEFT FORWARD    ";
        case SELF_TEST_LEFT_REVERSE:
            return "LEFT REVERSE    ";
        case SELF_TEST_RIGHT_FORWARD:
            return "RIGHT FORWARD   ";
        case SELF_TEST_RIGHT_REVERSE:
            return "RIGHT REVERSE   ";
        case SELF_TEST_BOTH_FORWARD:
            return "BOTH CLOSED LOOP";
        case SELF_TEST_WAIT_GROUND:
            return "GROUND+PRESS KEY";
        case SELF_TEST_RUN_500MM:
            return "RUN 500MM       ";
        case SELF_TEST_WAIT_POSITIVE_90:
            return "PRESS FOR +90   ";
        case SELF_TEST_RUN_POSITIVE_90:
            return "ROTATE +90      ";
        case SELF_TEST_WAIT_NEGATIVE_90:
            return "PRESS FOR -90   ";
        case SELF_TEST_RUN_NEGATIVE_90:
            return "ROTATE -90      ";
        case SELF_TEST_COMPLETE:
            return "TEST COMPLETE   ";
        case SELF_TEST_EMERGENCY:
            return "EMERGENCY STOP  ";
        case SELF_TEST_ERROR:
            return "SELF TEST ERROR ";
        default:
            return "INITIALIZING    ";
    }
}

static bool self_test_motion_running(self_test_state_t state)
{
    return ((state >= SELF_TEST_LEFT_FORWARD) &&
        (state <= SELF_TEST_BOTH_FORWARD)) ||
        (state == SELF_TEST_RUN_500MM) ||
        (state == SELF_TEST_RUN_POSITIVE_90) ||
        (state == SELF_TEST_RUN_NEGATIVE_90);
}

static bool self_test_center_pressed(void)
{
    return gpio_get(ML_KEY_CENTER_PORT,
        ML_KEY_CENTER_PIN) == ML_KEY_ACTIVE_LEVEL;
}

static bool self_test_right_pressed(void)
{
    return gpio_get(ML_KEY_RIGHT_PORT,
        ML_KEY_RIGHT_PIN) == ML_KEY_ACTIVE_LEVEL;
}

static ml_status_t self_test_keys_init(void)
{
    ml_status_t status;

    status = board_resource_claim(
        ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_SW6_CHANNEL_1_PORT,
            ML_SW6_CHANNEL_1_PIN,
            (GPIOn_enum) ML_SW6_CHANNEL_1_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_SW6_CHANNEL_2_PORT,
            ML_SW6_CHANNEL_2_PIN,
            (GPIOn_enum) ML_SW6_CHANNEL_2_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_SW6_CHANNEL_3_PORT,
            ML_SW6_CHANNEL_3_PIN,
            (GPIOn_enum) ML_SW6_CHANNEL_3_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_SW6_CHANNEL_4_PORT,
            ML_SW6_CHANNEL_4_PIN,
            (GPIOn_enum) ML_SW6_CHANNEL_4_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_SW6_CHANNEL_5_PORT,
            ML_SW6_CHANNEL_5_PIN,
            (GPIOn_enum) ML_SW6_CHANNEL_5_IOMUX, IN_UP);
    }
    if (status != ML_STATUS_OK) {
        board_resource_release(
            ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    }
    return status;
}

static void self_test_set_line(char line[], const char *text)
{
    uint8_t index = 0U;

    while ((index < CHASSIS_SELF_TEST_VIEW_COLUMNS) &&
        (text[index] != '\0')) {
        line[index] = text[index];
        ++index;
    }
    while (index < CHASSIS_SELF_TEST_VIEW_COLUMNS) {
        line[index] = ' ';
        ++index;
    }
    line[CHASSIS_SELF_TEST_VIEW_COLUMNS] = '\0';
}

static bool self_test_notice_active(uint32_t now_ms)
{
    return (int32_t) (g_self_test.imu_notice_until_ms - now_ms) > 0;
}

static uint32_t self_test_scale_norm(float value, float scale)
{
    float scaled;

    if ((value != value) || (value <= 0.0f)) {
        return 0U;
    }
    scaled = value * scale + 0.5f;
    return (scaled >= 999.0f) ? 999U : (uint32_t) scaled;
}

static void self_test_show_line(uint8_t line, const char text[])
{
    uint8_t index = line - 1U;

    if (!g_self_test.display_line_valid[index] ||
        (memcmp(g_self_test.display_line[index], text,
            CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U) != 0)) {
        if (OLED_ShowLine(line, text) == ML_STATUS_OK) {
            memcpy(g_self_test.display_line[index], text,
                CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U);
            g_self_test.display_line_valid[index] = true;
        }
    }
}

static void self_test_show_roll_calibration(const chassis_status_t *status)
{
    char lines[OLED_TEXT_LINE_COUNT]
        [CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];
    uint8_t line;

    self_test_set_line(lines[0], "ROLL C=ZERO");
    chassis_self_test_view_tick('L', status->encoder_total_left -
        g_self_test.roll_start_left_ticks, lines[1]);
    chassis_self_test_view_tick('R', status->encoder_total_right -
        g_self_test.roll_start_right_ticks, lines[2]);
    chassis_self_test_view_bad(status->encoder_invalid_left -
        g_self_test.roll_start_left_bad,
        status->encoder_invalid_right -
        g_self_test.roll_start_right_bad, lines[3]);
    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        self_test_show_line((uint8_t) (line + 1U), lines[line]);
    }
}

static void self_test_show_uart_diagnostic(void)
{
    uart_diagnostics_t diagnostics;
    char lines[OLED_TEXT_LINE_COUNT]
        [CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];
    uint8_t line;

    memset(&diagnostics, 0, sizeof(diagnostics));
    (void) uart_get_diagnostics(UART0, &diagnostics);
    self_test_set_line(lines[0], "UART DIAG STOP");
    chassis_self_test_view_uart_divisor(
        diagnostics.integer_divisor,
        diagnostics.fractional_divisor, lines[1]);
    chassis_self_test_view_uart_tx(
        g_self_test.uart_diag_banner_count,
        diagnostics.tx_timeouts, lines[2]);
    chassis_self_test_view_uart_rx(
        g_self_test.uart_diag_rx_seen,
        g_self_test.uart_diag_last_byte,
        g_self_test.uart_diag_last_errors,
        diagnostics.rx_queue_overflows, lines[3]);
    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        self_test_show_line((uint8_t) (line + 1U), lines[line]);
    }
}

static void self_test_show_fusion_capture(
    const chassis_status_t *status)
{
    char lines[OLED_TEXT_LINE_COUNT]
        [CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];
    uint8_t line;

    self_test_set_line(lines[0], "FUSION F/S STOP");
    chassis_self_test_view_fusion_heading(
        status->encoder_heading_deg,
        status->fused_heading_deg, lines[1]);
    chassis_self_test_view_fusion_imu(
        status->imu_yaw_deg,
        status->fused_yaw_rate_dps, lines[2]);
    chassis_self_test_view_fusion_status(
        status->heading_fusion_active,
        chassis_telemetry_count(), lines[3]);
    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        self_test_show_line((uint8_t) (line + 1U), lines[line]);
    }
}

static void self_test_show(const chassis_status_t *status)
{
    uint32_t left_state = 0U;
    uint32_t right_state = 0U;
    uint32_t now_ms = g_self_test.imu_output.timestamp_ms;
    ml_encoder_diagnostics_t diagnostics;
    icm42688_service_state_t imu_state;
    char lines[OLED_TEXT_LINE_COUNT]
        [CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U];
    uint8_t line;

    if (g_self_test.state == SELF_TEST_FUSION_CAPTURE) {
        self_test_show_fusion_capture(status);
        return;
    }
    if (g_self_test.state == SELF_TEST_ROLL_CALIBRATION) {
        self_test_show_roll_calibration(status);
        return;
    }
    if (g_self_test.state == SELF_TEST_UART_DIAGNOSTIC) {
        self_test_show_uart_diagnostic();
        return;
    }
    if (ml_encoder_get_diagnostics(&diagnostics) == ML_STATUS_OK) {
        left_state = diagnostics.live_state_a;
        right_state = diagnostics.live_state_b;
    }
    self_test_set_line(lines[0], self_test_state_text(g_self_test.state));
    chassis_self_test_view_encoder(
        (uint8_t) left_state, (uint8_t) right_state, lines[1]);
    chassis_self_test_view_bad(status->encoder_invalid_left,
        status->encoder_invalid_right, lines[2]);

    imu_state = icm42688_service_get_state(&g_self_test.imu_service);
    if (g_self_test.state == SELF_TEST_ERROR) {
        chassis_self_test_view_status(g_self_test.error_status, lines[3]);
    } else if (g_self_test.state == SELF_TEST_MAP_SW6) {
        chassis_self_test_view_sw6(
            gpio_get(ML_SW6_CHANNEL_1_PORT, ML_SW6_CHANNEL_1_PIN),
            gpio_get(ML_SW6_CHANNEL_2_PORT, ML_SW6_CHANNEL_2_PIN),
            gpio_get(ML_SW6_CHANNEL_3_PORT, ML_SW6_CHANNEL_3_PIN),
            gpio_get(ML_SW6_CHANNEL_4_PORT, ML_SW6_CHANNEL_4_PIN),
            gpio_get(ML_SW6_CHANNEL_5_PORT, ML_SW6_CHANNEL_5_PIN),
            lines[3]);
    } else if (g_self_test.state != SELF_TEST_CALIBRATING) {
        chassis_self_test_view_keys(
            gpio_get(ML_KEY_UP_PORT, ML_KEY_UP_PIN),
            gpio_get(ML_KEY_LEFT_PORT, ML_KEY_LEFT_PIN),
            gpio_get(ML_KEY_DOWN_PORT, ML_KEY_DOWN_PIN),
            gpio_get(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN),
            gpio_get(ML_KEY_RIGHT_PORT, ML_KEY_RIGHT_PIN),
            lines[3]);
    } else if (imu_state == ICM42688_SERVICE_STATE_SENSOR_READ_ERROR) {
        self_test_set_line(lines[3], "IMU READ ERR");
    } else if (imu_state ==
        ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR) {
        self_test_set_line(lines[3], "IMU DATA ERR");
    } else if ((g_self_test.imu_notice == SELF_TEST_IMU_NOTICE_TIMING) &&
        self_test_notice_active(now_ms)) {
        self_test_set_line(lines[3], "IMU LOOP SLOW");
    } else if ((g_self_test.imu_notice == SELF_TEST_IMU_NOTICE_MOVE) &&
        self_test_notice_active(now_ms)) {
        self_test_set_line(lines[0], "MOVE RETRY");
        chassis_self_test_view_move(
            self_test_scale_norm(
                g_self_test.imu_output.gyro_norm_dps, 10.0f),
            self_test_scale_norm(
                g_self_test.imu_output.accel_norm_g, 100.0f),
            lines[3]);
    } else {
        chassis_self_test_view_calibration(
            g_self_test.imu_output.calibration_samples, lines[3]);
    }
    for (line = 0U; line < OLED_TEXT_LINE_COUNT; ++line) {
        self_test_show_line((uint8_t) (line + 1U), lines[line]);
    }
}

static void self_test_fail(ml_status_t status)
{
    chassis_stop();
    g_self_test.error_status = status;
    g_self_test.state = SELF_TEST_ERROR;
}

static void self_test_handle_imu_event(icm42688_service_event_t event)
{
    uint32_t now_ms = g_self_test.imu_output.timestamp_ms;
    icm42688_service_state_t state =
        icm42688_service_get_state(&g_self_test.imu_service);
    bool service_error =
        (state == ICM42688_SERVICE_STATE_SENSOR_READ_ERROR) ||
        (state == ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR);

    if (event == ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED) {
        g_self_test.imu_notice = SELF_TEST_IMU_NOTICE_MOVE;
        g_self_test.imu_notice_until_ms = now_ms +
            SELF_TEST_IMU_NOTICE_MS;
        g_self_test.imu_timing_reset_streak = 0U;
    } else if (event == ICM42688_SERVICE_EVENT_TIMING_RESET) {
        if (g_self_test.imu_timing_reset_streak < UINT8_MAX) {
            ++g_self_test.imu_timing_reset_streak;
        }
        g_self_test.imu_notice = SELF_TEST_IMU_NOTICE_TIMING;
        g_self_test.imu_notice_until_ms = now_ms +
            SELF_TEST_IMU_NOTICE_MS;
        if (g_self_test.imu_timing_reset_streak >=
            SELF_TEST_IMU_TIMING_LIMIT) {
            self_test_fail(ML_STATUS_TIMEOUT);
        }
    } else if ((event == ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS) ||
        (event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE) ||
        (event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED) ||
        (event == ICM42688_SERVICE_EVENT_READ_RECOVERED) ||
        (event == ICM42688_SERVICE_EVENT_UPDATE_RECOVERED)) {
        g_self_test.imu_timing_reset_streak = 0U;
    }

    if (service_error) {
        if (!g_self_test.imu_error_active) {
            g_self_test.imu_error_active = true;
            g_self_test.imu_error_start_ms = now_ms;
        } else if ((uint32_t) (now_ms - g_self_test.imu_error_start_ms) >=
            SELF_TEST_IMU_ERROR_TIMEOUT_MS) {
            self_test_fail(icm42688_service_get_last_status(
                &g_self_test.imu_service));
        }
    } else {
        g_self_test.imu_error_active = false;
    }
}

static void self_test_begin_lift_stage(
    self_test_state_t state, float left_speed, float right_speed,
    const chassis_status_t *status)
{
    ml_status_t command_status;

    command_status = chassis_set_wheel_speed(left_speed, right_speed);
    if (command_status != ML_STATUS_OK) {
        self_test_fail(command_status);
        return;
    }
    g_self_test.state = state;
    g_self_test.stage_start_ms = status->timestamp_ms;
    g_self_test.stage_start_left_ticks = status->encoder_total_left;
    g_self_test.stage_start_right_ticks = status->encoder_total_right;
}

static bool self_test_lift_stage_passed(
    self_test_state_t state, const chassis_status_t *status)
{
    int32_t left_delta = status->encoder_total_left -
        g_self_test.stage_start_left_ticks;
    int32_t right_delta = status->encoder_total_right -
        g_self_test.stage_start_right_ticks;

    if (state == SELF_TEST_LEFT_FORWARD) {
        return left_delta >= SELF_TEST_MIN_STAGE_TICKS;
    }
    if (state == SELF_TEST_LEFT_REVERSE) {
        return left_delta <= -SELF_TEST_MIN_STAGE_TICKS;
    }
    if (state == SELF_TEST_RIGHT_FORWARD) {
        return right_delta >= SELF_TEST_MIN_STAGE_TICKS;
    }
    if (state == SELF_TEST_RIGHT_REVERSE) {
        return right_delta <= -SELF_TEST_MIN_STAGE_TICKS;
    }
    return (left_delta >= SELF_TEST_MIN_STAGE_TICKS) &&
        (right_delta >= SELF_TEST_MIN_STAGE_TICKS);
}

static void self_test_advance_lift(const chassis_status_t *status)
{
    self_test_state_t completed = g_self_test.state;

    chassis_stop();
    if (!self_test_lift_stage_passed(completed, status)) {
        self_test_fail(ML_STATUS_TIMEOUT);
        return;
    }
    if (completed == SELF_TEST_LEFT_FORWARD) {
        self_test_begin_lift_stage(SELF_TEST_LEFT_REVERSE,
            -SELF_TEST_LIFT_SPEED_MM_S, 0.0f, status);
    } else if (completed == SELF_TEST_LEFT_REVERSE) {
        self_test_begin_lift_stage(SELF_TEST_RIGHT_FORWARD,
            0.0f, SELF_TEST_LIFT_SPEED_MM_S, status);
    } else if (completed == SELF_TEST_RIGHT_FORWARD) {
        self_test_begin_lift_stage(SELF_TEST_RIGHT_REVERSE,
            0.0f, -SELF_TEST_LIFT_SPEED_MM_S, status);
    } else if (completed == SELF_TEST_RIGHT_REVERSE) {
        self_test_begin_lift_stage(SELF_TEST_BOTH_FORWARD,
            SELF_TEST_LIFT_BOTH_MM_S,
            SELF_TEST_LIFT_BOTH_MM_S, status);
    } else {
        g_self_test.state = SELF_TEST_WAIT_GROUND;
        chassis_key_require_release(&g_self_test.center_key);
    }
}

static void self_test_reset_roll_baseline(const chassis_status_t *status)
{
    g_self_test.roll_start_left_ticks = status->encoder_total_left;
    g_self_test.roll_start_right_ticks = status->encoder_total_right;
    g_self_test.roll_start_left_bad = status->encoder_invalid_left;
    g_self_test.roll_start_right_bad = status->encoder_invalid_right;
}

static void self_test_begin_roll_calibration(
    const chassis_status_t *status)
{
    chassis_stop();
    self_test_reset_roll_baseline(status);
    g_self_test.state = SELF_TEST_ROLL_CALIBRATION;
    chassis_key_require_release(&g_self_test.center_key);
}

static void self_test_begin_uart_diagnostic(
    const chassis_status_t *status)
{
    chassis_stop();
    g_self_test.state = SELF_TEST_UART_DIAGNOSTIC;
    g_self_test.uart_diag_next_banner_ms = status->timestamp_ms;
    g_self_test.uart_diag_banner_count = 0U;
    g_self_test.uart_diag_last_byte = 0U;
    g_self_test.uart_diag_last_errors = 0U;
    g_self_test.uart_diag_rx_seen = false;
    chassis_key_require_release(&g_self_test.center_key);
}

static ml_status_t self_test_begin_fusion_capture(void)
{
    ml_status_t status;

    chassis_stop();
    status = chassis_idle_capture_start();
    if (status == ML_STATUS_OK) {
        g_self_test.state = SELF_TEST_FUSION_CAPTURE;
        chassis_key_require_release(&g_self_test.center_key);
    }
    return status;
}

static void self_test_end_fusion_capture(void)
{
    chassis_idle_capture_stop();
    g_self_test.state = SELF_TEST_WAIT_LIFT;
    chassis_key_require_release(&g_self_test.center_key);
}

static void self_test_uart_diagnostic_send_banner(uint32_t now_ms)
{
    ml_status_t status;

    if ((int32_t) (now_ms -
        g_self_test.uart_diag_next_banner_ms) < 0) {
        return;
    }
    if (g_self_test.uart_diag_banner_count < 9999U) {
        ++g_self_test.uart_diag_banner_count;
    }
    status = chassis_uart0_send_diagnostic_banner(
        g_self_test.uart_diag_banner_count);
    if (status != ML_STATUS_OK) {
        g_self_test.error_status = status;
    }
    g_self_test.uart_diag_next_banner_ms = now_ms +
        SELF_TEST_UART_DIAG_PERIOD_MS;
}

static void self_test_uart_diagnostic_handle_byte(uint8_t byte)
{
    uart_diagnostics_t diagnostics;
    ml_status_t status;

    memset(&diagnostics, 0, sizeof(diagnostics));
    (void) uart_get_diagnostics(UART0, &diagnostics);
    g_self_test.uart_diag_last_byte = byte;
    g_self_test.uart_diag_last_errors = diagnostics.last_rx_errors;
    g_self_test.uart_diag_rx_seen = true;
    status = chassis_uart0_send_diagnostic_rx(
        byte, diagnostics.last_rx_errors);
    if (status != ML_STATUS_OK) {
        g_self_test.error_status = status;
    }
}

static void self_test_handle_waiting_key(const chassis_status_t *status)
{
    ml_status_t command_status;

    if (g_self_test.state == SELF_TEST_ROLL_CALIBRATION) {
        self_test_reset_roll_baseline(status);
    } else if (g_self_test.state == SELF_TEST_WAIT_LIFT) {
        self_test_begin_lift_stage(SELF_TEST_LEFT_FORWARD,
            SELF_TEST_LIFT_SPEED_MM_S, 0.0f, status);
    } else if (g_self_test.state == SELF_TEST_WAIT_GROUND) {
        chassis_reset_pose(0.0f, 0.0f, 0.0f);
        command_status = chassis_move_mm(
            500.0f, SELF_TEST_GROUND_SPEED_MM_S);
        if (command_status == ML_STATUS_OK) {
            g_self_test.state = SELF_TEST_RUN_500MM;
        } else {
            self_test_fail(command_status);
        }
    } else if (g_self_test.state == SELF_TEST_WAIT_POSITIVE_90) {
        command_status = chassis_rotate_deg(
            90.0f, SELF_TEST_TURN_SPEED_DEG_S);
        if (command_status == ML_STATUS_OK) {
            g_self_test.state = SELF_TEST_RUN_POSITIVE_90;
        } else {
            self_test_fail(command_status);
        }
    } else if (g_self_test.state == SELF_TEST_WAIT_NEGATIVE_90) {
        command_status = chassis_rotate_deg(
            -90.0f, SELF_TEST_TURN_SPEED_DEG_S);
        if (command_status == ML_STATUS_OK) {
            g_self_test.state = SELF_TEST_RUN_NEGATIVE_90;
        } else {
            self_test_fail(command_status);
        }
    }
}

ml_status_t chassis_self_test_init(void)
{
    chassis_status_t chassis_status;
    ml_status_t status;

    g_self_test.state = SELF_TEST_UNINITIALIZED;
    g_self_test.initialized = false;
    chassis_key_init(&g_self_test.center_key);
    g_self_test.last_display_ms = 0U;
    g_self_test.imu_error_start_ms = 0U;
    g_self_test.imu_notice_until_ms = 0U;
    g_self_test.imu_timing_reset_streak = 0U;
    g_self_test.imu_notice = SELF_TEST_IMU_NOTICE_NONE;
    g_self_test.imu_error_active = false;
    g_self_test.roll_start_left_ticks = 0;
    g_self_test.roll_start_right_ticks = 0;
    g_self_test.roll_start_left_bad = 0U;
    g_self_test.roll_start_right_bad = 0U;
    g_self_test.uart_diag_next_banner_ms = 0U;
    g_self_test.uart_diag_banner_count = 0U;
    g_self_test.uart_diag_last_byte = 0U;
    g_self_test.uart_diag_last_errors = 0U;
    g_self_test.uart_diag_rx_seen = false;
    memset(g_self_test.display_line_valid, 0,
        sizeof(g_self_test.display_line_valid));
    memset(g_self_test.display_line, 0, sizeof(g_self_test.display_line));
    g_self_test.error_status = ML_STATUS_OK;
    status = OLED_Init();
    if (status == ML_STATUS_OK) {
        status = self_test_keys_init();
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(UART0, SELF_TEST_UART_BAUD, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = chassis_init(&g_chassis_default_config);
    }
    if (status == ML_STATUS_OK) {
        status = icm42688_service_init(&g_self_test.imu_service,
            &g_self_test_imu_config);
    }
    if (status != ML_STATUS_OK) {
        chassis_emergency_stop();
        g_self_test.error_status = status;
        g_self_test.state = SELF_TEST_ERROR;
        chassis_status = chassis_get_status();
        self_test_show(&chassis_status);
        return status;
    }
    g_self_test.state = SELF_TEST_CALIBRATING;
    g_self_test.initialized = true;
    (void) OLED_Clear();
    return ML_STATUS_OK;
}

void chassis_self_test_poll(void)
{
    chassis_status_t status;
    icm42688_service_event_t imu_event;
    ml_status_t uart_status;
    uint8_t uart_byte;
#if !CHASSIS_SW6_MAPPING_DIAGNOSTIC
    bool key_pressed;
#endif

    if (!g_self_test.initialized) {
        return;
    }
    imu_event = icm42688_service_poll(&g_self_test.imu_service,
        &g_self_test.imu_output);
    self_test_handle_imu_event(imu_event);
    if ((imu_event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED) ||
        (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE)) {
        chassis_set_imu_sample(
            g_self_test.imu_output.angles.yaw_deg,
            g_self_test.imu_output.body_gyro_z_dps,
            g_self_test.imu_output.timestamp_ms, true);
    }
    chassis_poll();
    status = chassis_get_status();
#if !CHASSIS_SW6_MAPPING_DIAGNOSTIC
    chassis_key_update(&g_self_test.center_key,
        self_test_center_pressed(), status.timestamp_ms);
    key_pressed = chassis_key_take_press(&g_self_test.center_key);
    if (key_pressed && self_test_motion_running(g_self_test.state)) {
        chassis_emergency_stop();
        g_self_test.state = SELF_TEST_EMERGENCY;
    } else if (key_pressed) {
        self_test_handle_waiting_key(&status);
    }
#endif

    if ((g_self_test.state == SELF_TEST_CALIBRATING) &&
        (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE)) {
#if CHASSIS_SW6_MAPPING_DIAGNOSTIC
        chassis_stop();
        g_self_test.state = SELF_TEST_MAP_SW6;
#else
        if (self_test_right_pressed()) {
            self_test_begin_uart_diagnostic(&status);
        } else if (self_test_center_pressed()) {
            self_test_begin_roll_calibration(&status);
        } else {
            g_self_test.state = SELF_TEST_WAIT_LIFT;
            chassis_key_require_release(&g_self_test.center_key);
        }
#endif
    } else if (self_test_motion_running(g_self_test.state)) {
        if (status.result == CHASSIS_RESULT_FAULT) {
            self_test_fail(ML_STATUS_TIMEOUT);
        } else if ((g_self_test.state >= SELF_TEST_LEFT_FORWARD) &&
            (g_self_test.state <= SELF_TEST_BOTH_FORWARD) &&
            ((uint32_t) (status.timestamp_ms -
             g_self_test.stage_start_ms) >= SELF_TEST_LIFT_STAGE_MS)) {
            self_test_advance_lift(&status);
        } else if (status.result == CHASSIS_RESULT_COMPLETE) {
            if (g_self_test.state == SELF_TEST_RUN_500MM) {
                g_self_test.state = SELF_TEST_WAIT_POSITIVE_90;
                chassis_key_require_release(&g_self_test.center_key);
            } else if (g_self_test.state == SELF_TEST_RUN_POSITIVE_90) {
                g_self_test.state = SELF_TEST_WAIT_NEGATIVE_90;
                chassis_key_require_release(&g_self_test.center_key);
            } else if (g_self_test.state == SELF_TEST_RUN_NEGATIVE_90) {
                g_self_test.state = SELF_TEST_COMPLETE;
            }
        }
    }

    while (uart_try_read(UART0, &uart_byte) == ML_STATUS_OK) {
        if (g_self_test.state == SELF_TEST_UART_DIAGNOSTIC) {
            self_test_uart_diagnostic_handle_byte(uart_byte);
            uart_status = ML_STATUS_OK;
        } else if (((uart_byte == (uint8_t) 'F') ||
                    (uart_byte == (uint8_t) 'f')) &&
                   (g_self_test.state == SELF_TEST_WAIT_LIFT)) {
            uart_status = self_test_begin_fusion_capture();
        } else if (((uart_byte == (uint8_t) 'S') ||
                    (uart_byte == (uint8_t) 's')) &&
                   (g_self_test.state == SELF_TEST_FUSION_CAPTURE)) {
            self_test_end_fusion_capture();
            uart_status = ML_STATUS_OK;
        } else {
            uart_status = chassis_telemetry_uart0_handle_byte(uart_byte,
                !self_test_motion_running(g_self_test.state) &&
                (g_self_test.state != SELF_TEST_FUSION_CAPTURE));
        }
        if (uart_status == ML_STATUS_TIMEOUT) {
            /* Preserve the failure for debugger inspection without
             * changing the stopped self-test state or safety behavior. */
            g_self_test.error_status = uart_status;
        }
    }
    if (g_self_test.state == SELF_TEST_UART_DIAGNOSTIC) {
        self_test_uart_diagnostic_send_banner(status.timestamp_ms);
    }
    if ((uint32_t) (status.timestamp_ms - g_self_test.last_display_ms) >=
        SELF_TEST_DISPLAY_PERIOD_MS) {
        self_test_show(&status);
        g_self_test.last_display_ms = status.timestamp_ms;
    }
}
