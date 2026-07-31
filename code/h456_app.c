#include "h456_app.h"

#include <stdio.h>
#include <string.h>

#include "ball_balance.h"
#include "ball_balance_config.h"
#include "chassis.h"
#include "chassis_config.h"
#include "chassis_key.h"
#include "chassis_track_line_control.h"
#include "h456_telemetry.h"
#include "icm42688_service.h"
#include "line_sensor.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"

#define H456_APP_DISPLAY_PERIOD_MS       (100U)
#define H456_APP_UART_BAUD               (115200UL)
#define H456_APP_UART_PRIORITY           (2U)
#define H456_APP_BALL_READY_ERROR_CM     (0.8f)
#define H456_APP_BALL_READY_SPEED_CM_S   (1.0f)
#define H456_APP_BALL_READY_SETTLE_MS    (500U)
#define H456_APP_BALL_SCORE_ERROR_CM     (1.0f)
#define H456_APP_BALL_SCORE_IMMEDIATE_CM (1.2f)
#define H456_APP_BALL_SCORE_CONFIRM_MS   (500U)
#define H456_APP_BALL_ABORT_ERROR_CM     (2.0f)
#define H456_APP_BALL_ABORT_HOLD_MS      (100U)
#define H456_APP_LAUNCH_BIAS_US          (60.0f)
#define H456_APP_LAUNCH_BIAS_MS          (6000U)
#define H456_APP_H4_LINE_CORRECTION_RATIO (0.18f)
#define H456_APP_H4_LINE_CORRECTION_MAX_MM_S (60.0f)
#define H456_APP_H4_OUTER_CORRECTION_RATIO (0.28f)
#define H456_APP_H4_OUTER_CORRECTION_MAX_MM_S (80.0f)
#define H456_APP_H4_CURVE_HOLD_CORRECTION_RATIO \
    H456_APP_H4_LINE_CORRECTION_RATIO
#define H456_APP_H4_CURVE_HOLD_CORRECTION_MAX_MM_S \
    H456_APP_H4_LINE_CORRECTION_MAX_MM_S
#define H456_APP_H4_HEADING_ONLY_ENTER_DEG (6.0f)
#define H456_APP_H4_HEADING_ONLY_EXIT_DEG  (3.0f)
#define H456_APP_H6_TARGET_MIN_CM        (-10.0f)
#define H456_APP_H6_TARGET_MAX_CM        (10.0f)
#define H456_APP_H6_TARGET_STEP_CM       (0.5f)

typedef struct {
    icm42688_service_t imu;
    icm42688_service_output_t imu_output;
    h456_mission_t mission;
    h456_mission_output_t mission_output;
    chassis_track_line_control_t line_control;
    chassis_track_line_control_output_t line_output;
    chassis_key_t center_key;
    chassis_key_t up_key;
    chassis_key_t down_key;
    chassis_key_t left_key;
    chassis_key_t right_key;
    line_sensor_white_guard_t white_guard;
    line_sample_t last_line;
    ball_balance_status_t ball;
    h456_app_state_t state;
    h456_app_fault_t fault;
    h456_mode_t mode;
    float target_cm;
    float h6_target_cm;
    float maximum_score_error_cm;
    float interval_error_min_cm;
    float interval_error_max_cm;
    uint32_t ball_settle_start_ms;
    uint32_t ball_score_over_start_ms;
    uint32_t ball_abort_start_ms;
    uint32_t launch_bias_start_ms;
    uint32_t last_control_ms;
    uint32_t last_white_sample_ms;
    uint32_t last_display_ms;
    bool imu_ready;
    bool ball_settle_active;
    bool ball_settled;
    bool ball_score_over_active;
    bool ball_abort_active;
    bool ball_violation;
    bool launch_bias_active;
    bool h4_heading_only_active;
    bool score_frozen;
    bool velocity_started;
    bool display_dirty;
    bool initialized;
} h456_app_context_t;

static const icm42688_service_config_t g_h456_imu_config = {
    {
        {
            IMU_ATTITUDE_AXIS_X,
            IMU_ATTITUDE_AXIS_Y,
            IMU_ATTITUDE_AXIS_Z
        },
        {-1, 1, -1},
        150U
    },
    TIMG8,
    1U
};

static h456_app_context_t g_app;

static float h456_app_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float h456_app_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool h456_key_pressed(
    GPIO_Regs *port, uint32_t pin)
{
    return gpio_get(port, pin) == ML_KEY_ACTIVE_LEVEL;
}

static ml_status_t h456_keys_init(void)
{
    ml_status_t status;

    status = board_resource_claim(
        ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_UP_PORT, ML_KEY_UP_PIN,
            (GPIOn_enum) ML_KEY_UP_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_DOWN_PORT, ML_KEY_DOWN_PIN,
            (GPIOn_enum) ML_KEY_DOWN_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_LEFT_PORT, ML_KEY_LEFT_PIN,
            (GPIOn_enum) ML_KEY_LEFT_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_RIGHT_PORT, ML_KEY_RIGHT_PIN,
            (GPIOn_enum) ML_KEY_RIGHT_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
            (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    }
    return status;
}

static void h456_show_line(uint8_t line, const char *text)
{
    char padded[OLED_TEXT_COLUMN_COUNT + 1U];
    uint8_t index = 0U;

    while ((index < OLED_TEXT_COLUMN_COUNT) && (text[index] != '\0')) {
        padded[index] = text[index];
        ++index;
    }
    while (index < OLED_TEXT_COLUMN_COUNT) {
        padded[index++] = ' ';
    }
    padded[OLED_TEXT_COLUMN_COUNT] = '\0';
    (void) OLED_ShowLine(line, padded);
}

static int32_t h456_tenths(float value)
{
    float scaled = value * 10.0f;

    return (int32_t) (scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static const char *h456_setup_wait_text(void)
{
    if (!g_app.ball.vision_ready) {
        return "WAIT VISION";
    }
    if (!g_app.ball.enabled ||
        (g_app.ball.state != BALL_BALANCE_ACTIVE)) {
        return "WAIT CTRL ACTIVE";
    }
    if (h456_app_abs(g_app.ball.error_cm) >
        H456_APP_BALL_READY_ERROR_CM) {
        return "WAIT POS <=0.8";
    }
    if (h456_app_abs(g_app.ball.velocity_cm_per_s) >
        H456_APP_BALL_READY_SPEED_CM_S) {
        return "WAIT SPD <=1.0";
    }
    return "SETTLE 500MS";
}

static void h456_show_boot(void)
{
    h456_show_line(1U, "H456 BOOT");
    h456_show_line(2U, "INIT MODULES");
    h456_show_line(3U, "MOTORS SAFE");
    h456_show_line(4U, "WAIT...");
}

static void h456_show_init_failure(ml_status_t status, const char *detail)
{
    char line[OLED_TEXT_COLUMN_COUNT + 1U];

    h456_show_line(1U, "H456 INIT FAIL");
    (void) snprintf(line, sizeof(line), "STATUS %02u",
        (unsigned int) status);
    h456_show_line(2U, line);
    h456_show_line(3U, "MOTORS SAFE");
    h456_show_line(4U, detail == 0 ? "CHECK MODULES" : detail);
}

static void h456_show(void)
{
    char line[OLED_TEXT_COLUMN_COUNT + 1U];
    int32_t target_tenths = h456_tenths(g_app.target_cm);
    int32_t ball_tenths = h456_tenths(g_app.ball.position_cm);
    uint32_t time_ms = g_app.mission_output.score_elapsed_ms;
    bool score_pass = g_app.mission_output.score_point_passed &&
        (g_app.mission_output.result == H456_MISSION_RESULT_PASS) &&
        !g_app.ball_violation;

    if (g_app.state == H456_APP_CALIBRATING) {
        h456_show_line(1U, "H456 CALIBRATE");
        (void) snprintf(line, sizeof(line), "IMU%u LF%02u/%02u",
            g_app.imu_ready ? 1U : 0U,
            (unsigned int) g_app.white_guard.consecutive_samples,
            (unsigned int) LINE_SENSOR_WHITE_STABLE_SAMPLES);
        h456_show_line(2U, line);
        (void) snprintf(line, sizeof(line), "VISION %s",
            g_app.ball.vision_ready ? "READY" : "WAIT");
        h456_show_line(3U, line);
        h456_show_line(4U, "WAIT READY SAFE");
    } else if ((g_app.state == H456_APP_SETUP) ||
               (g_app.state == H456_APP_READY)) {
        (void) snprintf(line, sizeof(line), "H%u %s",
            (unsigned int) g_app.mode,
            g_app.state == H456_APP_READY ? "READY" : "BALL SETUP");
        h456_show_line(1U, line);
        (void) snprintf(line, sizeof(line), "T%+03ld.%01ld P%+03ld.%01ld",
            (long) (target_tenths / 10),
            (long) h456_app_abs((float) (target_tenths % 10)),
            (long) (ball_tenths / 10),
            (long) h456_app_abs((float) (ball_tenths % 10)));
        h456_show_line(2U, line);
        h456_show_line(3U, g_app.mode == H456_MODE_6 ?
            "L/R TARGET 0.5" : "TARGET FIXED 0");
        h456_show_line(4U, g_app.state == H456_APP_READY ?
            "C=GO U/D=MODE" : h456_setup_wait_text());
    } else if (g_app.state == H456_APP_FINISHED) {
        (void) snprintf(line, sizeof(line), "H%u %s T%02lu.%01lu",
            (unsigned int) g_app.mode,
            score_pass ? "PASS" : "FAIL",
            (unsigned long) (time_ms / 1000U),
            (unsigned long) ((time_ms / 100U) % 10U));
        h456_show_line(1U, line);
        (void) snprintf(line, sizeof(line), "BALL MAX %02ld.%01ld",
            (long) (h456_tenths(g_app.maximum_score_error_cm) / 10),
            (long) h456_app_abs((float)
                (h456_tenths(g_app.maximum_score_error_cm) % 10)));
        h456_show_line(2U, line);
        h456_show_line(3U, "C=SAFE RECENTER");
        (void) snprintf(line, sizeof(line), "CSV N%03u WAIT",
            (unsigned int) h456_telemetry_count());
        h456_show_line(4U, line);
    } else if (g_app.state == H456_APP_RECENTERING) {
        h456_show_line(1U, g_app.fault == H456_APP_FAULT_NONE ?
            "H456 RECENTER" : "H456 FAULT SAFE");
        (void) snprintf(line, sizeof(line), "SERVO %04u/%04u",
            (unsigned int) g_app.ball.servo_current_us,
            (unsigned int) g_app.ball.servo_target_us);
        h456_show_line(2U, line);
        h456_show_line(3U, "MOTORS LOCKED");
        h456_show_line(4U, "WAIT 1500 US");
    } else if ((g_app.state == H456_APP_EXPORT_READY) ||
               (g_app.state == H456_APP_FAULT)) {
        (void) snprintf(line, sizeof(line), "H456 %s F%u",
            g_app.fault == H456_APP_FAULT_NONE ? "SAFE" : "FAULT",
            (unsigned int) g_app.fault);
        h456_show_line(1U, line);
        (void) snprintf(line, sizeof(line), "T%02lu.%01lu M%02ld.%01ld",
            (unsigned long) (time_ms / 1000U),
            (unsigned long) ((time_ms / 100U) % 10U),
            (long) (h456_tenths(g_app.maximum_score_error_cm) / 10),
            (long) h456_app_abs((float)
                (h456_tenths(g_app.maximum_score_error_cm) % 10)));
        h456_show_line(2U, line);
        h456_show_line(3U, "MOTORS+SERVO SAFE");
        (void) snprintf(line, sizeof(line), "CSV N%03u D=OUT",
            (unsigned int) h456_telemetry_count());
        h456_show_line(4U, line);
    }
}

static void h456_update_keys(uint32_t now_ms)
{
    chassis_key_update(&g_app.center_key,
        h456_key_pressed(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN), now_ms);
    chassis_key_update(&g_app.up_key,
        h456_key_pressed(ML_KEY_UP_PORT, ML_KEY_UP_PIN), now_ms);
    chassis_key_update(&g_app.down_key,
        h456_key_pressed(ML_KEY_DOWN_PORT, ML_KEY_DOWN_PIN), now_ms);
    chassis_key_update(&g_app.left_key,
        h456_key_pressed(ML_KEY_LEFT_PORT, ML_KEY_LEFT_PIN), now_ms);
    chassis_key_update(&g_app.right_key,
        h456_key_pressed(ML_KEY_RIGHT_PORT, ML_KEY_RIGHT_PIN), now_ms);
}

static void h456_require_key_release(void)
{
    chassis_key_require_release(&g_app.center_key);
    chassis_key_require_release(&g_app.up_key);
    chassis_key_require_release(&g_app.down_key);
    chassis_key_require_release(&g_app.left_key);
    chassis_key_require_release(&g_app.right_key);
}

static ml_status_t h456_configure_line_control(h456_mode_t mode)
{
    chassis_track_line_control_config_t line_config =
        g_chassis_track_line_control_default_config;

    line_config.effective_track_mm =
        g_chassis_race_config.effective_track_mm;
    line_config.maximum_wheel_speed_mm_s =
        g_chassis_race_config.maximum_wheel_speed_mm_s;
    line_config.control_period_ms =
        g_chassis_race_config.control_period_ms;
    if (mode == H456_MODE_4) {
        line_config.correction_ratio =
            H456_APP_H4_LINE_CORRECTION_RATIO;
        line_config.maximum_correction_mm_s =
            H456_APP_H4_LINE_CORRECTION_MAX_MM_S;
        line_config.outer_single_correction_ratio =
            H456_APP_H4_OUTER_CORRECTION_RATIO;
        line_config.outer_single_maximum_correction_mm_s =
            H456_APP_H4_OUTER_CORRECTION_MAX_MM_S;
        line_config.curve_hold_correction_ratio =
            H456_APP_H4_CURVE_HOLD_CORRECTION_RATIO;
        line_config.curve_hold_maximum_correction_mm_s =
            H456_APP_H4_CURVE_HOLD_CORRECTION_MAX_MM_S;
    }
    return chassis_track_line_control_init(
        &g_app.line_control, &line_config);
}

static void h456_reset_settle(void)
{
    g_app.ball_settle_active = false;
    g_app.ball_settled = false;
    g_app.ball_settle_start_ms = 0U;
    if (g_app.state == H456_APP_READY) {
        g_app.state = H456_APP_SETUP;
    }
}

static void h456_clear_launch_bias(void)
{
    g_app.launch_bias_active = false;
    g_app.launch_bias_start_ms = 0U;
    (void) ball_balance_set_control_bias_us(0.0f);
}

static void h456_start_launch_bias(uint32_t now_ms)
{
    g_app.launch_bias_active = true;
    g_app.launch_bias_start_ms = now_ms;
    (void) ball_balance_set_control_bias_us(H456_APP_LAUNCH_BIAS_US);
}

static void h456_update_launch_bias(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float scale;

    if (!g_app.launch_bias_active) {
        return;
    }
    if (g_app.state != H456_APP_RUNNING) {
        h456_clear_launch_bias();
        return;
    }
    elapsed_ms = now_ms - g_app.launch_bias_start_ms;
    if (elapsed_ms >= H456_APP_LAUNCH_BIAS_MS) {
        h456_clear_launch_bias();
        return;
    }
    scale = (float) (H456_APP_LAUNCH_BIAS_MS - elapsed_ms) /
        (float) H456_APP_LAUNCH_BIAS_MS;
    (void) ball_balance_set_control_bias_us(
        H456_APP_LAUNCH_BIAS_US * scale);
}

static ml_status_t h456_apply_target(void)
{
    ml_status_t status = ball_balance_set_target_cm(g_app.target_cm);

    if (status != ML_STATUS_OK) {
        return status;
    }
    h456_reset_settle();
    return ML_STATUS_OK;
}

static void h456_select_next_mode(bool forward)
{
    if (forward) {
        g_app.mode = g_app.mode == H456_MODE_6 ?
            H456_MODE_4 : (h456_mode_t) ((uint8_t) g_app.mode + 1U);
    } else {
        g_app.mode = g_app.mode == H456_MODE_4 ?
            H456_MODE_6 : (h456_mode_t) ((uint8_t) g_app.mode - 1U);
    }
    g_app.target_cm = g_app.mode == H456_MODE_6 ?
        g_app.h6_target_cm : 0.0f;
    (void) h456_apply_target();
    g_app.h4_heading_only_active = false;
    (void) h456_configure_line_control(g_app.mode);
    g_app.display_dirty = true;
}

static void h456_adjust_h6_target(float delta_cm)
{
    if (g_app.mode != H456_MODE_6) {
        return;
    }
    g_app.h6_target_cm = h456_app_clamp(
        g_app.h6_target_cm + delta_cm,
        H456_APP_H6_TARGET_MIN_CM, H456_APP_H6_TARGET_MAX_CM);
    g_app.target_cm = g_app.h6_target_cm;
    (void) h456_apply_target();
    g_app.display_dirty = true;
}

static void h456_update_ball_settle(uint32_t now_ms)
{
    bool stable = g_app.ball.vision_ready && g_app.ball.enabled &&
        (g_app.ball.state == BALL_BALANCE_ACTIVE) &&
        (h456_app_abs(g_app.ball.error_cm) <=
         H456_APP_BALL_READY_ERROR_CM) &&
        (h456_app_abs(g_app.ball.velocity_cm_per_s) <=
         H456_APP_BALL_READY_SPEED_CM_S);

    if (!stable) {
        h456_reset_settle();
        return;
    }
    if (!g_app.ball_settle_active) {
        g_app.ball_settle_active = true;
        g_app.ball_settle_start_ms = now_ms;
        return;
    }
    if ((now_ms - g_app.ball_settle_start_ms) >=
        H456_APP_BALL_READY_SETTLE_MS) {
        g_app.ball_settled = true;
        g_app.state = H456_APP_READY;
    }
}

static void h456_build_telemetry_sample(
    const chassis_status_t *chassis,
    h456_telemetry_sample_t *sample)
{
    memset(sample, 0, sizeof(*sample));
    sample->timestamp_ms = chassis->timestamp_ms;
    sample->mode = g_app.mode;
    sample->mission_state = g_app.mission_output.state;
    sample->progress_mm = g_app.mission_output.progress_mm;
    sample->fused_heading_deg = chassis->fused_heading_deg -
        g_app.mission.start_heading_deg;
    sample->target_center_mm_s = 0.5f *
        (chassis->target_left_mm_s + chassis->target_right_mm_s);
    sample->actual_center_mm_s = 0.5f *
        (chassis->measured_left_mm_s + chassis->measured_right_mm_s);
    sample->pwm_left_count = chassis->pwm_left_count;
    sample->pwm_right_count = chassis->pwm_right_count;
    sample->line_bits = g_app.last_line.black_bits;
    sample->line_usable = g_app.line_output.line_valid;
    sample->line_recovering = g_app.line_output.recovering;
    sample->line_correction_mm_s = g_app.line_output.correction_mm_s;
    sample->final_steering_bias_mm_s =
        g_app.line_output.final_steering_bias_mm_s;
    sample->ball_target_cm = g_app.ball.target_cm;
    sample->ball_position_cm = g_app.ball.position_cm;
    sample->ball_error_min_cm = g_app.interval_error_min_cm;
    sample->ball_error_max_cm = g_app.interval_error_max_cm;
    sample->ball_velocity_cm_s = g_app.ball.velocity_cm_per_s;
    sample->ball_control_output_us = g_app.ball.control_output_us;
    sample->servo_target_us = g_app.ball.servo_target_us;
    sample->servo_current_us = g_app.ball.servo_current_us;
    sample->raw_x_px = g_app.ball.raw_center_x_px;
    sample->raw_y_px = g_app.ball.raw_center_y_px;
    sample->vision_age_ms = g_app.ball.vision_age_ms;
    sample->frame_interval_ms = g_app.ball.vision_frame_interval_ms;
    sample->vision_ready = g_app.ball.vision_ready;
    sample->ball_enabled = g_app.ball.enabled;
    sample->ball_violation = g_app.ball_violation;
    sample->breakaway_fault = g_app.ball.breakaway_fault;
    sample->score_point_passed =
        g_app.mission_output.score_point_passed;
}

static void h456_record_telemetry(
    const chassis_status_t *chassis, bool force)
{
    h456_telemetry_sample_t sample;
    ml_status_t status;

    if (!h456_telemetry_session_active()) {
        return;
    }
    h456_build_telemetry_sample(chassis, &sample);
    status = h456_telemetry_record(&sample, force);
    if (status == ML_STATUS_OK) {
        g_app.interval_error_min_cm = g_app.ball.error_cm;
        g_app.interval_error_max_cm = g_app.ball.error_cm;
    }
}

static void h456_finish_telemetry(const chassis_status_t *chassis)
{
    h456_telemetry_sample_t sample;
    bool ball_passed = !g_app.ball_violation &&
        g_app.mission_output.score_point_passed;

    h456_telemetry_set_result(g_app.mission_output.score_elapsed_ms,
        g_app.maximum_score_error_cm, ball_passed);
    if (h456_telemetry_session_active()) {
        h456_build_telemetry_sample(chassis, &sample);
        h456_telemetry_session_finish(&sample);
    }
}

static void h456_start_recenter(h456_app_fault_t fault,
    const chassis_status_t *chassis)
{
    chassis_emergency_stop();
    g_app.velocity_started = false;
    g_app.fault = fault;
    h456_clear_launch_bias();
    (void) ball_balance_enable(false);
    (void) ball_balance_get_status(&g_app.ball);
    h456_finish_telemetry(chassis);
    g_app.state = H456_APP_RECENTERING;
    g_app.display_dirty = true;
}

static ml_status_t h456_start_run(chassis_status_t *status)
{
    float center_distance_mm;
    ml_status_t result;

    result = line_sensor_reassert_inputs();
    if (result != ML_STATUS_OK) {
        return result;
    }
    g_app.last_line = line_sensor_read();
    if (g_app.last_line.io_fault) {
        return ML_STATUS_TIMEOUT;
    }
    chassis_reset_pose(0.0f, 0.0f, 0.0f);
    *status = chassis_get_status();
    center_distance_mm = 0.5f *
        (status->pose.left_distance_mm + status->pose.right_distance_mm);
    result = h456_mission_start(&g_app.mission, g_app.mode,
        center_distance_mm, status->fused_heading_deg,
        status->timestamp_ms);
    if (result != ML_STATUS_OK) {
        return result;
    }
    memset(&g_app.mission_output, 0, sizeof(g_app.mission_output));
    g_app.mission_output.mode = g_app.mode;
    g_app.mission_output.state = H456_MISSION_RUNNING;
    chassis_track_line_control_reset(&g_app.line_control);
    memset(&g_app.line_output, 0, sizeof(g_app.line_output));
    g_app.maximum_score_error_cm = h456_app_abs(g_app.ball.error_cm);
    g_app.interval_error_min_cm = g_app.ball.error_cm;
    g_app.interval_error_max_cm = g_app.ball.error_cm;
    g_app.ball_abort_active = false;
    g_app.ball_score_over_active = false;
    g_app.ball_score_over_start_ms = 0U;
    g_app.ball_violation = false;
    g_app.score_frozen = false;
    g_app.velocity_started = false;
    g_app.h4_heading_only_active = false;
    result = h456_configure_line_control(g_app.mode);
    if (result != ML_STATUS_OK) {
        return result;
    }
    g_app.last_control_ms = status->timestamp_ms -
        g_h456_mission_default_config.control_period_ms;
    h456_telemetry_session_start(g_app.mode, status->timestamp_ms);
    h456_start_launch_bias(status->timestamp_ms);
    g_app.state = H456_APP_RUNNING;
    return ML_STATUS_OK;
}

static void h456_update_score_and_safety(
    const chassis_status_t *chassis)
{
    float absolute_error = h456_app_abs(g_app.ball.error_cm);

    if (!g_app.score_frozen) {
        if (absolute_error > g_app.maximum_score_error_cm) {
            g_app.maximum_score_error_cm = absolute_error;
        }
        if (absolute_error >= H456_APP_BALL_SCORE_IMMEDIATE_CM) {
            g_app.ball_violation = true;
        } else if (absolute_error > H456_APP_BALL_SCORE_ERROR_CM) {
            if (!g_app.ball_score_over_active) {
                g_app.ball_score_over_active = true;
                g_app.ball_score_over_start_ms = chassis->timestamp_ms;
            } else if ((chassis->timestamp_ms -
                        g_app.ball_score_over_start_ms) >=
                       H456_APP_BALL_SCORE_CONFIRM_MS) {
                g_app.ball_violation = true;
            }
        } else {
            g_app.ball_score_over_active = false;
        }
        if (g_app.mission_output.score_point_passed) {
            g_app.score_frozen = true;
        }
    }
    if (g_app.ball.error_cm < g_app.interval_error_min_cm) {
        g_app.interval_error_min_cm = g_app.ball.error_cm;
    }
    if (g_app.ball.error_cm > g_app.interval_error_max_cm) {
        g_app.interval_error_max_cm = g_app.ball.error_cm;
    }

    if (absolute_error > H456_APP_BALL_ABORT_ERROR_CM) {
        if (!g_app.ball_abort_active) {
            g_app.ball_abort_active = true;
            g_app.ball_abort_start_ms = chassis->timestamp_ms;
        } else if ((chassis->timestamp_ms -
                    g_app.ball_abort_start_ms) >=
                   H456_APP_BALL_ABORT_HOLD_MS) {
            h456_start_recenter(H456_APP_FAULT_BALL_ERROR, chassis);
        }
    } else {
        g_app.ball_abort_active = false;
    }
}

static void h456_run_control(chassis_status_t *status)
{
    chassis_track_line_fusion_request_t request;
    float center_distance_mm;
    float heading_abs_deg;
    ml_status_t command_status;

    if (status->timestamp_ms == g_app.last_control_ms) {
        return;
    }
    g_app.last_control_ms = status->timestamp_ms;
    g_app.last_line = line_sensor_read();
    if (g_app.last_line.io_fault) {
        h456_start_recenter(H456_APP_FAULT_LINE_GPIO, status);
        return;
    }
    center_distance_mm = 0.5f *
        (status->pose.left_distance_mm + status->pose.right_distance_mm);
    command_status = h456_mission_update(&g_app.mission,
        center_distance_mm, status->measured_left_mm_s,
        status->measured_right_mm_s, status->fused_heading_deg,
        status->timestamp_ms, false, &g_app.mission_output);
    if (command_status != ML_STATUS_OK) {
        h456_start_recenter(H456_APP_FAULT_CHASSIS, status);
        return;
    }
    if (g_app.mission_output.state ==
        H456_MISSION_FAULT_LAP_GATE) {
        h456_start_recenter(H456_APP_FAULT_LAP_GATE, status);
        return;
    }
    if (g_app.mission_output.score_point_passed) {
        g_app.score_frozen = true;
    }
    if (g_app.mission_output.finished) {
        chassis_stop();
        g_app.velocity_started = false;
        h456_clear_launch_bias();
        h456_finish_telemetry(status);
        g_app.state = H456_APP_FINISHED;
        g_app.display_dirty = true;
        return;
    }

    request.linear_mm_s = g_app.mission_output.linear_mm_s;
    request.route_feedforward_rad_s =
        g_app.mission_output.route_feedforward_rad_s;
    request.heading_feedback_rad_s =
        g_app.mission_output.heading_feedback_rad_s;
    request.heading_error_deg = g_app.mission_output.heading_error_deg;
    heading_abs_deg = h456_app_abs(g_app.mission_output.heading_progress_deg);
    if (g_app.mode == H456_MODE_4) {
        if (heading_abs_deg >= H456_APP_H4_HEADING_ONLY_ENTER_DEG) {
            g_app.h4_heading_only_active = true;
        } else if (heading_abs_deg <= H456_APP_H4_HEADING_ONLY_EXIT_DEG) {
            g_app.h4_heading_only_active = false;
        }
    } else {
        g_app.h4_heading_only_active = false;
    }
    request.heading_only = g_app.h4_heading_only_active;
    command_status = chassis_track_line_control_update_fused(
        &g_app.line_control, &g_app.last_line,
        &request, &g_app.line_output);
    if (command_status == ML_STATUS_OK) {
        if (!g_app.velocity_started) {
            command_status = chassis_set_velocity(
                g_app.line_output.linear_mm_s,
                g_app.line_output.angular_rad_s);
            g_app.velocity_started = command_status == ML_STATUS_OK;
        } else {
            command_status = chassis_update_velocity(
                g_app.line_output.linear_mm_s,
                g_app.line_output.angular_rad_s);
        }
    }
    if (command_status != ML_STATUS_OK) {
        h456_start_recenter(H456_APP_FAULT_CHASSIS, status);
    }
}

ml_status_t h456_app_init(void)
{
    ml_status_t status;
    const char *failure_detail = "CHECK MODULES";

    memset(&g_app, 0, sizeof(g_app));
    g_app.mode = H456_MODE_4;
    chassis_key_init(&g_app.center_key);
    chassis_key_init(&g_app.up_key);
    chassis_key_init(&g_app.down_key);
    chassis_key_init(&g_app.left_key);
    chassis_key_init(&g_app.right_key);
    status = OLED_Init();
    if (status == ML_STATUS_OK) {
        h456_show_boot();
    }
    if (status == ML_STATUS_OK) {
        status = h456_keys_init();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_init();
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(UART0,
            H456_APP_UART_BAUD, H456_APP_UART_PRIORITY);
    }
    if (status == ML_STATUS_OK) {
        status = h456_telemetry_init();
    }
    if (status == ML_STATUS_OK) {
        status = chassis_init(&g_chassis_race_config);
    }
    if (status == ML_STATUS_OK) {
        status = h456_mission_init(&g_app.mission,
            &g_h456_mission_default_config);
    }
    if (status == ML_STATUS_OK) {
        failure_detail = "LINE CFG FAIL";
        status = h456_configure_line_control(g_app.mode);
    }
    if (status == ML_STATUS_OK) {
        failure_detail = "CHECK MODULES";
        status = icm42688_service_init(
            &g_app.imu, &g_h456_imu_config);
    }
    if (status == ML_STATUS_OK) {
        status = ball_balance_init();
    }
    if (status == ML_STATUS_OK) {
        status = ball_balance_set_target_cm(0.0f);
    }
    if (status == ML_STATUS_OK) {
        status = ball_balance_enable(true);
    }
    if (status != ML_STATUS_OK) {
        chassis_emergency_stop();
        g_app.fault = H456_APP_FAULT_INIT;
        g_app.state = H456_APP_FAULT;
        h456_show_init_failure(status, failure_detail);
        return status;
    }
    line_sensor_white_guard_reset(&g_app.white_guard);
    g_app.state = H456_APP_CALIBRATING;
    g_app.initialized = true;
    g_app.display_dirty = true;
    h456_require_key_release();
    (void) OLED_Clear();
    return ML_STATUS_OK;
}

void h456_app_poll(void)
{
    icm42688_service_event_t imu_event;
    chassis_status_t status;
    ml_status_t command_status;
    uint8_t uart_byte;
    bool center_press;

    if (!g_app.initialized) {
        return;
    }
    ball_balance_process();
    (void) ball_balance_get_status(&g_app.ball);
    imu_event = icm42688_service_poll(
        &g_app.imu, &g_app.imu_output);
    if ((imu_event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED) ||
        (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE)) {
        chassis_set_imu_sample(g_app.imu_output.angles.yaw_deg,
            g_app.imu_output.body_gyro_z_dps,
            g_app.imu_output.timestamp_ms, true);
    }
    if (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE) {
        g_app.imu_ready = true;
    }
    chassis_poll();
    status = chassis_get_status();
    h456_update_keys(status.timestamp_ms);
    center_press = chassis_key_take_press(&g_app.center_key);

    if ((g_app.state == H456_APP_CALIBRATING) &&
        ((status.timestamp_ms - g_app.last_white_sample_ms) >=
         g_h456_mission_default_config.control_period_ms)) {
        g_app.last_white_sample_ms = status.timestamp_ms;
        g_app.last_line = line_sensor_read();
        if (g_app.last_line.io_fault) {
            h456_start_recenter(H456_APP_FAULT_LINE_GPIO, &status);
        } else {
            (void) line_sensor_white_guard_update(
                &g_app.white_guard, g_app.last_line.raw_bits);
        }
    }
    if ((g_app.state == H456_APP_CALIBRATING) &&
        g_app.imu_ready &&
        (g_app.white_guard.consecutive_samples >=
         LINE_SENSOR_WHITE_STABLE_SAMPLES)) {
        g_app.state = H456_APP_SETUP;
        h456_require_key_release();
        g_app.display_dirty = true;
    }

    if ((g_app.state == H456_APP_SETUP) ||
        (g_app.state == H456_APP_READY)) {
        if (g_app.ball.vision_ready && !g_app.ball.enabled &&
            !g_app.ball.breakaway_fault) {
            (void) ball_balance_enable(true);
            (void) ball_balance_get_status(&g_app.ball);
        }
        if (chassis_key_take_press(&g_app.up_key)) {
            h456_select_next_mode(true);
        }
        if (chassis_key_take_press(&g_app.down_key)) {
            h456_select_next_mode(false);
        }
        if (chassis_key_take_press(&g_app.left_key)) {
            h456_adjust_h6_target(-H456_APP_H6_TARGET_STEP_CM);
        }
        if (chassis_key_take_press(&g_app.right_key)) {
            h456_adjust_h6_target(H456_APP_H6_TARGET_STEP_CM);
        }
        h456_update_ball_settle(status.timestamp_ms);
        if ((g_app.state == H456_APP_READY) && center_press) {
            command_status = h456_start_run(&status);
            if (command_status != ML_STATUS_OK) {
                h456_start_recenter(H456_APP_FAULT_CHASSIS, &status);
            }
        }
    } else if ((g_app.state == H456_APP_RUNNING) && center_press) {
        h456_start_recenter(H456_APP_FAULT_EMERGENCY, &status);
    } else if ((g_app.state == H456_APP_FINISHED) && center_press) {
        h456_clear_launch_bias();
        (void) ball_balance_enable(false);
        g_app.state = H456_APP_RECENTERING;
        g_app.display_dirty = true;
    }

    if (g_app.state == H456_APP_RUNNING) {
        h456_update_launch_bias(status.timestamp_ms);
        if ((status.result == CHASSIS_RESULT_FAULT) ||
            status.emergency_stop_latched) {
            h456_start_recenter(H456_APP_FAULT_CHASSIS, &status);
        } else if (g_app.ball.breakaway_fault) {
            h456_start_recenter(
                H456_APP_FAULT_BALL_CONTROL, &status);
        } else if (!g_app.ball.vision_ready || !g_app.ball.enabled) {
            h456_start_recenter(H456_APP_FAULT_VISION, &status);
        } else {
            h456_update_score_and_safety(&status);
            if (g_app.state == H456_APP_RUNNING) {
                h456_run_control(&status);
            }
            if (g_app.state == H456_APP_RUNNING) {
                h456_record_telemetry(&status, false);
            }
        }
    }

    if (g_app.state == H456_APP_RECENTERING) {
        if ((g_app.ball.servo_target_us == BALL_SERVO_CENTER_US) &&
            (g_app.ball.servo_current_us == BALL_SERVO_CENTER_US)) {
            g_app.state = H456_APP_EXPORT_READY;
            g_app.display_dirty = true;
        }
    }

    while (uart_try_read(UART0, &uart_byte) == ML_STATUS_OK) {
        (void) h456_telemetry_uart0_handle_byte(uart_byte,
            g_app.state == H456_APP_EXPORT_READY,
            status.timestamp_ms);
    }

    if (g_app.state != H456_APP_RUNNING &&
        (g_app.display_dirty ||
         ((status.timestamp_ms - g_app.last_display_ms) >=
          H456_APP_DISPLAY_PERIOD_MS))) {
        h456_show();
        g_app.last_display_ms = status.timestamp_ms;
        g_app.display_dirty = false;
    }
}

ml_status_t h456_app_get_status(h456_app_status_t *status)
{
    if (status == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_app.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    memset(status, 0, sizeof(*status));
    status->state = g_app.state;
    status->fault = g_app.fault;
    status->mode = g_app.mode;
    status->target_cm = g_app.target_cm;
    status->ball_position_cm = g_app.ball.position_cm;
    status->ball_error_cm = g_app.ball.error_cm;
    status->maximum_score_error_cm = g_app.maximum_score_error_cm;
    status->score_elapsed_ms = g_app.mission_output.score_elapsed_ms;
    status->telemetry_count = h456_telemetry_count();
    status->imu_ready = g_app.imu_ready;
    status->white_ready =
        g_app.white_guard.consecutive_samples >=
        LINE_SENSOR_WHITE_STABLE_SAMPLES;
    status->vision_ready = g_app.ball.vision_ready;
    status->ball_settled = g_app.ball_settled;
    status->ball_violation = g_app.ball_violation;
    status->score_point_passed =
        g_app.mission_output.score_point_passed;
    status->export_allowed = g_app.state == H456_APP_EXPORT_READY;
    status->oled_frozen = g_app.state == H456_APP_RUNNING;
    return ML_STATUS_OK;
}
