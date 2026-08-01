#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "h456_app.h"
#include "h456_telemetry.h"

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;
UART_Regs g_test_uart0;
GPTIMER_Regs g_test_timg8;

const chassis_config_t g_chassis_race_config = {
    214.2f, 500.0f, 20U
};
const chassis_track_line_control_config_t
    g_chassis_track_line_control_default_config = {
        214.2f, 500.0f, 90.0f, 0.22f, 120.0f, 0.35f,
        110.0f, 0.31f, 20U, 0U
    };

static chassis_status_t g_chassis;
static ball_balance_status_t g_ball;
static chassis_track_line_control_config_t g_line_config;
static uint32_t g_pressed_a;
static uint32_t g_pressed_b;
static uint32_t g_oled_writes;
static uint32_t g_velocity_commands;
static float g_control_bias_us;
static float g_imu_yaw_deg;
static float g_last_angular_rad_s;
static float g_last_heading_feedback_rad_s;
static float g_forced_line_bias_mm_s;
static ml_status_t g_line_init_status;
static ml_status_t g_icm_init_status;
static bool g_imu_complete_sent;
static bool g_chassis_emergency;
static bool g_last_heading_only;
static bool g_force_line_bias;
static char g_oled_lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_COLUMN_COUNT + 1U];

static void reset_mocks(void)
{
    memset(&g_chassis, 0, sizeof(g_chassis));
    memset(&g_ball, 0, sizeof(g_ball));
    g_pressed_a = 0U;
    g_pressed_b = 0U;
    g_oled_writes = 0U;
    g_velocity_commands = 0U;
    g_control_bias_us = 0.0f;
    g_imu_yaw_deg = 0.0f;
    g_last_angular_rad_s = 0.0f;
    g_last_heading_feedback_rad_s = 0.0f;
    g_forced_line_bias_mm_s = 0.0f;
    g_line_init_status = ML_STATUS_OK;
    g_icm_init_status = ML_STATUS_OK;
    g_imu_complete_sent = false;
    g_chassis_emergency = false;
    g_last_heading_only = false;
    g_force_line_bias = false;
    memset(&g_line_config, 0, sizeof(g_line_config));
    memset(g_oled_lines, 0, sizeof(g_oled_lines));
}

ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner)
{
    assert(resource == ML_BOARD_RESOURCE_PB24);
    assert(owner == ML_BOARD_OWNER_KEY);
    return ML_STATUS_OK;
}

ml_status_t gpio_init(GPIO_Regs *gpio, uint32_t pins,
    GPIOn_enum gpion, GPIO_Mode_enum mode)
{
    (void) gpio;
    (void) pins;
    (void) gpion;
    assert(mode == IN_UP);
    return ML_STATUS_OK;
}

uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins)
{
    uint32_t pressed = gpio == GPIOA ? g_pressed_a : g_pressed_b;

    return (pressed & pins) != 0U ? 0U : 1U;
}

ml_status_t OLED_Init(void)
{
    return ML_STATUS_OK;
}

ml_status_t OLED_Clear(void)
{
    return ML_STATUS_OK;
}

ml_status_t OLED_ShowLine(uint8_t line, const char *string)
{
    assert(line >= 1U && line <= 4U);
    assert(string != 0);
    strncpy(g_oled_lines[line - 1U], string, OLED_TEXT_COLUMN_COUNT);
    g_oled_lines[line - 1U][OLED_TEXT_COLUMN_COUNT] = '\0';
    ++g_oled_writes;
    return ML_STATUS_OK;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    assert(uart == UART0);
    assert(baud == 115200UL);
    assert(priority == 2U);
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    assert(uart == UART0);
    assert(byte != 0);
    return ML_STATUS_BUFFER_EMPTY;
}

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    (void) byte;
    assert(uart == UART0);
    return ML_STATUS_OK;
}

ml_status_t line_sensor_init(void)
{
    return ML_STATUS_OK;
}

ml_status_t line_sensor_reassert_inputs(void)
{
    return ML_STATUS_OK;
}

line_sample_t line_sensor_read(void)
{
    line_sample_t sample = {0x0FU, 0x06U, false, false};

    return sample;
}

void line_sensor_white_guard_reset(line_sensor_white_guard_t *guard)
{
    guard->consecutive_samples = 0U;
}

bool line_sensor_white_guard_update(
    line_sensor_white_guard_t *guard, uint8_t raw_bits)
{
    assert(raw_bits == 0x0FU);
    if (guard->consecutive_samples < LINE_SENSOR_WHITE_STABLE_SAMPLES) {
        ++guard->consecutive_samples;
    }
    return guard->consecutive_samples >= LINE_SENSOR_WHITE_STABLE_SAMPLES;
}

ml_status_t chassis_init(const chassis_config_t *config)
{
    assert(config == &g_chassis_race_config);
    g_chassis.result = CHASSIS_RESULT_IDLE;
    return ML_STATUS_OK;
}

void chassis_poll(void)
{
    float center_target = 0.5f *
        (g_chassis.target_left_mm_s + g_chassis.target_right_mm_s);

    g_chassis.timestamp_ms += 20U;
    if (g_velocity_commands > 0U && !g_chassis_emergency) {
        g_chassis.pose.left_distance_mm += center_target * 0.020f;
        g_chassis.pose.right_distance_mm += center_target * 0.020f;
        g_chassis.measured_left_mm_s = g_chassis.target_left_mm_s;
        g_chassis.measured_right_mm_s = g_chassis.target_right_mm_s;
        g_chassis.result = CHASSIS_RESULT_RUNNING;
    }
}

chassis_status_t chassis_get_status(void)
{
    return g_chassis;
}

void chassis_set_imu_sample(float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid)
{
    (void) body_gyro_z_dps;
    (void) timestamp_ms;
    assert(valid);
    g_chassis.fused_heading_deg = yaw_deg;
}

void chassis_reset_pose(float x_mm, float y_mm, float heading_deg)
{
    (void) x_mm;
    (void) y_mm;
    (void) heading_deg;
    g_chassis.pose.left_distance_mm = 0.0f;
    g_chassis.pose.right_distance_mm = 0.0f;
}

ml_status_t chassis_set_velocity(float linear_mm_s, float angular_rad_s)
{
    float steering_bias_mm_s = angular_rad_s *
        g_chassis_race_config.effective_track_mm * 0.5f;

    g_last_angular_rad_s = angular_rad_s;
    g_chassis.target_left_mm_s = linear_mm_s - steering_bias_mm_s;
    g_chassis.target_right_mm_s = linear_mm_s + steering_bias_mm_s;
    ++g_velocity_commands;
    return ML_STATUS_OK;
}

ml_status_t chassis_update_velocity(float linear_mm_s, float angular_rad_s)
{
    return chassis_set_velocity(linear_mm_s, angular_rad_s);
}

void chassis_stop(void)
{
    g_chassis.target_left_mm_s = 0.0f;
    g_chassis.target_right_mm_s = 0.0f;
}

void chassis_emergency_stop(void)
{
    g_chassis_emergency = true;
    g_chassis.emergency_stop_latched = true;
    chassis_stop();
}

ml_status_t chassis_track_line_control_init(
    chassis_track_line_control_t *control,
    const chassis_track_line_control_config_t *config)
{
    assert(fabsf(config->effective_track_mm - 214.2f) < 0.01f);
    assert(fabsf(config->maximum_wheel_speed_mm_s - 500.0f) < 0.01f);
    assert(config->control_period_ms == 20U);
    if (g_line_init_status != ML_STATUS_OK) {
        return g_line_init_status;
    }
    assert(config->curve_hold_maximum_correction_mm_s <=
        config->outer_single_maximum_correction_mm_s);
    assert(config->curve_hold_correction_ratio <=
        config->outer_single_correction_ratio);
    g_line_config = *config;
    control->initialized = true;
    return ML_STATUS_OK;
}

void chassis_track_line_control_reset(
    chassis_track_line_control_t *control)
{
    assert(control->initialized);
}

ml_status_t chassis_track_line_control_update_fused(
    chassis_track_line_control_t *control,
    const line_sample_t *sample,
    const chassis_track_line_fusion_request_t *request,
    chassis_track_line_control_output_t *output)
{
    assert(control->initialized);
    assert(!sample->io_fault);
    memset(output, 0, sizeof(*output));
    output->linear_mm_s = request->linear_mm_s;
    output->final_steering_bias_mm_s =
        g_force_line_bias ? g_forced_line_bias_mm_s : 0.0f;
    output->left_mm_s =
        request->linear_mm_s - output->final_steering_bias_mm_s;
    output->right_mm_s =
        request->linear_mm_s + output->final_steering_bias_mm_s;
    output->angular_rad_s =
        (output->right_mm_s - output->left_mm_s) /
        g_chassis_race_config.effective_track_mm;
    output->line_valid = true;
    g_last_heading_only = request->heading_only;
    g_last_heading_feedback_rad_s = request->heading_feedback_rad_s;
    return ML_STATUS_OK;
}

ml_status_t icm42688_service_init(
    icm42688_service_t *context,
    const icm42688_service_config_t *config)
{
    (void) context;
    if (g_icm_init_status != ML_STATUS_OK) {
        return g_icm_init_status;
    }
    assert(config->timer == TIMG8);
    assert(config->axis_config.calibration_samples_required == 150U);
    return ML_STATUS_OK;
}

icm42688_service_event_t icm42688_service_poll(
    icm42688_service_t *context,
    icm42688_service_output_t *output)
{
    (void) context;
    memset(output, 0, sizeof(*output));
    output->timestamp_ms = g_chassis.timestamp_ms;
    output->angles.yaw_deg = g_imu_yaw_deg;
    if (!g_imu_complete_sent) {
        g_imu_complete_sent = true;
        return ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE;
    }
    return ICM42688_SERVICE_EVENT_ANGLES_UPDATED;
}

ml_status_t ball_balance_init(void)
{
    memset(&g_ball, 0, sizeof(g_ball));
    g_ball.state = BALL_BALANCE_WAITING_FOR_VISION;
    g_ball.sequence_state = BALL_SEQUENCE_IDLE;
    g_ball.servo_current_us = BALL_SERVO_CENTER_US;
    g_ball.servo_target_us = BALL_CONTROL_NEUTRAL_US;
    return ML_STATUS_OK;
}

void ball_balance_process(void)
{
    g_ball.uptime_ms += 20U;
    g_ball.servo_current_us = g_ball.servo_target_us;
    g_ball.error_cm = g_ball.target_cm - g_ball.position_cm;
    if (g_ball.vision_ready && g_ball.enabled) {
        g_ball.state = BALL_BALANCE_ACTIVE;
    }
}

ml_status_t ball_balance_enable(bool enable)
{
    g_ball.enabled = enable;
    g_ball.control_mode = enable ?
        BALL_CONTROL_CASCADE : BALL_CONTROL_DISABLED;
    if (enable) {
        g_ball.servo_target_us = BALL_CONTROL_NEUTRAL_US;
        g_ball.state = g_ball.vision_ready ?
            BALL_BALANCE_ACTIVE : BALL_BALANCE_WAITING_FOR_VISION;
    } else {
        g_control_bias_us = 0.0f;
        g_ball.servo_target_us = BALL_SERVO_CENTER_US;
        g_ball.state = BALL_BALANCE_DISABLED;
    }
    return ML_STATUS_OK;
}

ml_status_t ball_balance_set_control_bias_us(float bias_us)
{
    g_control_bias_us = bias_us;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_set_target_cm(float target_cm)
{
    g_ball.target_cm = target_cm;
    g_ball.error_cm = target_cm - g_ball.position_cm;
    return ML_STATUS_OK;
}

ml_status_t ball_balance_get_status(ball_balance_status_t *status)
{
    *status = g_ball;
    return ML_STATUS_OK;
}

static void poll_count(uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; ++i) {
        h456_app_poll();
    }
}

static void press_key(GPIO_Regs *port, uint32_t pin)
{
    uint32_t *pressed = port == GPIOA ? &g_pressed_a : &g_pressed_b;

    *pressed |= pin;
    poll_count(4U);
    *pressed &= ~pin;
    poll_count(1U);
}

static void hold_key(GPIO_Regs *port, uint32_t pin)
{
    uint32_t *pressed = port == GPIOA ? &g_pressed_a : &g_pressed_b;

    *pressed |= pin;
}

static void release_key(GPIO_Regs *port, uint32_t pin)
{
    uint32_t *pressed = port == GPIOA ? &g_pressed_a : &g_pressed_b;

    *pressed &= ~pin;
}

static h456_app_status_t get_status(void)
{
    h456_app_status_t status;

    assert(h456_app_get_status(&status) == ML_STATUS_OK);
    return status;
}

static float commanded_steering_bias_mm_s(void)
{
    return g_last_angular_rad_s *
        g_chassis_race_config.effective_track_mm * 0.5f;
}

static void prepare_ready(void)
{
    h456_app_status_t status;

    g_ball.vision_ready = true;
    g_ball.position_cm = g_ball.target_cm;
    poll_count(12U);
    poll_count(27U);
    status = get_status();
    assert(status.imu_ready);
    assert(status.white_ready);
    assert(status.vision_ready);
    assert(status.state == H456_APP_READY);
}

static void select_h4_ready(void)
{
    press_key(GPIOA, ML_KEY_UP_PIN);
    assert(get_status().mode == H456_MODE_4);
    poll_count(27U);
    assert(get_status().state == H456_APP_READY);
}

static void select_h5_ready(void)
{
    select_h4_ready();
    press_key(GPIOA, ML_KEY_UP_PIN);
    assert(get_status().mode == H456_MODE_5);
    poll_count(27U);
    assert(get_status().state == H456_APP_READY);
}

static void assert_h4_line_config(void)
{
    assert(fabsf(g_line_config.correction_ratio - 0.18f) < 0.001f);
    assert(fabsf(g_line_config.maximum_correction_mm_s - 60.0f) < 0.001f);
    assert(fabsf(g_line_config.outer_single_correction_ratio - 0.28f) <
        0.001f);
    assert(fabsf(g_line_config.outer_single_maximum_correction_mm_s -
        80.0f) < 0.001f);
    assert(fabsf(g_line_config.curve_hold_correction_ratio - 0.18f) <
        0.001f);
    assert(fabsf(g_line_config.curve_hold_maximum_correction_mm_s -
        60.0f) < 0.001f);
    assert(g_line_config.curve_exit_fade_ms == 0U);
    assert(g_line_config.curve_hold_maximum_correction_mm_s <=
        g_line_config.outer_single_maximum_correction_mm_s);
}

static void assert_h5_h6_line_config(void)
{
    assert(fabsf(g_line_config.correction_ratio - 0.20f) < 0.001f);
    assert(fabsf(g_line_config.maximum_correction_mm_s - 70.0f) < 0.001f);
    assert(fabsf(g_line_config.outer_single_correction_ratio - 0.22f) <
        0.001f);
    assert(fabsf(g_line_config.outer_single_maximum_correction_mm_s -
        60.0f) < 0.001f);
    assert(fabsf(g_line_config.curve_hold_correction_ratio - 0.16f) <
        0.001f);
    assert(fabsf(g_line_config.curve_hold_maximum_correction_mm_s -
        50.0f) < 0.001f);
    assert(g_line_config.curve_exit_fade_ms == 120U);
}

static void prepare_setup_without_vision(void)
{
    h456_app_status_t status;

    poll_count(12U);
    status = get_status();
    assert(status.imu_ready);
    assert(status.white_ready);
    assert(!status.vision_ready);
    assert(status.state == H456_APP_SETUP);
}

static void test_calibrating_display_is_not_fault_locked(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    poll_count(1U);
    status = get_status();
    assert(status.state == H456_APP_CALIBRATING);
    assert(strcmp(g_oled_lines[0], "H456 CALIBRATE  ") == 0);
    assert(strcmp(g_oled_lines[3], "WAIT READY SAFE ") == 0);
    assert(g_velocity_commands == 0U);
    assert(!g_chassis_emergency);
}

static void test_init_failure_keeps_oled_diagnostic_visible(void)
{
    h456_app_status_t status;

    reset_mocks();
    g_icm_init_status = ML_STATUS_DEVICE_NOT_FOUND;
    assert(h456_app_init() == ML_STATUS_DEVICE_NOT_FOUND);
    assert(strcmp(g_oled_lines[0], "H456 INIT FAIL  ") == 0);
    assert(strcmp(g_oled_lines[1], "STATUS 09       ") == 0);
    assert(strcmp(g_oled_lines[2], "MOTORS SAFE     ") == 0);
    assert(strcmp(g_oled_lines[3], "CHECK MODULES   ") == 0);
    assert(g_chassis_emergency);
    assert(h456_app_get_status(&status) == ML_STATUS_NOT_INITIALIZED);
}

static void test_line_config_failure_reports_specific_oled_reason(void)
{
    h456_app_status_t status;

    reset_mocks();
    g_line_init_status = ML_STATUS_INVALID_ARGUMENT;
    assert(h456_app_init() == ML_STATUS_INVALID_ARGUMENT);
    assert(strcmp(g_oled_lines[0], "H456 INIT FAIL  ") == 0);
    assert(strcmp(g_oled_lines[1], "STATUS 01       ") == 0);
    assert(strcmp(g_oled_lines[2], "MOTORS SAFE     ") == 0);
    assert(strcmp(g_oled_lines[3], "LINE CFG FAIL   ") == 0);
    assert(g_chassis_emergency);
    assert(h456_app_get_status(&status) == ML_STATUS_NOT_INITIALIZED);
}

static void test_setup_display_reports_wait_reason(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_setup_without_vision();
    assert(get_status().mode == H456_MODE_6);
    assert(strcmp(g_oled_lines[3], "WAIT VISION     ") == 0);
    assert(g_velocity_commands == 0U);

    g_ball.vision_ready = true;
    g_ball.position_cm = 13.0f;
    poll_count(5U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(strcmp(g_oled_lines[3], "WAIT <=12.5CM   ") == 0);

    press_key(GPIOA, ML_KEY_UP_PIN);
    assert(get_status().mode == H456_MODE_4);
    g_ball.position_cm = 1.0f;
    poll_count(5U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(strcmp(g_oled_lines[3], "WAIT POS <=0.8  ") == 0);

    g_ball.position_cm = g_ball.target_cm;
    g_ball.velocity_cm_per_s = 1.1f;
    poll_count(5U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(strcmp(g_oled_lines[3], "WAIT SPD <=1.0  ") == 0);

    g_ball.velocity_cm_per_s = 0.0f;
    poll_count(5U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(strcmp(g_oled_lines[3], "SETTLE 500MS    ") == 0);

    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(strcmp(g_oled_lines[3], "C=GO U/D=MODE   ") == 0);
}

static void test_default_h6_tracks_current_position_before_ready(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    g_ball.vision_ready = true;
    g_ball.position_cm = 2.0f;
    poll_count(1U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_CALIBRATING);
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    assert(fabsf(status.ball_error_cm) < 0.01f);
    assert(g_ball.enabled);
    assert(g_velocity_commands == 0U);

    poll_count(38U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    assert(strcmp(g_oled_lines[2], "AUTO LOCK PB24  ") == 0);
    assert(strcmp(g_oled_lines[3], "C=GO U/D=MODE   ") == 0);
}

static void test_center_held_through_ready_does_not_start(void)
{
    h456_app_status_t status;

    reset_mocks();
    hold_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(h456_app_init() == ML_STATUS_OK);
    g_ball.vision_ready = true;
    g_ball.position_cm = 2.0f;
    poll_count(50U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    assert(g_velocity_commands == 0U);
    assert(strcmp(g_oled_lines[3], "REL PB24        ") == 0);

    release_key(GPIOB, ML_KEY_CENTER_PIN);
    poll_count(5U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(g_velocity_commands == 0U);
    assert(strcmp(g_oled_lines[3], "C=GO U/D=MODE   ") == 0);

    g_ball.position_cm = 2.3f;
    poll_count(1U);
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(fabsf(status.target_cm - 2.3f) < 0.01f);
    assert(fabsf(status.ball_error_cm) < 0.01f);
}

static void test_mode_target_start_freeze_and_emergency(void)
{
    h456_app_status_t status;
    uint32_t oled_at_start;
    float locked_target;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(g_velocity_commands == 0U);
    g_ball.position_cm = 2.0f;
    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    assert(g_velocity_commands == 0U);
    assert(strcmp(g_oled_lines[2], "AUTO LOCK PB24  ") == 0);
    press_key(GPIOB, ML_KEY_LEFT_PIN);
    status = get_status();
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    press_key(GPIOB, ML_KEY_RIGHT_PIN);
    status = get_status();
    assert(fabsf(status.target_cm - 2.0f) < 0.01f);
    assert(g_velocity_commands == 0U);
    g_ball.position_cm = 2.3f;
    poll_count(1U);
    assert(get_status().state == H456_APP_READY);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(status.oled_frozen);
    assert(fabsf(status.target_cm - 2.3f) < 0.01f);
    assert(fabsf(status.ball_error_cm) < 0.01f);
    locked_target = status.target_cm;
    assert_h5_h6_line_config();
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us <= 60.0f);
    assert(g_velocity_commands > 0U);
    oled_at_start = g_oled_writes;
    poll_count(8U);
    assert(g_oled_writes == oled_at_start);
    assert(g_control_bias_us > 0.0f);
    poll_count(40U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 60.0f);
    poll_count(50U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 60.0f);
    poll_count(25U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 60.0f);
    poll_count(30U);
    assert(g_control_bias_us == 0.0f);
    assert(g_oled_writes == oled_at_start);

    g_ball.position_cm = locked_target - 1.2f;
    poll_count(1U);
    status = get_status();
    assert(status.ball_violation);
    assert(status.state == H456_APP_RUNNING);
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    poll_count(2U);
    status = get_status();
    assert(status.fault == H456_APP_FAULT_EMERGENCY);
    assert(status.export_allowed);
    assert(g_chassis_emergency);
    assert(g_control_bias_us == 0.0f);
    assert(g_ball.servo_current_us == BALL_SERVO_CENTER_US);
}

static void test_h6_positive_target_uses_stronger_launch_bias(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = 6.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 6.0f) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(fabsf(status.target_cm - 6.0f) < 0.01f);
    assert(g_control_bias_us > 150.0f);
    assert(g_control_bias_us <= 160.0f);

    poll_count(120U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 160.0f);
    poll_count(40U);
    assert(g_control_bias_us == 0.0f);
}

static void test_h6_far_positive_target_uses_extended_launch_bias(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = 9.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 9.0f) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(fabsf(status.target_cm - 9.0f) < 0.01f);
    assert(g_control_bias_us > 180.0f);
    assert(g_control_bias_us <= 190.0f);

    poll_count(150U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 190.0f);
    poll_count(110U);
    assert(g_control_bias_us == 0.0f);
}

static void test_h6_center_target_keeps_mild_launch_bias(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = 0.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us <= 60.0f);
}

static void test_h6_negative_targets_use_corrected_positive_launch_bias(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = -4.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm + 4.0f) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(g_control_bias_us > 50.0f);
    assert(g_control_bias_us <= 60.0f);

    poll_count(150U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 60.0f);
    poll_count(110U);
    assert(g_control_bias_us == 0.0f);

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = -6.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm + 6.0f) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(g_control_bias_us > 50.0f);
    assert(g_control_bias_us <= 60.0f);

    poll_count(150U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 60.0f);
    poll_count(110U);
    assert(g_control_bias_us == 0.0f);

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    g_ball.position_cm = -9.0f;
    poll_count(27U);
    status = get_status();
    assert(status.mode == H456_MODE_6);
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm + 9.0f) < 0.01f);

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_RUNNING);
    assert(g_control_bias_us > 70.0f);
    assert(g_control_bias_us <= 80.0f);

    poll_count(150U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 80.0f);
    poll_count(151U);
    assert(g_control_bias_us == 0.0f);
}

static void test_h6_position_range_gate(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    assert(get_status().mode == H456_MODE_6);

    g_ball.position_cm = 12.6f;
    g_ball.velocity_cm_per_s = 0.0f;
    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(strcmp(g_oled_lines[3], "WAIT <=12.5CM   ") == 0);
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(g_velocity_commands == 0U);

    g_ball.position_cm = 12.5f;
    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm - 12.5f) < 0.01f);

    g_ball.position_cm = -12.5f;
    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_READY);
    assert(fabsf(status.target_cm + 12.5f) < 0.01f);

    g_ball.position_cm = -12.6f;
    poll_count(27U);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    status = get_status();
    assert(status.state == H456_APP_SETUP);
    assert(g_velocity_commands == 0U);
}

static void test_h4_line_config_and_heading_protection(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h4_ready();
    assert_h4_line_config();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);
    assert_h4_line_config();
    assert(!g_last_heading_only);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us <= 100.0f);

    g_imu_yaw_deg = 7.0f;
    poll_count(1U);
    assert(g_last_heading_only);
    assert(fabsf(g_last_heading_feedback_rad_s + 0.35f) < 0.001f);

    g_imu_yaw_deg = 4.0f;
    poll_count(1U);
    assert(g_last_heading_only);

    g_imu_yaw_deg = 2.5f;
    poll_count(1U);
    assert(!g_last_heading_only);

    status = get_status();
    assert(status.state == H456_APP_RUNNING);
}

static void test_h5_uses_lap_line_config_and_no_h4_heading_only(void)
{
    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h5_ready();
    assert_h5_h6_line_config();

    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);
    g_imu_yaw_deg = 7.0f;
    poll_count(1U);
    assert(!g_last_heading_only);
    assert_h5_h6_line_config();
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us <= 100.0f);
    poll_count(150U);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us < 100.0f);
    poll_count(151U);
    assert(g_control_bias_us == 0.0f);
}

static void test_h5_cd_protection_limits_heading_feedback(void)
{
    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h5_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);

    g_imu_yaw_deg = 210.0f;
    g_chassis.pose.left_distance_mm = 3000.0f;
    g_chassis.pose.right_distance_mm = 3000.0f;
    poll_count(1U);
    assert(fabsf(g_last_heading_feedback_rad_s + 0.35f) < 0.001f);

    g_chassis.pose.left_distance_mm = 3500.0f;
    g_chassis.pose.right_distance_mm = 3500.0f;
    poll_count(1U);
    assert(fabsf(g_last_heading_feedback_rad_s + 0.16f) < 0.001f);

    g_chassis.pose.left_distance_mm = 4300.0f;
    g_chassis.pose.right_distance_mm = 4300.0f;
    poll_count(1U);
    assert(fabsf(g_last_heading_feedback_rad_s + 0.35f) < 0.001f);
}

static void test_h5_cd_protection_ramps_final_bias(void)
{
    float first_bias;
    float second_bias;
    float exit_bias;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h5_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);

    g_force_line_bias = true;
    g_forced_line_bias_mm_s = 9.0f;
    g_chassis.pose.left_distance_mm = 3500.0f;
    g_chassis.pose.right_distance_mm = 3500.0f;
    poll_count(1U);
    first_bias = commanded_steering_bias_mm_s();
    assert(fabsf(first_bias - 9.0f) < 0.001f);

    g_forced_line_bias_mm_s = -48.0f;
    poll_count(1U);
    second_bias = commanded_steering_bias_mm_s();
    assert(fabsf((second_bias - first_bias) + 8.0f) < 0.01f);
    assert(second_bias > -48.0f);

    g_chassis.pose.left_distance_mm = 4300.0f;
    g_chassis.pose.right_distance_mm = 4300.0f;
    poll_count(1U);
    exit_bias = commanded_steering_bias_mm_s();
    assert(fabsf(exit_bias + 48.0f) < 0.01f);
}

static void test_launch_bias_clears_on_finish(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h4_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);
    assert(g_control_bias_us > 0.0f);
    assert(g_control_bias_us <= 100.0f);

    g_chassis.pose.left_distance_mm = 1500.0f;
    g_chassis.pose.right_distance_mm = 1500.0f;
    poll_count(120U);
    status = get_status();
    assert(status.state == H456_APP_FINISHED);
    assert(g_control_bias_us == 0.0f);
    assert(fabsf(status.target_cm) < 0.01f);
}

static void test_sustained_two_cm_error_stops(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h4_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);
    g_ball.position_cm = -2.1f;
    poll_count(7U);
    poll_count(2U);
    status = get_status();
    assert(status.fault == H456_APP_FAULT_BALL_ERROR);
    assert(status.export_allowed);
    assert(status.ball_violation);
    assert(g_control_bias_us == 0.0f);
}

static void test_score_boundary_confirmation_gate(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h4_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);

    g_ball.position_cm = -1.1f;
    poll_count(24U);
    status = get_status();
    assert(!status.ball_violation);
    assert(fabsf(status.maximum_score_error_cm - 1.1f) < 0.01f);

    g_ball.position_cm = 0.0f;
    poll_count(1U);
    assert(!get_status().ball_violation);

    g_ball.position_cm = -1.1f;
    poll_count(26U);
    status = get_status();
    assert(status.ball_violation);
    assert(fabsf(status.maximum_score_error_cm - 1.1f) < 0.01f);
}

static void test_score_immediate_gate_at_one_point_two_cm(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);

    g_ball.position_cm = -1.2f;
    poll_count(1U);
    status = get_status();
    assert(status.ball_violation);
    assert(fabsf(status.maximum_score_error_cm - 1.2f) < 0.01f);
}

static void test_ball_score_freezes_on_pass_cycle(void)
{
    h456_app_status_t status;

    reset_mocks();
    assert(h456_app_init() == ML_STATUS_OK);
    prepare_ready();
    select_h4_ready();
    press_key(GPIOB, ML_KEY_CENTER_PIN);
    assert(get_status().state == H456_APP_RUNNING);

    g_ball.position_cm = -0.8f;
    g_chassis.pose.left_distance_mm = 1500.0f;
    g_chassis.pose.right_distance_mm = 1500.0f;
    poll_count(1U);
    status = get_status();
    assert(status.score_point_passed);
    assert(!status.ball_violation);
    assert(fabsf(status.maximum_score_error_cm - 0.8f) < 0.01f);

    g_ball.position_cm = -1.5f;
    poll_count(1U);
    status = get_status();
    assert(!status.ball_violation);
    assert(fabsf(status.maximum_score_error_cm - 0.8f) < 0.01f);
}

int main(void)
{
    test_calibrating_display_is_not_fault_locked();
    test_init_failure_keeps_oled_diagnostic_visible();
    test_line_config_failure_reports_specific_oled_reason();
    test_setup_display_reports_wait_reason();
    test_default_h6_tracks_current_position_before_ready();
    test_center_held_through_ready_does_not_start();
    test_mode_target_start_freeze_and_emergency();
    test_h6_positive_target_uses_stronger_launch_bias();
    test_h6_far_positive_target_uses_extended_launch_bias();
    test_h6_center_target_keeps_mild_launch_bias();
    test_h6_negative_targets_use_corrected_positive_launch_bias();
    test_h6_position_range_gate();
    test_h4_line_config_and_heading_protection();
    test_h5_uses_lap_line_config_and_no_h4_heading_only();
    test_h5_cd_protection_limits_heading_feedback();
    test_h5_cd_protection_ramps_final_bias();
    test_launch_bias_clears_on_finish();
    test_sustained_two_cm_error_stops();
    test_score_boundary_confirmation_gate();
    test_score_immediate_gate_at_one_point_two_cm();
    test_ball_score_freezes_on_pass_cycle();
    printf("H456 app tests passed\n");
    return 0;
}
