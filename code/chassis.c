#include "chassis.h"

#include <math.h>

#include "chassis_motion.h"
#include "chassis_telemetry.h"
#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "ml_tim.h"
#include "motor_velocity.h"

#define CHASSIS_PI (3.14159265358979323846f)

static chassis_motion_t g_chassis;
static volatile uint8_t g_pending_cycles;
static uint32_t g_next_telemetry_ms;
static bool g_idle_capture_active;
static bool g_chassis_initialized;

static motor_velocity_wheel_config_t chassis_velocity_wheel_config(
    const chassis_wheel_config_t *wheel)
{
    motor_velocity_wheel_config_t result;

    result.kp = wheel->kp;
    result.ki = wheel->ki;
    result.kd = wheel->kd;
    result.feedforward = wheel->feedforward;
    result.output_limit = wheel->pid_output_limit;
    result.integral_limit = wheel->pid_integral_limit;
    return result;
}

static void chassis_timer_callback(void *context)
{
    (void) context;
    if (g_pending_cycles < UINT8_MAX) {
        ++g_pending_cycles;
    }
}

static uint8_t chassis_take_pending_cycles(void)
{
    uint32_t interrupt_state;
    uint8_t cycles;

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    cycles = g_pending_cycles;
    g_pending_cycles = 0U;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return cycles;
}

ml_status_t chassis_init(const chassis_config_t *config)
{
    motor_velocity_config_t velocity_config;
    ml_status_t status;

    if (config == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    g_chassis_initialized = false;
    g_pending_cycles = 0U;
    g_next_telemetry_ms = 100U;
    g_idle_capture_active = false;
    chassis_telemetry_init();
    status = chassis_motion_init(&g_chassis, config);
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_init();
    }
    if (status == ML_STATUS_OK) {
        status = ml_encoder_init();
    }
    velocity_config.motor_a =
        chassis_velocity_wheel_config(&config->left);
    velocity_config.motor_b =
        chassis_velocity_wheel_config(&config->right);
    velocity_config.filter_alpha = config->speed_filter_alpha;
    if (status == ML_STATUS_OK) {
        status = motor_velocity_init(&velocity_config);
    }
    if (status == ML_STATUS_OK) {
        status = tim_interrupt_ms_init_ex(TIMG0,
            config->control_period_ms, 1U,
            chassis_timer_callback, 0);
    }
    if (status == ML_STATUS_OK) {
        g_chassis_initialized = true;
    } else {
        (void) ml_motor_driver_stop_all();
    }
    return status;
}

ml_status_t chassis_set_wheel_speed(float left_mm_s, float right_mm_s)
{
    if ((left_mm_s == 0.0f) && (right_mm_s == 0.0f) &&
        g_chassis_initialized) {
        chassis_stop();
        return ML_STATUS_OK;
    }
    chassis_idle_capture_stop();
    return chassis_motion_set_wheel_speed(&g_chassis,
        left_mm_s, right_mm_s, CHASSIS_MODE_WHEEL_SPEED);
}

ml_status_t chassis_update_wheel_speed(
    float left_mm_s, float right_mm_s)
{
    if (!g_chassis_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    return chassis_motion_update_wheel_speed(&g_chassis,
        left_mm_s, right_mm_s, CHASSIS_MODE_WHEEL_SPEED);
}

ml_status_t chassis_set_velocity(float linear_mm_s, float angular_rad_s)
{
    float half_track;

    if (!g_chassis_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((linear_mm_s == 0.0f) && (angular_rad_s == 0.0f)) {
        chassis_stop();
        return ML_STATUS_OK;
    }
    chassis_idle_capture_stop();
    half_track = g_chassis.config.effective_track_mm * 0.5f;
    return chassis_motion_set_wheel_speed(&g_chassis,
        linear_mm_s - angular_rad_s * half_track,
        linear_mm_s + angular_rad_s * half_track,
        CHASSIS_MODE_VELOCITY);
}

ml_status_t chassis_update_velocity(float linear_mm_s, float angular_rad_s)
{
    float half_track;

    if (!g_chassis_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    half_track = g_chassis.config.effective_track_mm * 0.5f;
    return chassis_motion_update_wheel_speed(&g_chassis,
        linear_mm_s - angular_rad_s * half_track,
        linear_mm_s + angular_rad_s * half_track,
        CHASSIS_MODE_VELOCITY);
}

ml_status_t chassis_move_mm(float distance_mm, float max_speed_mm_s)
{
    chassis_idle_capture_stop();
    return chassis_motion_move(&g_chassis, distance_mm, max_speed_mm_s);
}

ml_status_t chassis_rotate_deg(float angle_deg, float max_angular_deg_s)
{
    chassis_idle_capture_stop();
    return chassis_motion_rotate(&g_chassis,
        angle_deg * CHASSIS_PI / 180.0f,
        max_angular_deg_s * CHASSIS_PI / 180.0f);
}

ml_status_t chassis_arc(
    float radius_mm, float angle_deg, float max_speed_mm_s)
{
    chassis_idle_capture_stop();
    return chassis_motion_arc(&g_chassis, radius_mm,
        angle_deg * CHASSIS_PI / 180.0f, max_speed_mm_s);
}

void chassis_stop(void)
{
    if (!g_chassis_initialized) {
        return;
    }
    chassis_idle_capture_stop();
    chassis_motion_stop(&g_chassis, false);
    (void) motor_velocity_reset();
    (void) ml_motor_driver_stop_all();
    g_chassis.status.pwm_left_count = 0U;
    g_chassis.status.pwm_right_count = 0U;
}

void chassis_emergency_stop(void)
{
    if (!g_chassis_initialized) {
        return;
    }
    chassis_idle_capture_stop();
    chassis_motion_stop(&g_chassis, true);
    (void) motor_velocity_reset();
    (void) ml_motor_driver_stop_all();
    g_chassis.status.pwm_left_count = 0U;
    g_chassis.status.pwm_right_count = 0U;
}

void chassis_reset_pose(float x_mm, float y_mm, float heading_deg)
{
    if (!g_chassis_initialized) {
        return;
    }
    (void) chassis_motion_reset_pose(&g_chassis, x_mm, y_mm,
        heading_deg * CHASSIS_PI / 180.0f);
}

chassis_status_t chassis_get_status(void)
{
    chassis_status_t empty = {
        .mode = CHASSIS_MODE_IDLE,
        .result = CHASSIS_RESULT_IDLE,
        .fault = CHASSIS_FAULT_NONE
    };

    return g_chassis_initialized ? g_chassis.status : empty;
}

ml_status_t chassis_idle_capture_start(void)
{
    if (!g_chassis_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (g_chassis.status.result == CHASSIS_RESULT_RUNNING) {
        return ML_STATUS_BUSY;
    }
    g_idle_capture_active = true;
    g_next_telemetry_ms = g_chassis.status.timestamp_ms + 100U;
    return ML_STATUS_OK;
}

void chassis_idle_capture_stop(void)
{
    if (!g_chassis_initialized) {
        return;
    }
    g_idle_capture_active = false;
    g_next_telemetry_ms = g_chassis.status.timestamp_ms + 100U;
}

bool chassis_idle_capture_active(void)
{
    return g_chassis_initialized && g_idle_capture_active;
}

ml_status_t chassis_capture_telemetry_now(void)
{
    float target_center_mm_s;
    float actual_center_mm_s;

    if (!g_chassis_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    target_center_mm_s = 0.5f *
        (g_chassis.status.target_left_mm_s +
         g_chassis.status.target_right_mm_s);
    actual_center_mm_s = 0.5f *
        (g_chassis.status.measured_left_mm_s +
         g_chassis.status.measured_right_mm_s);
    return chassis_telemetry_record(
        g_chassis.status.timestamp_ms,
        g_chassis.status.encoder_total_left,
        g_chassis.status.encoder_total_right,
        g_chassis.status.pose.x_mm,
        g_chassis.status.pose.y_mm,
        g_chassis.status.encoder_heading_deg,
        g_chassis.status.fused_heading_deg,
        g_chassis.status.imu_yaw_deg,
        g_chassis.status.fused_yaw_rate_dps,
        target_center_mm_s,
        actual_center_mm_s,
        g_chassis.status.pwm_left_count,
        g_chassis.status.pwm_right_count,
        g_chassis.status.heading_fusion_active);
}

void chassis_set_imu_yaw(float yaw_deg)
{
    if (g_chassis_initialized && (yaw_deg == yaw_deg)) {
        g_chassis.status.imu_yaw_deg = yaw_deg;
    }
}

void chassis_set_imu_sample(float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid)
{
    if (!g_chassis_initialized) {
        return;
    }
    chassis_motion_set_imu_sample(&g_chassis, yaw_deg,
        body_gyro_z_dps, timestamp_ms, valid);
}

void chassis_poll(void)
{
    ml_encoder_diagnostics_t diagnostics;
    motor_velocity_measurement_t measurement;
    int32_t left_delta;
    int32_t right_delta;
    float period_scale;
    float target_left_ticks;
    float target_right_ticks;
    uint8_t cycles;
    ml_status_t status;

    if (!g_chassis_initialized) {
        return;
    }
    cycles = chassis_take_pending_cycles();
    if (cycles == 0U) {
        return;
    }
    status = ml_encoder_read_and_clear(&left_delta, &right_delta);
    if (status != ML_STATUS_OK) {
        chassis_motion_stop(&g_chassis, false);
        g_chassis.status.result = CHASSIS_RESULT_FAULT;
        g_chassis.status.fault = CHASSIS_FAULT_ENCODER;
        (void) ml_motor_driver_stop_all();
        return;
    }
    left_delta *= g_chassis.config.left.encoder_sign;
    right_delta *= g_chassis.config.right.encoder_sign;
    (void) chassis_motion_update(&g_chassis,
        left_delta, right_delta, cycles);

    period_scale = (float) g_chassis.config.control_period_ms / 1000.0f;
    target_left_ticks = g_chassis.status.target_left_mm_s * period_scale /
        g_chassis.config.left.mm_per_tick;
    target_right_ticks = g_chassis.status.target_right_mm_s * period_scale /
        g_chassis.config.right.mm_per_tick;
    if (g_chassis.status.result == CHASSIS_RESULT_RUNNING) {
        g_idle_capture_active = false;
        status = motor_velocity_update(target_left_ticks,
            target_right_ticks, (float) left_delta / (float) cycles,
            (float) right_delta / (float) cycles,
            g_chassis.config.left.motor_sign,
            g_chassis.config.right.motor_sign);
        if (status != ML_STATUS_OK) {
            g_chassis.status.mode = CHASSIS_MODE_IDLE;
            g_chassis.status.result = CHASSIS_RESULT_FAULT;
            g_chassis.status.fault = CHASSIS_FAULT_MOTOR_DRIVER;
            (void) ml_motor_driver_stop_all();
        }
    } else {
        (void) motor_velocity_reset();
        (void) ml_motor_driver_stop_all();
    }

    if ((g_chassis.status.result == CHASSIS_RESULT_RUNNING) &&
        (motor_velocity_get_measurement(&measurement) == ML_STATUS_OK)) {
        g_chassis.status.measured_left_mm_s =
            measurement.filtered_a_ticks *
            g_chassis.config.left.mm_per_tick / period_scale;
        g_chassis.status.measured_right_mm_s =
            measurement.filtered_b_ticks *
            g_chassis.config.right.mm_per_tick / period_scale;
        g_chassis.status.pwm_left_count = measurement.duty_a_count;
        g_chassis.status.pwm_right_count = measurement.duty_b_count;
    } else {
        /* Keep passive wheel-speed observation alive while braking. */
        g_chassis.status.measured_left_mm_s =
            ((float) left_delta / (float) cycles) *
            g_chassis.config.left.mm_per_tick / period_scale;
        g_chassis.status.measured_right_mm_s =
            ((float) right_delta / (float) cycles) *
            g_chassis.config.right.mm_per_tick / period_scale;
        g_chassis.status.pwm_left_count = 0U;
        g_chassis.status.pwm_right_count = 0U;
    }
    if (ml_encoder_get_diagnostics(&diagnostics) == ML_STATUS_OK) {
        g_chassis.status.encoder_total_left =
            diagnostics.total_count_a * g_chassis.config.left.encoder_sign;
        g_chassis.status.encoder_total_right =
            diagnostics.total_count_b * g_chassis.config.right.encoder_sign;
        g_chassis.status.encoder_invalid_left =
            diagnostics.invalid_transitions_a;
        g_chassis.status.encoder_invalid_right =
            diagnostics.invalid_transitions_b;
    }
    if ((g_chassis.status.result == CHASSIS_RESULT_RUNNING) ||
        g_idle_capture_active) {
        if (g_chassis.status.timestamp_ms >= g_next_telemetry_ms) {
            (void) chassis_capture_telemetry_now();
            g_next_telemetry_ms =
                g_chassis.status.timestamp_ms + 100U;
        }
    } else {
        g_next_telemetry_ms = g_chassis.status.timestamp_ms + 100U;
    }
}
