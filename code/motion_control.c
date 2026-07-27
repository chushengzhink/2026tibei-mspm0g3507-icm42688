#include "motion_control.h"

#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "ml_tim.h"
#include "motion_engine.h"
#include "motor_velocity.h"

static motion_engine_t g_motion_engine;
static uint8_t g_speed_divider;

static void motion_zero_motors(void)
{
    (void) motor_velocity_reset();
    (void) ml_motor_driver_stop_all();
}

static void motion_apply_speeds(const motion_engine_output_t *output)
{
    const robot_calibration_t *calibration =
        g_motion_engine.calibration;
    const float period_s = ((float) ROBOT_CONTROL_TICK_MS *
        (float) ROBOT_SPEED_TICK_DIVIDER) / 1000.0f;
    float left_target_counts = output->left_speed_mm_s * period_s /
        calibration->left_mm_per_tick;
    float right_target_counts = output->right_speed_mm_s * period_s /
        calibration->right_mm_per_tick;

    (void) motor_velocity_update(left_target_counts, right_target_counts,
        output->left_delta_ticks, output->right_delta_ticks,
        calibration->left_motor_sign, calibration->right_motor_sign);
}

static void motion_service_stop_request(void)
{
    if (motion_engine_take_stop_request(&g_motion_engine)) {
        motion_zero_motors();
    }
}

static void motion_timer_callback(void *context)
{
    motion_engine_output_t output;
    int32_t count_a;
    int32_t count_b;
    ml_status_t status;

    (void) context;
    motion_engine_line_tick(&g_motion_engine, line_sensor_read());
    motion_service_stop_request();
    if (++g_speed_divider < ROBOT_SPEED_TICK_DIVIDER) {
        return;
    }
    g_speed_divider = 0U;

    status = ml_encoder_read_and_clear(&count_a, &count_b);
    if (status != ML_STATUS_OK) {
        motion_engine_fail(&g_motion_engine, MOTION_FAULT_ENCODER);
        motion_service_stop_request();
        return;
    }
    motion_engine_velocity_tick(
        &g_motion_engine, count_a, count_b, &output);
    motion_service_stop_request();
    if (output.apply_speeds) {
        motion_apply_speeds(&output);
    }
}

ml_status_t motion_init(const robot_calibration_t *calibration)
{
    motor_velocity_config_t velocity_config;
    ml_status_t status;

    status = motion_engine_init(
        &g_motion_engine, calibration, line_sensor_read());
    if (status != ML_STATUS_OK) {
        return status;
    }
    g_speed_divider = 0U;
    velocity_config.kp = ROBOT_VELOCITY_KP;
    velocity_config.ki = ROBOT_VELOCITY_KI;
    velocity_config.kd = ROBOT_VELOCITY_KD;
    velocity_config.feedforward = ROBOT_MOTOR_FEEDFORWARD;
    velocity_config.output_limit = ROBOT_PID_OUTPUT_LIMIT;
    velocity_config.integral_limit = ROBOT_PID_INTEGRAL_LIMIT;
    status = motor_velocity_init(&velocity_config);
    if (status == ML_STATUS_OK) {
        status = tim_interrupt_ms_init_ex(TIMG0, ROBOT_CONTROL_TICK_MS,
            0U, motion_timer_callback, 0);
    }
    return status;
}

static ml_status_t motion_finish_start(ml_status_t status)
{
    if (status == ML_STATUS_OK) {
        (void) motor_velocity_reset();
    }
    return status;
}

ml_status_t motion_start_find_a(float speed_mm_s)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_find_a(&g_motion_engine, speed_mm_s);
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

ml_status_t motion_start_straight(float distance_mm, float speed_mm_s)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_straight(
        &g_motion_engine, distance_mm, speed_mm_s);
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

ml_status_t motion_start_line(float distance_mm, float speed_mm_s)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_line(&g_motion_engine,
        distance_mm, speed_mm_s, line_sensor_read());
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

ml_status_t motion_start_line_to_end(float expected_distance_mm,
    float speed_mm_s)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_line_to_end(&g_motion_engine,
        expected_distance_mm, speed_mm_s, line_sensor_read());
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

ml_status_t motion_start_turn90(bool turn_right, bool align_to_line)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_turn90(
        &g_motion_engine, turn_right, align_to_line);
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

ml_status_t motion_start_circle(uint16_t radius_mm)
{
    uint32_t interrupt_state = __get_PRIMASK();
    ml_status_t status;

    __disable_irq();
    status = motion_engine_start_circle(&g_motion_engine, radius_mm);
    status = motion_finish_start(status);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}

void motion_stop(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    motion_engine_stop(&g_motion_engine);
    motion_service_stop_request();
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

void motion_emergency_stop(void)
{
    motion_stop();
}

motion_status_t motion_get_status(void)
{
    motion_status_t result;
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    result = motion_engine_get_status(&g_motion_engine);
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return result;
}

uint32_t motion_get_uptime_ticks(void)
{
    return motion_engine_get_uptime_ticks(&g_motion_engine);
}
