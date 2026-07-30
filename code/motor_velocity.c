#include "motor_velocity.h"

#include <float.h>

#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "pid.h"

pid_t motorA;
pid_t motorB;

static motor_velocity_config_t g_velocity_config;
static motor_velocity_measurement_t g_measurement;
static int8_t g_previous_direction_a;
static int8_t g_previous_direction_b;
static bool g_velocity_initialized;

static bool velocity_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float velocity_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int8_t velocity_direction(float target)
{
    if (target > 0.01f) {
        return 1;
    }
    if (target < -0.01f) {
        return -1;
    }
    return 0;
}

static uint16_t velocity_duty_magnitude(int32_t duty)
{
    uint32_t magnitude = duty >= 0 ? (uint32_t) duty :
        ((uint32_t) (-(duty + 1)) + 1U);

    if (magnitude > ML_MOTOR_DUTY_LIMIT) {
        magnitude = ML_MOTOR_DUTY_LIMIT;
    }
    return (uint16_t) magnitude;
}

static bool velocity_wheel_config_valid(
    const motor_velocity_wheel_config_t *config)
{
    return (config != 0) && velocity_is_finite(config->kp) &&
        velocity_is_finite(config->ki) &&
        velocity_is_finite(config->kd) &&
        velocity_is_finite(config->feedforward) &&
        velocity_is_finite(config->output_limit) &&
        velocity_is_finite(config->integral_limit) &&
        (config->feedforward >= 0.0f) &&
        (config->output_limit > 0.0f) &&
        (config->output_limit <= (float) ML_MOTOR_DUTY_LIMIT) &&
        (config->integral_limit > 0.0f);
}

static ml_status_t velocity_init_pid(pid_t *controller,
    const motor_velocity_wheel_config_t *config)
{
    ml_status_t status;

    status = pid_init(controller, POSITION_PID,
        config->kp, config->ki, config->kd);
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(controller,
            -config->output_limit, config->output_limit,
            -config->integral_limit, config->integral_limit);
    }
    return status;
}

static int32_t velocity_output(pid_t *controller,
    const motor_velocity_wheel_config_t *config,
    float target_ticks, float actual_ticks, int8_t motor_sign,
    int8_t *previous_direction)
{
    float output;
    int8_t direction = velocity_direction(target_ticks);

    if ((direction == 0) ||
        ((*previous_direction != 0) &&
         (direction != *previous_direction))) {
        (void) pid_reset(controller);
    }
    *previous_direction = direction;
    if (direction == 0) {
        return 0;
    }

    controller->target = target_ticks;
    controller->now = actual_ticks;
    (void) pid_cal(controller);
    output = controller->out + ((direction > 0) ?
        config->feedforward : -config->feedforward);
    output = velocity_clamp(output,
        -(float) ML_MOTOR_DUTY_LIMIT, (float) ML_MOTOR_DUTY_LIMIT);
    return (int32_t) (output * (float) motor_sign);
}

ml_status_t motor_velocity_init(const motor_velocity_config_t *config)
{
    ml_status_t status;

    if ((config == 0) ||
        !velocity_wheel_config_valid(&config->motor_a) ||
        !velocity_wheel_config_valid(&config->motor_b) ||
        !velocity_is_finite(config->filter_alpha) ||
        (config->filter_alpha <= 0.0f) ||
        (config->filter_alpha > 1.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    g_velocity_initialized = false;
    status = velocity_init_pid(&motorA, &config->motor_a);
    if (status == ML_STATUS_OK) {
        status = velocity_init_pid(&motorB, &config->motor_b);
    }
    if (status == ML_STATUS_OK) {
        g_velocity_config = *config;
        g_measurement.filtered_a_ticks = 0.0f;
        g_measurement.filtered_b_ticks = 0.0f;
        g_measurement.duty_a_count = 0U;
        g_measurement.duty_b_count = 0U;
        g_previous_direction_a = 0;
        g_previous_direction_b = 0;
        g_velocity_initialized = true;
    }
    return status;
}

ml_status_t motor_velocity_reset(void)
{
    ml_status_t status;

    if (!g_velocity_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    status = pid_reset(&motorA);
    if (status == ML_STATUS_OK) {
        status = pid_reset(&motorB);
    }
    g_measurement.filtered_a_ticks = 0.0f;
    g_measurement.filtered_b_ticks = 0.0f;
    g_measurement.duty_a_count = 0U;
    g_measurement.duty_b_count = 0U;
    g_previous_direction_a = 0;
    g_previous_direction_b = 0;
    return status;
}

ml_status_t motor_velocity_update(float target_a_ticks,
    float target_b_ticks, float actual_a_ticks, float actual_b_ticks,
    int8_t motor_a_sign, int8_t motor_b_sign)
{
    ml_status_t status;
    int32_t duty_a;
    int32_t duty_b;

    if (!g_velocity_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!velocity_is_finite(target_a_ticks) ||
        !velocity_is_finite(target_b_ticks) ||
        !velocity_is_finite(actual_a_ticks) ||
        !velocity_is_finite(actual_b_ticks) ||
        ((motor_a_sign != 1) && (motor_a_sign != -1)) ||
        ((motor_b_sign != 1) && (motor_b_sign != -1))) {
        (void) ml_motor_driver_stop_all();
        g_measurement.duty_a_count = 0U;
        g_measurement.duty_b_count = 0U;
        return ML_STATUS_INVALID_ARGUMENT;
    }

    g_measurement.filtered_a_ticks += g_velocity_config.filter_alpha *
        (actual_a_ticks - g_measurement.filtered_a_ticks);
    g_measurement.filtered_b_ticks += g_velocity_config.filter_alpha *
        (actual_b_ticks - g_measurement.filtered_b_ticks);
    duty_a = velocity_output(&motorA, &g_velocity_config.motor_a,
        target_a_ticks, g_measurement.filtered_a_ticks,
        motor_a_sign, &g_previous_direction_a);
    duty_b = velocity_output(&motorB, &g_velocity_config.motor_b,
        target_b_ticks, g_measurement.filtered_b_ticks,
        motor_b_sign, &g_previous_direction_b);
    g_measurement.duty_a_count = velocity_duty_magnitude(duty_a);
    g_measurement.duty_b_count = velocity_duty_magnitude(duty_b);
    status = ml_motor_driver_set_duty(ML_MOTOR_A, duty_a);
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_set_duty(ML_MOTOR_B, duty_b);
    }
    if (status != ML_STATUS_OK) {
        (void) ml_motor_driver_stop_all();
        g_measurement.duty_a_count = 0U;
        g_measurement.duty_b_count = 0U;
    }
    return status;
}

ml_status_t motor_velocity_get_measurement(
    motor_velocity_measurement_t *measurement)
{
    if (measurement == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_velocity_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    *measurement = g_measurement;
    return ML_STATUS_OK;
}

void motor_target_set(int spe1, int spe2)
{
    motorA.target = (float) spe1;
    motorB.target = (float) spe2;
}

ml_status_t pid_control(void)
{
    int32_t count_a;
    int32_t count_b;
    ml_status_t status;

    status = ml_encoder_read_and_clear(&count_a, &count_b);
    if (status != ML_STATUS_OK) {
        return status;
    }
    return motor_velocity_update(motorA.target, motorB.target,
        (float) count_a, (float) count_b, 1, 1);
}
