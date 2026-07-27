#include "motor_velocity.h"

#include <float.h>

#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "pid.h"

pid_t motorA;
pid_t motorB;

static motor_velocity_config_t g_velocity_config;
static bool g_velocity_initialized;

static bool velocity_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) &&
        (value >= -FLT_MAX);
}

static float velocity_clamp(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t velocity_output(
    pid_t *controller, float target_ticks, float actual_ticks,
    int8_t motor_sign)
{
    float output;

    if ((target_ticks > -0.01f) && (target_ticks < 0.01f)) {
        (void) pid_reset(controller);
        return 0;
    }
    controller->target = target_ticks;
    controller->now = actual_ticks;
    (void) pid_cal(controller);
    output = controller->out + ((target_ticks > 0.0f) ?
        g_velocity_config.feedforward : -g_velocity_config.feedforward);
    output = velocity_clamp(output,
        -(float) ML_MOTOR_DUTY_LIMIT, (float) ML_MOTOR_DUTY_LIMIT);
    return (int32_t) (output * (float) motor_sign);
}

ml_status_t motor_velocity_init(
    const motor_velocity_config_t *config)
{
    ml_status_t status;

    if ((config == 0) || !velocity_is_finite(config->kp) ||
        !velocity_is_finite(config->ki) ||
        !velocity_is_finite(config->kd) ||
        !velocity_is_finite(config->feedforward) ||
        !velocity_is_finite(config->output_limit) ||
        !velocity_is_finite(config->integral_limit) ||
        (config->feedforward < 0.0f) ||
        (config->output_limit <= 0.0f) ||
        (config->output_limit > (float) ML_MOTOR_DUTY_LIMIT) ||
        (config->integral_limit <= 0.0f)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    g_velocity_config = *config;
    g_velocity_initialized = false;
    status = pid_init(&motorA, POSITION_PID,
        config->kp, config->ki, config->kd);
    if (status == ML_STATUS_OK) {
        status = pid_init(&motorB, POSITION_PID,
            config->kp, config->ki, config->kd);
    }
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(&motorA, -config->output_limit,
            config->output_limit, -config->integral_limit,
            config->integral_limit);
    }
    if (status == ML_STATUS_OK) {
        status = pid_set_limits(&motorB, -config->output_limit,
            config->output_limit, -config->integral_limit,
            config->integral_limit);
    }
    if (status == ML_STATUS_OK) {
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
    return status;
}

ml_status_t motor_velocity_update(
    float target_a_ticks,
    float target_b_ticks,
    int32_t actual_a_ticks,
    int32_t actual_b_ticks,
    int8_t motor_a_sign,
    int8_t motor_b_sign)
{
    ml_status_t status;
    int32_t duty_a;
    int32_t duty_b;

    if (!g_velocity_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!velocity_is_finite(target_a_ticks) ||
        !velocity_is_finite(target_b_ticks) ||
        ((motor_a_sign != 1) && (motor_a_sign != -1)) ||
        ((motor_b_sign != 1) && (motor_b_sign != -1))) {
        (void) ml_motor_driver_stop_all();
        return ML_STATUS_INVALID_ARGUMENT;
    }
    duty_a = velocity_output(&motorA, target_a_ticks,
        (float) actual_a_ticks, motor_a_sign);
    duty_b = velocity_output(&motorB, target_b_ticks,
        (float) actual_b_ticks, motor_b_sign);
    status = ml_motor_driver_set_duty(ML_MOTOR_A, duty_a);
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_set_duty(ML_MOTOR_B, duty_b);
    }
    if (status != ML_STATUS_OK) {
        (void) ml_motor_driver_stop_all();
    }
    return status;
}

/* Legacy PID entry points retained for older application code. */
void motor_target_set(int spe1, int spe2)
{
    motorA.target = (float) spe1;
    motorB.target = (float) spe2;
}

ml_status_t pid_control(void)
{
    int32_t count1;
    int32_t count2;
    ml_status_t status;

    if (!motorA.initialized || !motorB.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    status = ml_encoder_read_and_clear(&count1, &count2);
    if (status != ML_STATUS_OK) {
        return status;
    }
    motorA.now = (float) count1;
    motorB.now = (float) count2;
    status = pid_cal(&motorA);
    if (status == ML_STATUS_OK) {
        status = pid_cal(&motorB);
    }
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_set_duty(ML_MOTOR_A,
            (int32_t) velocity_clamp(motorA.out,
                -(float) ML_MOTOR_DUTY_LIMIT,
                (float) ML_MOTOR_DUTY_LIMIT));
    }
    if (status == ML_STATUS_OK) {
        status = ml_motor_driver_set_duty(ML_MOTOR_B,
            (int32_t) velocity_clamp(motorB.out,
                -(float) ML_MOTOR_DUTY_LIMIT,
                (float) ML_MOTOR_DUTY_LIMIT));
    }
    if (status != ML_STATUS_OK) {
        (void) ml_motor_driver_stop_all();
    }
    return status;
}
