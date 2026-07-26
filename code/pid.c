#include "pid.h"

#include <float.h>

#include "ml_board.h"
#include "ml_motor.h"

pid_t motorA;
pid_t motorB;

static float pid_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static bool pid_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

ml_status_t pid_init(
    pid_t *pid, uint32_t mode, float p, float i, float d)
{
    if ((pid == 0) ||
        ((mode != (uint32_t) POSITION_PID) &&
         (mode != (uint32_t) DELTA_PID)) ||
        !pid_is_finite(p) || !pid_is_finite(i) || !pid_is_finite(d)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    pid->target = 0.0f;
    pid->now = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
    pid->pid_mode = mode;
    pid->output_min = -(float) ML_PWM_DUTY_MAX;
    pid->output_max = (float) ML_PWM_DUTY_MAX;
    pid->integral_min = -(float) ML_PWM_DUTY_MAX;
    pid->integral_max = (float) ML_PWM_DUTY_MAX;
    pid->initialized = true;
    return ML_STATUS_OK;
}

ml_status_t pid_reset(pid_t *pid)
{
    if (pid == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!pid->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    pid->target = 0.0f;
    pid->now = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
    return ML_STATUS_OK;
}

ml_status_t pid_set_limits(pid_t *pid, float output_min, float output_max,
    float integral_min, float integral_max)
{
    if ((pid == 0) || !pid_is_finite(output_min) ||
        !pid_is_finite(output_max) || !pid_is_finite(integral_min) ||
        !pid_is_finite(integral_max) || (output_min >= output_max) ||
        (integral_min > integral_max)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!pid->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->iout = pid_clamp(pid->iout, integral_min, integral_max);
    pid->out = pid_clamp(pid->out, output_min, output_max);
    return ML_STATUS_OK;
}

ml_status_t pid_cal(pid_t *pid)
{
    float proposed_integral;
    float proposed_output;

    if (pid == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!pid->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((pid->pid_mode != (uint32_t) POSITION_PID) &&
        (pid->pid_mode != (uint32_t) DELTA_PID)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!pid_is_finite(pid->target) || !pid_is_finite(pid->now)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    pid->error[0] = pid->target - pid->now;
    if (pid->pid_mode == (uint32_t) DELTA_PID) {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid_clamp(pid->i * pid->error[0],
            pid->integral_min, pid->integral_max);
        pid->dout = pid->d *
            (pid->error[0] - (2.0f * pid->error[1]) + pid->error[2]);
        proposed_output = pid->out + pid->pout + pid->iout + pid->dout;
        pid->out = pid_clamp(
            proposed_output, pid->output_min, pid->output_max);
    } else {
        pid->pout = pid->p * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        proposed_integral = pid_clamp(
            pid->iout + (pid->i * pid->error[0]),
            pid->integral_min, pid->integral_max);
        proposed_output = pid->pout + proposed_integral + pid->dout;

        if (!((proposed_output > pid->output_max) &&
              (pid->error[0] > 0.0f)) &&
            !((proposed_output < pid->output_min) &&
              (pid->error[0] < 0.0f))) {
            pid->iout = proposed_integral;
        }
        pid->out = pid_clamp(pid->pout + pid->iout + pid->dout,
            pid->output_min, pid->output_max);
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    return ML_STATUS_OK;
}

ml_status_t pidout_limit(pid_t *pid)
{
    if (pid == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!pid->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    pid->out = pid_clamp(pid->out, pid->output_min, pid->output_max);
    return ML_STATUS_OK;
}

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
    status = encoder_get_and_clear(&count1, &count2);

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
        status = motorA_duty((int32_t) pid_clamp(motorA.out,
            -(float) ML_PWM_DUTY_MAX, (float) ML_PWM_DUTY_MAX));
    }
    if (status == ML_STATUS_OK) {
        status = motorB_duty((int32_t) pid_clamp(motorB.out,
            -(float) ML_PWM_DUTY_MAX, (float) ML_PWM_DUTY_MAX));
    }
    return status;
}
