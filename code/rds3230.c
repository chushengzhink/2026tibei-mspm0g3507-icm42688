#include "rds3230.h"

#include "ml_pwm.h"

static ml_status_t rds3230_write_pulse(
    const rds3230_t *servo, uint16_t pulse_us)
{
    uint32_t duty;

    duty = (uint32_t) ((((uint64_t) pulse_us * servo->frequency_hz *
        ML_PWM_DUTY_MAX) + 500000ULL) / 1000000ULL);
    return pwm_update(servo->timer, servo->channel, duty);
}

ml_status_t rds3230_init(rds3230_t *servo, GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX channel, uint16_t frequency_hz,
    uint16_t minimum_us, uint16_t center_us, uint16_t maximum_us,
    uint32_t maximum_slew_us_per_s)
{
    uint32_t period_us;
    ml_status_t status;

    if ((servo == 0) || (timer == 0) || (frequency_hz == 0U) ||
        (minimum_us >= center_us) || (center_us >= maximum_us) ||
        (maximum_slew_us_per_s == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    period_us = 1000000UL / frequency_hz;
    if (maximum_us >= period_us) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    status = pwm_init(timer, channel, frequency_hz);
    if (status != ML_STATUS_OK) {
        return status;
    }

    servo->timer = timer;
    servo->channel = channel;
    servo->frequency_hz = frequency_hz;
    servo->minimum_us = minimum_us;
    servo->center_us = center_us;
    servo->maximum_us = maximum_us;
    servo->current_us = center_us;
    servo->target_us = center_us;
    servo->maximum_slew_us_per_s = maximum_slew_us_per_s;
    servo->last_update_ms = 0U;
    servo->time_initialized = false;
    servo->initialized = true;

    status = rds3230_write_pulse(servo, center_us);
    if (status != ML_STATUS_OK) {
        servo->initialized = false;
    }
    return status;
}

ml_status_t rds3230_set_target_us(rds3230_t *servo, uint16_t pulse_us)
{
    if (servo == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!servo->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if ((pulse_us < servo->minimum_us) ||
        (pulse_us > servo->maximum_us)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    servo->target_us = pulse_us;
    return ML_STATUS_OK;
}

ml_status_t rds3230_set_center(rds3230_t *servo)
{
    if (servo == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    return rds3230_set_target_us(servo, servo->center_us);
}

ml_status_t rds3230_update(rds3230_t *servo, uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t maximum_step;
    uint32_t difference;

    if (servo == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!servo->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (!servo->time_initialized) {
        servo->last_update_ms = now_ms;
        servo->time_initialized = true;
        return rds3230_write_pulse(servo, servo->current_us);
    }

    elapsed_ms = now_ms - servo->last_update_ms;
    if (elapsed_ms == 0U) {
        return ML_STATUS_OK;
    }
    servo->last_update_ms = now_ms;
    if (elapsed_ms > 100U) {
        elapsed_ms = 100U;
    }

    maximum_step = (uint32_t) (((uint64_t)
        servo->maximum_slew_us_per_s * elapsed_ms) / 1000U);
    if (maximum_step == 0U) {
        maximum_step = 1U;
    }

    if (servo->current_us < servo->target_us) {
        difference = (uint32_t) servo->target_us - servo->current_us;
        if (maximum_step > difference) {
            maximum_step = difference;
        }
        servo->current_us = (uint16_t) (servo->current_us + maximum_step);
    } else if (servo->current_us > servo->target_us) {
        difference = (uint32_t) servo->current_us - servo->target_us;
        if (maximum_step > difference) {
            maximum_step = difference;
        }
        servo->current_us = (uint16_t) (servo->current_us - maximum_step);
    }

    return rds3230_write_pulse(servo, servo->current_us);
}

uint16_t rds3230_get_current_us(const rds3230_t *servo)
{
    return ((servo == 0) || !servo->initialized) ? 0U : servo->current_us;
}

uint16_t rds3230_get_target_us(const rds3230_t *servo)
{
    return ((servo == 0) || !servo->initialized) ? 0U : servo->target_us;
}
