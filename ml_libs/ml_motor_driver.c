#include "ml_motor_driver.h"

#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_pwm.h"

uint8_t motorA_dir = 1U;
uint8_t motorB_dir = 1U;

static bool g_motor_initialized;

typedef struct {
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIOn_enum in1_iomux;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    GPIOn_enum in2_iomux;
    GPTIMER_Regs *timer;
    DL_TIMER_CC_INDEX channel;
    uint8_t *direction;
} motor_driver_channel_t;

static const motor_driver_channel_t g_channels[] = {
    {
        ML_MOTOR_A_IN1_PORT, ML_MOTOR_A_IN1_PIN,
        (GPIOn_enum) ML_MOTOR_A_IN1_IOMUX,
        ML_MOTOR_A_IN2_PORT, ML_MOTOR_A_IN2_PIN,
        (GPIOn_enum) ML_MOTOR_A_IN2_IOMUX,
        ML_MOTOR_A_PWM_TIMER, ML_MOTOR_A_PWM_CHANNEL,
        &motorA_dir
    },
    {
        ML_MOTOR_B_IN1_PORT, ML_MOTOR_B_IN1_PIN,
        (GPIOn_enum) ML_MOTOR_B_IN1_IOMUX,
        ML_MOTOR_B_IN2_PORT, ML_MOTOR_B_IN2_PIN,
        (GPIOn_enum) ML_MOTOR_B_IN2_IOMUX,
        ML_MOTOR_B_PWM_TIMER, ML_MOTOR_B_PWM_CHANNEL,
        &motorB_dir
    }
};

static bool motor_id_valid(ml_motor_id_t motor)
{
    return (motor == ML_MOTOR_A) || (motor == ML_MOTOR_B);
}

static ml_status_t motor_apply(
    const motor_driver_channel_t *channel, int32_t duty)
{
    uint32_t magnitude;
    bool forward;
    ml_status_t status;

    if (!g_motor_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (channel == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    forward = duty >= 0;
    magnitude = forward ? (uint32_t) duty :
        ((uint32_t) (-(duty + 1)) + 1U);
    if (magnitude > ML_MOTOR_DUTY_LIMIT) {
        magnitude = ML_MOTOR_DUTY_LIMIT;
    }

    status = pwm_update(channel->timer, channel->channel, 0U);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (magnitude == 0U) {
        gpio_set(channel->in1_port, channel->in1_pin, 0U);
        gpio_set(channel->in2_port, channel->in2_pin, 0U);
        return ML_STATUS_OK;
    }

    *(channel->direction) = forward ? 1U : 0U;
    gpio_set(channel->in1_port, channel->in1_pin,
        *(channel->direction));
    gpio_set(channel->in2_port, channel->in2_pin,
        (uint8_t) !(*(channel->direction)));
    return pwm_update(channel->timer, channel->channel, magnitude);
}

ml_status_t ml_motor_driver_init(void)
{
    ml_status_t status;

    if (g_motor_initialized) {
        return ML_STATUS_OK;
    }
    status = board_resource_claim(
        ML_BOARD_RESOURCE_PA13, ML_BOARD_OWNER_MOTOR);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = board_resource_claim(
        ML_BOARD_RESOURCE_PB7, ML_BOARD_OWNER_MOTOR);
    if (status != ML_STATUS_OK) {
        board_resource_release(
            ML_BOARD_RESOURCE_PA13, ML_BOARD_OWNER_MOTOR);
        return status;
    }

    status = gpio_init(g_channels[ML_MOTOR_A].in1_port,
        g_channels[ML_MOTOR_A].in1_pin,
        g_channels[ML_MOTOR_A].in1_iomux, OUT);
    if (status == ML_STATUS_OK) {
        status = gpio_init(g_channels[ML_MOTOR_A].in2_port,
            g_channels[ML_MOTOR_A].in2_pin,
            g_channels[ML_MOTOR_A].in2_iomux, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(g_channels[ML_MOTOR_B].in1_port,
            g_channels[ML_MOTOR_B].in1_pin,
            g_channels[ML_MOTOR_B].in1_iomux, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(g_channels[ML_MOTOR_B].in2_port,
            g_channels[ML_MOTOR_B].in2_pin,
            g_channels[ML_MOTOR_B].in2_iomux, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = pwm_init(g_channels[ML_MOTOR_A].timer,
            g_channels[ML_MOTOR_A].channel, ML_MOTOR_PWM_FREQUENCY_HZ);
    }
    if (status == ML_STATUS_OK) {
        status = pwm_init(g_channels[ML_MOTOR_B].timer,
            g_channels[ML_MOTOR_B].channel, ML_MOTOR_PWM_FREQUENCY_HZ);
    }
    if (status != ML_STATUS_OK) {
        board_resource_release(
            ML_BOARD_RESOURCE_PB7, ML_BOARD_OWNER_MOTOR);
        board_resource_release(
            ML_BOARD_RESOURCE_PA13, ML_BOARD_OWNER_MOTOR);
        return status;
    }

    motorA_dir = 1U;
    motorB_dir = 1U;
    g_motor_initialized = true;
    (void) ml_motor_driver_stop_all();
    return ML_STATUS_OK;
}

ml_status_t ml_motor_driver_set_duty(
    ml_motor_id_t motor, int32_t duty)
{
    if (!motor_id_valid(motor)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    return motor_apply(&g_channels[motor], duty);
}

ml_status_t ml_motor_driver_stop(ml_motor_id_t motor)
{
    return ml_motor_driver_set_duty(motor, 0);
}

ml_status_t ml_motor_driver_stop_all(void)
{
    ml_status_t status_a;
    ml_status_t status_b;

    status_a = ml_motor_driver_stop(ML_MOTOR_A);
    status_b = ml_motor_driver_stop(ML_MOTOR_B);
    return (status_a != ML_STATUS_OK) ? status_a : status_b;
}
