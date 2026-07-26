#include "ml_motor.h"

#include "ml_board.h"
#include "ml_exti.h"
#include "ml_gpio.h"
#include "ml_pwm.h"

typedef struct {
    GPIO_Regs *phase_b_port;
    uint32_t phase_b_pin;
    volatile int32_t *count;
    int8_t polarity;
} encoder_context_t;

uint8_t motorA_dir = 1U;
uint8_t motorB_dir = 1U;
volatile int32_t Encoder_count1 = 0;
volatile int32_t Encoder_count2 = 0;

static bool g_motor_initialized;
static bool g_encoder_initialized;

static encoder_context_t g_encoder_a_context = {
    ML_ENCODER_A_PHASE_B_PORT,
    ML_ENCODER_A_PHASE_B_PIN,
    &Encoder_count1,
    1
};

static encoder_context_t g_encoder_b_context = {
    ML_ENCODER_B_PHASE_B_PORT,
    ML_ENCODER_B_PHASE_B_PIN,
    &Encoder_count2,
    -1
};

static void encoder_edge_callback(uint32_t pin, void *context)
{
    encoder_context_t *encoder = (encoder_context_t *) context;
    int32_t delta;

    (void) pin;
    if (encoder == 0) {
        return;
    }
    delta = gpio_get(encoder->phase_b_port, encoder->phase_b_pin) ?
        encoder->polarity : -encoder->polarity;
    if ((delta > 0) && (*encoder->count < INT32_MAX)) {
        ++(*encoder->count);
    } else if ((delta < 0) && (*encoder->count > INT32_MIN)) {
        --(*encoder->count);
    }
}

static ml_status_t motor_apply(GPIO_Regs *in1_port, uint32_t in1_pin,
    GPIO_Regs *in2_port, uint32_t in2_pin, GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX channel, int32_t duty, uint8_t *direction)
{
    uint32_t magnitude;
    bool forward;
    ml_status_t status;

    if (!g_motor_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    forward = duty >= 0;
    magnitude = forward ? (uint32_t) duty :
        ((uint32_t) (-(duty + 1)) + 1U);
    if (magnitude > ML_PWM_DUTY_MAX) {
        magnitude = ML_PWM_DUTY_MAX;
    }

    status = pwm_update(timer, channel, 0U);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (magnitude == 0U) {
        gpio_set(in1_port, in1_pin, 0U);
        gpio_set(in2_port, in2_pin, 0U);
        return ML_STATUS_OK;
    }

    *direction = forward ? 1U : 0U;
    gpio_set(in1_port, in1_pin, *direction);
    gpio_set(in2_port, in2_pin, (uint8_t) !(*direction));
    return pwm_update(timer, channel, magnitude);
}

ml_status_t motor_init(void)
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

    status = gpio_init(ML_MOTOR_A_IN1_PORT, ML_MOTOR_A_IN1_PIN,
        (GPIOn_enum) ML_MOTOR_A_IN1_IOMUX, OUT);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_MOTOR_A_IN2_PORT, ML_MOTOR_A_IN2_PIN,
            (GPIOn_enum) ML_MOTOR_A_IN2_IOMUX, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_MOTOR_B_IN1_PORT, ML_MOTOR_B_IN1_PIN,
            (GPIOn_enum) ML_MOTOR_B_IN1_IOMUX, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_MOTOR_B_IN2_PORT, ML_MOTOR_B_IN2_PIN,
            (GPIOn_enum) ML_MOTOR_B_IN2_IOMUX, OUT);
    }
    if (status == ML_STATUS_OK) {
        status = pwm_init(ML_MOTOR_A_PWM_TIMER,
            ML_MOTOR_A_PWM_CHANNEL, ML_MOTOR_PWM_FREQUENCY_HZ);
    }
    if (status == ML_STATUS_OK) {
        status = pwm_init(ML_MOTOR_B_PWM_TIMER,
            ML_MOTOR_B_PWM_CHANNEL, ML_MOTOR_PWM_FREQUENCY_HZ);
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
    (void) motorA_duty(0);
    (void) motorB_duty(0);
    return ML_STATUS_OK;
}

ml_status_t motorA_duty(int32_t duty)
{
    return motor_apply(ML_MOTOR_A_IN1_PORT, ML_MOTOR_A_IN1_PIN,
        ML_MOTOR_A_IN2_PORT, ML_MOTOR_A_IN2_PIN,
        ML_MOTOR_A_PWM_TIMER, ML_MOTOR_A_PWM_CHANNEL, duty, &motorA_dir);
}

ml_status_t motorB_duty(int32_t duty)
{
    return motor_apply(ML_MOTOR_B_IN1_PORT, ML_MOTOR_B_IN1_PIN,
        ML_MOTOR_B_IN2_PORT, ML_MOTOR_B_IN2_PIN,
        ML_MOTOR_B_PWM_TIMER, ML_MOTOR_B_PWM_CHANNEL, duty, &motorB_dir);
}

ml_status_t encoder_init(void)
{
    ml_status_t status;

    if (g_encoder_initialized) {
        return ML_STATUS_OK;
    }
    status = gpio_init(ML_ENCODER_A_PHASE_B_PORT, ML_ENCODER_A_PHASE_B_PIN,
        (GPIOn_enum) ML_ENCODER_A_PHASE_B_IOMUX, IN_UP);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_ENCODER_B_PHASE_B_PORT,
            ML_ENCODER_B_PHASE_B_PIN,
            (GPIOn_enum) ML_ENCODER_B_PHASE_B_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = exti_init_ex(EXTI_PB23, FALLING, 1U,
            encoder_edge_callback, &g_encoder_a_context);
    }
    if (status == ML_STATUS_OK) {
        status = exti_init_ex(EXTI_PB4, FALLING, 1U,
            encoder_edge_callback, &g_encoder_b_context);
    }
    if (status != ML_STATUS_OK) {
        return status;
    }

    Encoder_count1 = 0;
    Encoder_count2 = 0;
    g_encoder_initialized = true;
    return ML_STATUS_OK;
}

ml_status_t encoder_get_and_clear(int32_t *count1, int32_t *count2)
{
    uint32_t interrupt_state;

    if ((count1 == 0) || (count2 == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_encoder_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *count1 = Encoder_count1;
    *count2 = Encoder_count2;
    Encoder_count1 = 0;
    Encoder_count2 = 0;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}
