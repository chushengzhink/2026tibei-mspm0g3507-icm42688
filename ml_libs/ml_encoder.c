#include "ml_encoder.h"

#include "ml_board.h"
#include "ml_exti.h"
#include "ml_gpio.h"

typedef struct {
    GPIO_Regs *phase_b_port;
    uint32_t phase_b_pin;
    volatile int32_t *count;
    int8_t polarity;
} encoder_context_t;

volatile int32_t Encoder_count1 = 0;
volatile int32_t Encoder_count2 = 0;

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

ml_status_t ml_encoder_init(void)
{
    ml_status_t status;

    if (g_encoder_initialized) {
        return ML_STATUS_OK;
    }
    status = gpio_init(ML_ENCODER_A_PHASE_B_PORT,
        ML_ENCODER_A_PHASE_B_PIN,
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

ml_status_t ml_encoder_read_and_clear(
    int32_t *count_a, int32_t *count_b)
{
    uint32_t interrupt_state;

    if ((count_a == 0) || (count_b == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_encoder_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *count_a = Encoder_count1;
    *count_b = Encoder_count2;
    Encoder_count1 = 0;
    Encoder_count2 = 0;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}
