#include "ml_encoder.h"

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_exti.h"
#include "ml_gpio.h"
#include "ml_quadrature.h"

typedef struct {
    GPIO_Regs *phase_a_port;
    uint32_t phase_a_pin;
    GPIO_Regs *phase_b_port;
    uint32_t phase_b_pin;
    volatile int32_t *delta_count;
    volatile int32_t *total_count;
    volatile uint32_t phase_a_edges;
    volatile uint32_t phase_b_edges;
    ml_quadrature_t decoder;
    int8_t polarity;
} encoder_context_t;

volatile int32_t Encoder_count1 = 0;
volatile int32_t Encoder_count2 = 0;

static volatile int32_t g_encoder_total_a;
static volatile int32_t g_encoder_total_b;
static bool g_encoder_initialized;

static encoder_context_t g_encoder_a_context = {
    ML_ENCODER_A_PHASE_A_PORT,
    ML_ENCODER_A_PHASE_A_PIN,
    ML_ENCODER_A_PHASE_B_PORT,
    ML_ENCODER_A_PHASE_B_PIN,
    &Encoder_count1,
    &g_encoder_total_a,
    0U,
    0U,
    {0U, 1, 0U, false},
    1
};

static encoder_context_t g_encoder_b_context = {
    ML_ENCODER_B_PHASE_A_PORT,
    ML_ENCODER_B_PHASE_A_PIN,
    ML_ENCODER_B_PHASE_B_PORT,
    ML_ENCODER_B_PHASE_B_PIN,
    &Encoder_count2,
    &g_encoder_total_b,
    0U,
    0U,
    {0U, 1, 0U, false},
    -1
};

static void encoder_add_saturated(
    volatile int32_t *count, int8_t delta)
{
    if ((delta > 0) && (*count < INT32_MAX)) {
        ++(*count);
    } else if ((delta < 0) && (*count > INT32_MIN)) {
        --(*count);
    }
}

static void encoder_edge_callback(uint32_t pin, void *context)
{
    encoder_context_t *encoder = (encoder_context_t *) context;
    uint32_t pin_mask;
    int8_t delta;

    if ((encoder == 0) || (pin >= 32U)) {
        return;
    }
    pin_mask = UINT32_C(1) << pin;
    if ((pin_mask == encoder->phase_a_pin) &&
        (encoder->phase_a_edges < UINT32_MAX)) {
        ++encoder->phase_a_edges;
    } else if ((pin_mask == encoder->phase_b_pin) &&
        (encoder->phase_b_edges < UINT32_MAX)) {
        ++encoder->phase_b_edges;
    }
    delta = ml_quadrature_update(&encoder->decoder,
        gpio_get(encoder->phase_a_port, encoder->phase_a_pin),
        gpio_get(encoder->phase_b_port, encoder->phase_b_pin));
    encoder_add_saturated(encoder->delta_count, delta);
    encoder_add_saturated(encoder->total_count, delta);
}

static ml_status_t encoder_register_interrupts(void)
{
    ml_status_t status;

    status = exti_init_ex(EXTI_PB23, BOTH, 1U,
        encoder_edge_callback, &g_encoder_a_context);
    if (status == ML_STATUS_OK) {
        status = exti_init_ex(EXTI_PB12, BOTH, 1U,
            encoder_edge_callback, &g_encoder_a_context);
    }
    if (status == ML_STATUS_OK) {
        status = exti_init_ex(EXTI_PB4, BOTH, 1U,
            encoder_edge_callback, &g_encoder_b_context);
    }
    if (status == ML_STATUS_OK) {
        status = exti_init_ex(EXTI_PB5, BOTH, 1U,
            encoder_edge_callback, &g_encoder_b_context);
    }
    return status;
}

ml_status_t ml_encoder_init(void)
{
    ml_status_t status;

    if (g_encoder_initialized) {
        return ML_STATUS_OK;
    }
    status = gpio_init(ML_ENCODER_A_PHASE_A_PORT,
        ML_ENCODER_A_PHASE_A_PIN,
        (GPIOn_enum) ML_ENCODER_A_PHASE_A_IOMUX, IN_UP);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_ENCODER_A_PHASE_B_PORT,
            ML_ENCODER_A_PHASE_B_PIN,
            (GPIOn_enum) ML_ENCODER_A_PHASE_B_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_ENCODER_B_PHASE_A_PORT,
            ML_ENCODER_B_PHASE_A_PIN,
            (GPIOn_enum) ML_ENCODER_B_PHASE_A_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_ENCODER_B_PHASE_B_PORT,
            ML_ENCODER_B_PHASE_B_PIN,
            (GPIOn_enum) ML_ENCODER_B_PHASE_B_IOMUX, IN_UP);
    }
    if (status != ML_STATUS_OK) {
        return status;
    }
    delay_ms(1U);
    ml_quadrature_init(&g_encoder_a_context.decoder,
        gpio_get(g_encoder_a_context.phase_a_port,
            g_encoder_a_context.phase_a_pin),
        gpio_get(g_encoder_a_context.phase_b_port,
            g_encoder_a_context.phase_b_pin),
        g_encoder_a_context.polarity);
    ml_quadrature_init(&g_encoder_b_context.decoder,
        gpio_get(g_encoder_b_context.phase_a_port,
            g_encoder_b_context.phase_a_pin),
        gpio_get(g_encoder_b_context.phase_b_port,
            g_encoder_b_context.phase_b_pin),
        g_encoder_b_context.polarity);

    Encoder_count1 = 0;
    Encoder_count2 = 0;
    g_encoder_total_a = 0;
    g_encoder_total_b = 0;
    g_encoder_a_context.phase_a_edges = 0U;
    g_encoder_a_context.phase_b_edges = 0U;
    g_encoder_b_context.phase_a_edges = 0U;
    g_encoder_b_context.phase_b_edges = 0U;
    status = encoder_register_interrupts();
    if (status != ML_STATUS_OK) {
        return status;
    }
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

ml_status_t ml_encoder_get_diagnostics(
    ml_encoder_diagnostics_t *diagnostics)
{
    uint32_t interrupt_state;

    if (diagnostics == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_encoder_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    diagnostics->total_count_a = g_encoder_total_a;
    diagnostics->total_count_b = g_encoder_total_b;
    diagnostics->invalid_transitions_a =
        g_encoder_a_context.decoder.invalid_transitions;
    diagnostics->invalid_transitions_b =
        g_encoder_b_context.decoder.invalid_transitions;
    diagnostics->state_a = g_encoder_a_context.decoder.state;
    diagnostics->state_b = g_encoder_b_context.decoder.state;
    diagnostics->live_state_a = (uint8_t) (
        (gpio_get(g_encoder_a_context.phase_a_port,
            g_encoder_a_context.phase_a_pin) << 1U) |
        gpio_get(g_encoder_a_context.phase_b_port,
            g_encoder_a_context.phase_b_pin));
    diagnostics->live_state_b = (uint8_t) (
        (gpio_get(g_encoder_b_context.phase_a_port,
            g_encoder_b_context.phase_a_pin) << 1U) |
        gpio_get(g_encoder_b_context.phase_b_port,
            g_encoder_b_context.phase_b_pin));
    diagnostics->phase_a_edges_a = g_encoder_a_context.phase_a_edges;
    diagnostics->phase_b_edges_a = g_encoder_a_context.phase_b_edges;
    diagnostics->phase_a_edges_b = g_encoder_b_context.phase_a_edges;
    diagnostics->phase_b_edges_b = g_encoder_b_context.phase_b_edges;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return ML_STATUS_OK;
}
