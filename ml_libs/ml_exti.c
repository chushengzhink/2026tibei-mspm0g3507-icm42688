#include "ml_exti.h"

#define ML_EXTI_PINS_PER_PORT (28U)
#define ML_EXTI_SLOT_COUNT    (ML_EXTI_PINS_PER_PORT * 2U)
#define ML_EXTI_PENDING_MASK  UINT32_C(0x0FFFFFFF)

typedef struct {
    exti_callback_t callback;
    void *context;
} exti_slot_t;

static exti_slot_t g_exti_slots[ML_EXTI_SLOT_COUNT];

static const GPIOn_enum g_pa_iomux[ML_EXTI_PINS_PER_PORT] = {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
    PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23,
    PA24, PA25, PA26, PA27
};

static const GPIOn_enum g_pb_iomux[ML_EXTI_PINS_PER_PORT] = {
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
    PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
    PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23,
    PB24, PB25, PB26, PB27
};

static ml_status_t exti_decode(EXTI_GPIO_enum exti_gpio,
    GPIO_Regs **port, uint32_t *pin_number, uint32_t *slot_index)
{
    uint32_t encoded = (uint32_t) exti_gpio;

    if ((port == 0) || (pin_number == 0) || (slot_index == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (encoded <= (uint32_t) EXTI_PA27) {
        *port = GPIOA;
        *pin_number = encoded;
        *slot_index = encoded;
        return ML_STATUS_OK;
    }
    if ((encoded >= (uint32_t) EXTI_PB0) &&
        (encoded <= (uint32_t) EXTI_PB27)) {
        *port = GPIOB;
        *pin_number = encoded - (uint32_t) EXTI_PB0;
        *slot_index = ML_EXTI_PINS_PER_PORT + *pin_number;
        return ML_STATUS_OK;
    }
    return ML_STATUS_INVALID_ARGUMENT;
}

ml_status_t exti_pin_init(EXTI_GPIO_enum exti_gpio, GPIOn_enum *gpion)
{
    GPIO_Regs *port;
    uint32_t pin_number;
    uint32_t slot_index;
    ml_status_t status;

    if (gpion == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = exti_decode(exti_gpio, &port, &pin_number, &slot_index);
    if (status != ML_STATUS_OK) {
        return status;
    }
    (void) slot_index;
    *gpion = (port == GPIOA) ?
        g_pa_iomux[pin_number] : g_pb_iomux[pin_number];
    return ML_STATUS_OK;
}

ml_status_t exti_init(
    EXTI_GPIO_enum exti_gpio, EXTI_MODE_enum mode, uint8_t priority)
{
    return exti_init_ex(exti_gpio, mode, priority, 0, 0);
}

ml_status_t exti_init_ex(EXTI_GPIO_enum exti_gpio, EXTI_MODE_enum mode,
    uint8_t priority, exti_callback_t callback, void *context)
{
    GPIO_Regs *port;
    IRQn_Type irq;
    GPIOn_enum iomux;
    uint32_t pin_number;
    uint32_t slot_index;
    uint32_t local_pin;
    uint32_t polarity;
    uint32_t polarity_mask;
    uint32_t pin_mask;
    ml_status_t status;

    if (((mode != RISING) && (mode != FALLING)) ||
        (priority > ML_NVIC_PRIORITY_MAX)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = exti_decode(exti_gpio, &port, &pin_number, &slot_index);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = exti_pin_init(exti_gpio, &iomux);
    if (status != ML_STATUS_OK) {
        return status;
    }

    pin_mask = UINT32_C(1) << pin_number;
    DL_GPIO_initDigitalInputFeatures((uint32_t) iomux,
        DL_GPIO_INVERSION_DISABLE,
        (mode == RISING) ? DL_GPIO_RESISTOR_PULL_DOWN : DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    if (pin_number < 16U) {
        local_pin = pin_number;
        polarity_mask = UINT32_C(3) << (2U * local_pin);
        polarity = UINT32_C(1) <<
            ((2U * local_pin) + ((mode == FALLING) ? 1U : 0U));
        port->POLARITY15_0 =
            (port->POLARITY15_0 & ~polarity_mask) | polarity;
    } else {
        local_pin = pin_number - 16U;
        polarity_mask = UINT32_C(3) << (2U * local_pin);
        polarity = UINT32_C(1) <<
            ((2U * local_pin) + ((mode == FALLING) ? 1U : 0U));
        port->POLARITY31_16 =
            (port->POLARITY31_16 & ~polarity_mask) | polarity;
    }

    g_exti_slots[slot_index].callback = callback;
    g_exti_slots[slot_index].context = context;

    DL_GPIO_clearInterruptStatus(port, pin_mask);
    DL_GPIO_enableInterrupt(port, pin_mask);
    irq = (port == GPIOA) ? GPIOA_INT_IRQn : GPIOB_INT_IRQn;
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);

    return ML_STATUS_OK;
}

static void exti_irq_dispatch_port(GPIO_Regs *port, uint32_t slot_offset)
{
    uint32_t pending = port->CPU_INT.MIS & ML_EXTI_PENDING_MASK;
    uint32_t pin_number;

    for (pin_number = 0U; pin_number < ML_EXTI_PINS_PER_PORT; ++pin_number) {
        uint32_t pin_mask = UINT32_C(1) << pin_number;
        exti_slot_t *slot = &g_exti_slots[slot_offset + pin_number];

        if ((pending & pin_mask) != 0U) {
            if (slot->callback != 0) {
                slot->callback(pin_number, slot->context);
            }
            port->CPU_INT.ICLR = pin_mask;
        }
    }
}

void exti_irq_dispatch(void)
{
    exti_irq_dispatch_port(GPIOA, 0U);
    exti_irq_dispatch_port(GPIOB, ML_EXTI_PINS_PER_PORT);
}
