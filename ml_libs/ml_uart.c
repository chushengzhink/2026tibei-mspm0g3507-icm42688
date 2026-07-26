#include "ml_uart.h"

#include <stdio.h>

typedef struct {
    UART_Regs *uart;
    IRQn_Type irq;
    uint32_t tx_iomux;
    uint32_t tx_function;
    uint32_t rx_iomux;
    uint32_t rx_function;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_count;
    uint8_t rx_buffer[ML_UART_RX_BUFFER_SIZE];
    uart_rx_callback_t callback;
    void *context;
    bool initialized;
} uart_state_t;

static uart_state_t g_uart_states[] = {
    {UART0, UART0_INT_IRQn, ML_UART0_TX_IOMUX, ML_UART0_TX_FUNCTION,
        ML_UART0_RX_IOMUX, ML_UART0_RX_FUNCTION,
        0U, 0U, 0U, {0}, 0, 0, false},
    {UART1, UART1_INT_IRQn, ML_UART1_TX_IOMUX, ML_UART1_TX_FUNCTION,
        ML_UART1_RX_IOMUX, ML_UART1_RX_FUNCTION,
        0U, 0U, 0U, {0}, 0, 0, false},
    {UART2, UART2_INT_IRQn, ML_UART2_TX_IOMUX, ML_UART2_TX_FUNCTION,
        ML_UART2_RX_IOMUX, ML_UART2_RX_FUNCTION,
        0U, 0U, 0U, {0}, 0, 0, false},
    {UART3, UART3_INT_IRQn, ML_UART3_TX_IOMUX, ML_UART3_TX_FUNCTION,
        ML_UART3_RX_IOMUX, ML_UART3_RX_FUNCTION,
        0U, 0U, 0U, {0}, 0, 0, false}
};

static uart_state_t *uart_find_state(UART_Regs *uart)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(g_uart_states) / sizeof(g_uart_states[0])); ++i) {
        if (g_uart_states[i].uart == uart) {
            return &g_uart_states[i];
        }
    }
    return 0;
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE {
    int handle;
};
#endif
FILE __stdout;

void _sys_exit(int return_code)
{
    (void) return_code;
    while (1) {
    }
}
#endif

int fputc(int character, FILE *stream)
{
    (void) stream;
    if (uart_sendbyte(ML_PRINTF_UART, (uint8_t) character) != ML_STATUS_OK) {
        return EOF;
    }
    return character;
}

ml_status_t uart_init(UART_Regs *uart, uint32_t baud, uint32_t priority)
{
    uart_state_t *state = uart_find_state(uart);
    uint64_t scaled_divisor;
    uint32_t integer_divisor;
    uint32_t fractional_divisor;
    DL_UART_Main_ClockConfig clock_config;
    DL_UART_Main_Config uart_config;

    if ((state == 0) || (baud == 0U) ||
        (priority > ML_NVIC_PRIORITY_MAX)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    scaled_divisor = (((uint64_t) ML_MFCLK_HZ * 4U) + (baud / 2U)) / baud;
    integer_divisor = (uint32_t) (scaled_divisor / 64U);
    fractional_divisor = (uint32_t) (scaled_divisor % 64U);
    if ((integer_divisor == 0U) || (integer_divisor > 65535U)) {
        return ML_STATUS_UNSUPPORTED;
    }
    if ((uart == UART3) &&
        (board_resource_claim(ML_BOARD_RESOURCE_PA26,
            ML_BOARD_OWNER_UART3) != ML_STATUS_OK)) {
        return ML_STATUS_BUSY;
    }

    clock_config.clockSel = DL_UART_MAIN_CLOCK_MFCLK;
    clock_config.divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1;

    uart_config.mode = DL_UART_MAIN_MODE_NORMAL;
    uart_config.direction = DL_UART_MAIN_DIRECTION_TX_RX;
    uart_config.flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE;
    uart_config.parity = DL_UART_MAIN_PARITY_NONE;
    uart_config.wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS;
    uart_config.stopBits = DL_UART_MAIN_STOP_BITS_ONE;

    DL_UART_Main_reset(uart);
    DL_UART_Main_enablePower(uart);
    delay_cycles(16U);
    DL_GPIO_initPeripheralOutputFunction(
        state->tx_iomux, state->tx_function);
    DL_GPIO_initPeripheralInputFunction(
        state->rx_iomux, state->rx_function);
    DL_UART_Main_setClockConfig(uart, &clock_config);
    DL_UART_Main_init(uart, &uart_config);
    DL_UART_Main_setOversampling(uart, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        uart, integer_divisor, fractional_divisor);

    state->head = 0U;
    state->tail = 0U;
    state->overflow_count = 0U;
    state->initialized = true;

    DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(uart);
    NVIC_ClearPendingIRQ(state->irq);
    NVIC_SetPriority(state->irq, priority);
    NVIC_EnableIRQ(state->irq);

    return ML_STATUS_OK;
}

ml_status_t uart_sendbyte(UART_Regs *uart, uint8_t byte)
{
    uart_state_t *state = uart_find_state(uart);
    uint32_t timeout = ML_UART_TX_TIMEOUT_LOOPS;

    if ((state == 0) || !state->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    while (DL_UART_isBusy(uart)) {
        if (timeout-- == 0U) {
            return ML_STATUS_TIMEOUT;
        }
    }
    DL_UART_Main_transmitData(uart, byte);
    return ML_STATUS_OK;
}

ml_status_t uart_try_read(UART_Regs *uart, uint8_t *byte)
{
    uart_state_t *state = uart_find_state(uart);
    uint16_t tail;

    if ((state == 0) || (byte == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!state->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (state->head == state->tail) {
        return ML_STATUS_BUFFER_EMPTY;
    }

    tail = state->tail;
    *byte = state->rx_buffer[tail % ML_UART_RX_BUFFER_SIZE];
    state->tail = (uint16_t) (tail + 1U);
    return ML_STATUS_OK;
}

uint8_t uart_getbyte(UART_Regs *uart)
{
    uint8_t byte = 0U;

    (void) uart_try_read(uart, &byte);
    return byte;
}

ml_status_t uart_set_rx_callback(
    UART_Regs *uart, uart_rx_callback_t callback, void *context)
{
    uart_state_t *state = uart_find_state(uart);

    if (state == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    state->callback = callback;
    state->context = context;
    return ML_STATUS_OK;
}

uint32_t uart_get_rx_overflow_count(UART_Regs *uart)
{
    uart_state_t *state = uart_find_state(uart);

    return (state == 0) ? 0U : state->overflow_count;
}

void uart_irq_dispatch(UART_Regs *uart)
{
    uart_state_t *state = uart_find_state(uart);

    if (state == 0) {
        return;
    }

    (void) DL_UART_getPendingInterrupt(uart);
    while (!DL_UART_Main_isRXFIFOEmpty(uart)) {
        uint8_t byte = DL_UART_Main_receiveData(uart);
        uint16_t head = state->head;

        if ((uint16_t) (head - state->tail) >= ML_UART_RX_BUFFER_SIZE) {
            ++state->overflow_count;
        } else {
            state->rx_buffer[head % ML_UART_RX_BUFFER_SIZE] = byte;
            state->head = (uint16_t) (head + 1U);
            if (state->callback != 0) {
                state->callback(uart, byte, state->context);
            }
        }
    }
}
