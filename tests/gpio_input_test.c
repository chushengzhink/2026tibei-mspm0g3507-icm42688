#include "ml_gpio.h"

#include <stdio.h>

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;

static int g_failures;
static uint32_t g_disable_calls;
static uint32_t g_enable_calls;
static uint32_t g_output_init_calls;
static uint32_t g_input_init_calls;
static uint32_t g_last_resistor;
static uint32_t g_last_hysteresis;
static uint32_t g_event_counter;
static uint32_t g_last_disable_event;
static uint32_t g_last_input_init_event;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

void DL_GPIO_initDigitalOutput(uint32_t pincm_index)
{
    (void) pincm_index;
    ++g_output_init_calls;
}

void DL_GPIO_enableOutput(GPIO_Regs *gpio, uint32_t pins)
{
    (void) gpio;
    (void) pins;
    ++g_enable_calls;
}

void DL_GPIO_disableOutput(GPIO_Regs *gpio, uint32_t pins)
{
    (void) gpio;
    (void) pins;
    ++g_disable_calls;
    g_last_disable_event = ++g_event_counter;
}

void DL_GPIO_initDigitalInputFeatures(uint32_t pincm_index,
    uint32_t inversion, uint32_t resistor,
    uint32_t hysteresis, uint32_t wakeup)
{
    (void) pincm_index;
    (void) inversion;
    (void) wakeup;
    ++g_input_init_calls;
    g_last_input_init_event = ++g_event_counter;
    g_last_resistor = resistor;
    g_last_hysteresis = hysteresis;
}

int main(void)
{
    check(gpio_init(GPIOA, DL_GPIO_PIN_27, PA27, IN_UP) ==
            ML_STATUS_OK &&
        g_disable_calls == 1U && g_input_init_calls == 1U &&
        g_last_disable_event < g_last_input_init_event &&
        g_last_resistor == DL_GPIO_RESISTOR_PULL_UP,
        "IN_UP disables output before enabling the pull-up input");
    check(gpio_init(GPIOA, DL_GPIO_PIN_27, PA27, IN_DOWN) ==
            ML_STATUS_OK &&
        g_disable_calls == 2U && g_input_init_calls == 2U &&
        g_last_disable_event < g_last_input_init_event &&
        g_last_resistor == DL_GPIO_RESISTOR_PULL_DOWN,
        "IN_DOWN also clears a stale output-enable bit");
    check(gpio_init(GPIOA, DL_GPIO_PIN_27, PA27, IN_FLOAT) ==
            ML_STATUS_OK &&
        g_disable_calls == 3U && g_input_init_calls == 3U &&
        g_last_disable_event < g_last_input_init_event &&
        g_last_resistor == DL_GPIO_RESISTOR_NONE &&
        g_last_hysteresis == DL_GPIO_HYSTERESIS_ENABLE,
        "IN_FLOAT clears output enable and preserves hysteresis");
    check(gpio_init(GPIOA, DL_GPIO_PIN_27, PA27, OUT) ==
            ML_STATUS_OK &&
        g_disable_calls == 3U && g_output_init_calls == 1U &&
        g_enable_calls == 1U,
        "OUT keeps the existing low-before-enable behavior");
    check(gpio_init(GPIOB, DL_GPIO_PIN_27, PA27, IN_UP) ==
            ML_STATUS_INVALID_ARGUMENT &&
        g_disable_calls == 3U,
        "invalid port/pin mappings do not touch output enable");

    if (g_failures == 0) {
        puts("GPIO input protection tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
