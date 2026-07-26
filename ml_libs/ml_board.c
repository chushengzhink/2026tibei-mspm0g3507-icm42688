#include "ml_board.h"

#include "ml_gpio.h"

static ml_board_owner_t g_resource_owners[ML_BOARD_RESOURCE_COUNT];

ml_status_t board_led_init(void)
{
    ml_status_t status = gpio_init(
        ML_BOARD_LED_PORT, ML_BOARD_LED_PIN,
        (GPIOn_enum) ML_BOARD_LED_IOMUX, OUT);

    if (status == ML_STATUS_OK) {
        board_led_on();
    }
    return status;
}

void board_led_on(void)
{
    gpio_set(ML_BOARD_LED_PORT, ML_BOARD_LED_PIN, ML_BOARD_LED_ACTIVE_LEVEL);
}

void board_led_off(void)
{
    gpio_set(ML_BOARD_LED_PORT, ML_BOARD_LED_PIN, !ML_BOARD_LED_ACTIVE_LEVEL);
}

void board_led_toggle(void)
{
    gpio_toggle(ML_BOARD_LED_PORT, ML_BOARD_LED_PIN);
}

ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner)
{
    if (((uint32_t) resource >= (uint32_t) ML_BOARD_RESOURCE_COUNT) ||
        (owner == ML_BOARD_OWNER_NONE)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if ((g_resource_owners[resource] != ML_BOARD_OWNER_NONE) &&
        (g_resource_owners[resource] != owner)) {
        return ML_STATUS_BUSY;
    }
    g_resource_owners[resource] = owner;
    return ML_STATUS_OK;
}

void board_resource_release(
    ml_board_resource_t resource, ml_board_owner_t owner)
{
    if (((uint32_t) resource < (uint32_t) ML_BOARD_RESOURCE_COUNT) &&
        (g_resource_owners[resource] == owner)) {
        g_resource_owners[resource] = ML_BOARD_OWNER_NONE;
    }
}
