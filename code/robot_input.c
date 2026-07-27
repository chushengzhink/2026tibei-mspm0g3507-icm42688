#include "robot_input.h"

#include "hmi_control.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_uart.h"
#include "robot_config.h"
#include "vision.h"

#define ROBOT_INPUT_KEY_DEBOUNCE_TICKS (3U)

typedef struct {
    uint8_t key_pressed_ticks;
    bool key_stable_pressed;
    bool center_press_pending;
} robot_input_context_t;

static robot_input_context_t g_robot_input;

static void robot_input_drain_uart(
    UART_Regs *uart, void (*feed_byte)(uint8_t))
{
    uint8_t byte;

    while (uart_try_read(uart, &byte) == ML_STATUS_OK) {
        feed_byte(byte);
    }
}

static void robot_input_key_tick(void)
{
    bool pressed = gpio_get(ML_KEY_CENTER_PORT,
        ML_KEY_CENTER_PIN) == ML_KEY_ACTIVE_LEVEL;
    bool stable;

    if (pressed) {
        if (g_robot_input.key_pressed_ticks < UINT8_MAX) {
            ++g_robot_input.key_pressed_ticks;
        }
    } else {
        g_robot_input.key_pressed_ticks = 0U;
    }
    stable = g_robot_input.key_pressed_ticks >=
        ROBOT_INPUT_KEY_DEBOUNCE_TICKS;
    if (stable && !g_robot_input.key_stable_pressed) {
        g_robot_input.center_press_pending = true;
    }
    g_robot_input.key_stable_pressed = stable;
}

ml_status_t robot_input_init(void)
{
    ml_status_t status;

    g_robot_input.key_pressed_ticks = 0U;
    g_robot_input.key_stable_pressed = false;
    g_robot_input.center_press_pending = false;
    status = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
        (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    if (status == ML_STATUS_OK) {
        status = uart_init(ROBOT_HMI_UART, ROBOT_HMI_BAUD, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(ROBOT_VISION_UART, ROBOT_VISION_BAUD, 2U);
    }
    hmi_control_init();
    vision_init();
    return status;
}

void robot_input_poll(uint32_t elapsed_ticks)
{
    robot_input_drain_uart(ROBOT_HMI_UART, hmi_control_feed_byte);
    robot_input_drain_uart(ROBOT_VISION_UART, vision_feed_byte);
    while (elapsed_ticks-- != 0U) {
        vision_tick_10ms();
        robot_input_key_tick();
    }
}

bool robot_input_take_hmi_start(void)
{
    return hmi_control_take_start_request();
}

bool robot_input_take_center_press(void)
{
    bool pending = g_robot_input.center_press_pending;

    g_robot_input.center_press_pending = false;
    return pending;
}

void robot_input_reset_vision(void)
{
    vision_reset();
}

bool robot_input_try_get_radius(uint8_t *radius_cm)
{
    return vision_try_get_radius(radius_cm);
}
