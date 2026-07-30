#include "hmi_control.h"

#include "robot_config.h"

typedef enum {
    HMI_WAIT_START_BYTE = 0,
    HMI_WAIT_CR,
    HMI_WAIT_LF
} hmi_parser_state_t;

static hmi_parser_state_t g_parser_state;
static bool g_start_pending;

void hmi_control_init(void)
{
    g_parser_state = HMI_WAIT_START_BYTE;
    g_start_pending = false;
}

void hmi_control_feed_byte(uint8_t byte)
{
    if (g_parser_state == HMI_WAIT_START_BYTE) {
        if (byte == ROBOT_HMI_START_BYTE) {
            g_parser_state = HMI_WAIT_CR;
        }
        return;
    }

    if (g_parser_state == HMI_WAIT_CR) {
        if (byte == ROBOT_HMI_START_CR) {
            g_parser_state = HMI_WAIT_LF;
        } else if (byte != ROBOT_HMI_START_BYTE) {
            g_parser_state = HMI_WAIT_START_BYTE;
        }
        return;
    }

    if (byte == ROBOT_HMI_START_LF) {
        g_start_pending = true;
        g_parser_state = HMI_WAIT_START_BYTE;
    } else if (byte == ROBOT_HMI_START_BYTE) {
        g_parser_state = HMI_WAIT_CR;
    } else {
        g_parser_state = HMI_WAIT_START_BYTE;
    }
}

bool hmi_control_take_start_request(void)
{
    bool pending = g_start_pending;

    g_start_pending = false;
    return pending;
}
