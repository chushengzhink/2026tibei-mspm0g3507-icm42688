#include "vision.h"

#include <string.h>

#define VISION_RADIUS_MIN          (30U)
#define VISION_RADIUS_MAX          (60U)
#define VISION_RADIUS_COUNT        (31U)
#define VISION_MIN_PACKETS         (8U)
#define VISION_WINDOW_TICKS        (100U)

typedef enum {
    VISION_WAIT_AA = 0,
    VISION_WAIT_55,
    VISION_READ_PAYLOAD
} vision_parser_state_t;

static vision_parser_state_t g_parser_state;
static uint8_t g_payload[4];
static uint8_t g_payload_index;
static uint8_t g_histogram[VISION_RADIUS_COUNT];
static uint16_t g_valid_packets;
static uint16_t g_window_ticks;
static bool g_locked;
static uint8_t g_locked_radius;

static void vision_clear_window(void)
{
    (void) memset(g_histogram, 0, sizeof(g_histogram));
    g_valid_packets = 0U;
    g_window_ticks = 0U;
}

static void vision_consider_value(uint32_t value)
{
    uint8_t index;
    uint8_t best_count = 0U;
    uint8_t best_radius = 0U;

    if (g_locked || (value < VISION_RADIUS_MIN) ||
        (value > VISION_RADIUS_MAX)) {
        return;
    }

    index = (uint8_t) (value - VISION_RADIUS_MIN);
    if (g_histogram[index] != UINT8_MAX) {
        ++g_histogram[index];
    }
    if (g_valid_packets != UINT16_MAX) {
        ++g_valid_packets;
    }

    if (g_valid_packets < VISION_MIN_PACKETS) {
        return;
    }
    for (index = 0U; index < VISION_RADIUS_COUNT; ++index) {
        if (g_histogram[index] > best_count) {
            best_count = g_histogram[index];
            best_radius = (uint8_t) (index + VISION_RADIUS_MIN);
        }
    }
    if (((uint32_t) best_count * 4U) >=
        ((uint32_t) g_valid_packets * 3U)) {
        g_locked_radius = best_radius;
        g_locked = true;
    }
}

void vision_init(void)
{
    vision_reset();
}

void vision_reset(void)
{
    g_parser_state = VISION_WAIT_AA;
    g_payload_index = 0U;
    g_locked = false;
    g_locked_radius = 0U;
    vision_clear_window();
}

void vision_feed_byte(uint8_t byte)
{
    uint32_t value;

    if (g_parser_state == VISION_WAIT_AA) {
        if (byte == 0xAAU) {
            g_parser_state = VISION_WAIT_55;
        }
        return;
    }
    if (g_parser_state == VISION_WAIT_55) {
        if (byte == 0x55U) {
            g_payload_index = 0U;
            g_parser_state = VISION_READ_PAYLOAD;
        } else if (byte != 0xAAU) {
            g_parser_state = VISION_WAIT_AA;
        }
        return;
    }

    g_payload[g_payload_index++] = byte;
    if (g_payload_index < sizeof(g_payload)) {
        return;
    }
    value = (uint32_t) g_payload[0] |
        ((uint32_t) g_payload[1] << 8) |
        ((uint32_t) g_payload[2] << 16) |
        ((uint32_t) g_payload[3] << 24);
    vision_consider_value(value);
    g_payload_index = 0U;
    g_parser_state = VISION_WAIT_AA;
}

void vision_tick_10ms(void)
{
    if (g_locked) {
        return;
    }
    if (++g_window_ticks >= VISION_WINDOW_TICKS) {
        vision_clear_window();
    }
}

bool vision_try_get_radius(uint8_t *radius_cm)
{
    if ((radius_cm == 0) || !g_locked) {
        return false;
    }
    *radius_cm = g_locked_radius;
    return true;
}

uint16_t vision_valid_packet_count(void)
{
    return g_valid_packets;
}
