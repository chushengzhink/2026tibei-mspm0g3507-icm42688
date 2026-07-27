#include "hmi_control.h"
#include "vision.h"

#include <stdio.h>

static int g_failures;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static void feed_radius(uint32_t radius)
{
    vision_feed_byte(0xAAU);
    vision_feed_byte(0x55U);
    vision_feed_byte((uint8_t) radius);
    vision_feed_byte((uint8_t) (radius >> 8));
    vision_feed_byte((uint8_t) (radius >> 16));
    vision_feed_byte((uint8_t) (radius >> 24));
}

static void test_hmi(void)
{
    hmi_control_init();
    hmi_control_feed_byte(0x99U);
    hmi_control_feed_byte(0x31U);
    hmi_control_feed_byte(0x00U);
    check(!hmi_control_take_start_request(),
        "noise and a truncated HMI frame do not start");
    hmi_control_feed_byte(0x31U);
    hmi_control_feed_byte(0x0DU);
    hmi_control_feed_byte(0x0AU);
    check(hmi_control_take_start_request(),
        "complete HMI start frame is accepted");
    check(!hmi_control_take_start_request(),
        "HMI start event is consumed once");
}

static void test_vision(void)
{
    uint8_t radius = 0U;
    uint8_t index;

    vision_init();
    vision_feed_byte(0xAAU);
    vision_feed_byte(0x00U);
    feed_radius(20U);
    for (index = 0U; index < 8U; ++index) {
        feed_radius(40U);
    }
    check(vision_try_get_radius(&radius) && radius == 40U,
        "vision parser locks a valid 75-percent majority radius");

    vision_reset();
    for (index = 0U; index < 7U; ++index) {
        feed_radius(45U);
    }
    for (index = 0U; index < 100U; ++index) {
        vision_tick_10ms();
    }
    check(vision_valid_packet_count() == 0U &&
        !vision_try_get_radius(&radius),
        "unlocked vision window expires after one second");

    vision_feed_byte(0xAAU);
    vision_feed_byte(0x55U);
    vision_feed_byte(0x32U);
    vision_reset();
    for (index = 0U; index < 8U; ++index) {
        feed_radius(50U);
    }
    check(vision_try_get_radius(&radius) && radius == 50U,
        "vision reset recovers from a partial frame");
}

int main(void)
{
    test_hmi();
    test_vision();
    if (g_failures == 0) {
        printf("PASS: HMI and vision parser tests\n");
    }
    return g_failures == 0 ? 0 : 1;
}
