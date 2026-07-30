#include "ball_demo.h"

#include <string.h>

#include "ball_balance.h"
#include "ball_balance_config.h"

static const int16_t g_manual_offsets_us[] = {0, -100, 0, 100, 0};

typedef struct {
    uint8_t next_step;
    bool initialized;
} ball_demo_context_t;

static ball_demo_context_t g_demo;

ml_status_t ball_demo_init(void)
{
    ml_status_t status;

    memset(&g_demo, 0, sizeof(g_demo));
    status = ball_balance_enable(false);
    if (status == ML_STATUS_OK) {
        status = ball_balance_set_manual_servo_offset_us(0);
    }
    if (status == ML_STATUS_OK) {
        g_demo.initialized = true;
    }
    return status;
}

void ball_demo_process(void)
{
    ball_balance_status_t status;

    if (!g_demo.initialized ||
        (ball_balance_get_status(&status) != ML_STATUS_OK)) {
        return;
    }
    if (!status.enabled &&
        (status.state == BALL_BALANCE_VISION_LOST)) {
        (void) ball_balance_set_manual_servo_offset_us(0);
    }
}

ml_status_t ball_demo_short_press(void)
{
    ml_status_t status;

    if (!g_demo.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    status = ball_balance_set_manual_servo_offset_us(
        g_manual_offsets_us[g_demo.next_step]);
    if (status == ML_STATUS_OK) {
        ++g_demo.next_step;
        if (g_demo.next_step >=
            (sizeof(g_manual_offsets_us) / sizeof(g_manual_offsets_us[0]))) {
            g_demo.next_step = 0U;
        }
    }
    return status;
}

ml_status_t ball_demo_long_press(void)
{
    ball_balance_status_t status;
    ml_status_t result;

    if (!g_demo.initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    result = ball_balance_get_status(&status);
    if (result != ML_STATUS_OK) {
        return result;
    }
    if (status.enabled) {
        return ball_balance_enable(false);
    }
    if (!status.vision_ready) {
        return ML_STATUS_BUSY;
    }
#if BALL_CALIBRATION_SPEED_TEST
    return ball_balance_enable_speed_test(true);
#else
    result = ball_balance_set_target_cm(0.0f);
    if (result != ML_STATUS_OK) {
        return result;
    }
    return ball_balance_enable(true);
#endif
}
