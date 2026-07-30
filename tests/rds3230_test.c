#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "rds3230.h"

UART_Regs g_test_uart2;
GPTIMER_Regs g_test_tima1;
GPTIMER_Regs g_test_timg6;

static uint32_t g_last_duty;
static uint32_t g_update_count;

ml_status_t pwm_init(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t frequency_hz)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    assert(frequency_hz == 50U);
    return ML_STATUS_OK;
}

ml_status_t pwm_update(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint32_t duty)
{
    assert(timer == TIMA1);
    assert(channel == DL_TIMER_CC_1_INDEX);
    g_last_duty = duty;
    ++g_update_count;
    return ML_STATUS_OK;
}

int main(void)
{
    rds3230_t servo;

    assert(rds3230_init(0, TIMA1, DL_TIMER_CC_1_INDEX,
        50U, 1300U, 1500U, 1700U, 2000U) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(rds3230_init(&servo, TIMA1, DL_TIMER_CC_1_INDEX,
        0U, 1300U, 1500U, 1700U, 2000U) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(rds3230_init(&servo, TIMA1, DL_TIMER_CC_1_INDEX,
        50U, 1500U, 1500U, 1700U, 2000U) ==
        ML_STATUS_INVALID_ARGUMENT);

    assert(rds3230_init(&servo, TIMA1, DL_TIMER_CC_1_INDEX,
        50U, 1300U, 1500U, 1700U, 2000U) == ML_STATUS_OK);
    assert(g_last_duty == 3750U);
    assert(rds3230_get_current_us(&servo) == 1500U);
    assert(rds3230_set_target_us(&servo, 1299U) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(rds3230_set_target_us(&servo, 1701U) ==
        ML_STATUS_INVALID_ARGUMENT);
    assert(rds3230_set_target_us(&servo, 1700U) == ML_STATUS_OK);

    assert(rds3230_update(&servo, 0U) == ML_STATUS_OK);
    assert(rds3230_get_current_us(&servo) == 1500U);
    assert(rds3230_update(&servo, 10U) == ML_STATUS_OK);
    assert(rds3230_get_current_us(&servo) == 1520U);
    assert(g_last_duty == 3800U);
    assert(rds3230_update(&servo, 1010U) == ML_STATUS_OK);
    assert(rds3230_get_current_us(&servo) == 1700U);
    assert(g_last_duty == 4250U);

    assert(rds3230_set_target_us(&servo, 1300U) == ML_STATUS_OK);
    assert(rds3230_update(&servo, 1060U) == ML_STATUS_OK);
    assert(rds3230_get_current_us(&servo) == 1600U);
    assert(rds3230_set_center(&servo) == ML_STATUS_OK);
    assert(rds3230_get_target_us(&servo) == 1500U);
    assert(g_update_count >= 4U);

    printf("rds3230 tests: PASS\n");
    return 0;
}
