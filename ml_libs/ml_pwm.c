#include "ml_pwm.h"

typedef struct {
    GPTIMER_Regs *timer;
    uint32_t divided_clock_hz;
    uint16_t frequency_hz;
    uint32_t channel_mask;
    bool initialized;
} pwm_state_t;

static ml_status_t pwm_claim_pin_resource(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel)
{
    if ((timer == TIMG0) && (channel == DL_TIMER_CC_1_INDEX)) {
        return board_resource_claim(
            ML_BOARD_RESOURCE_PA13, ML_BOARD_OWNER_PWM_TIMG0);
    }
    if ((timer == TIMG6) && (channel == DL_TIMER_CC_1_INDEX)) {
        return board_resource_claim(
            ML_BOARD_RESOURCE_PB7, ML_BOARD_OWNER_PWM_TIMG6);
    }
    if ((timer == TIMG8) && (channel == DL_TIMER_CC_0_INDEX)) {
        return board_resource_claim(
            ML_BOARD_RESOURCE_PA26, ML_BOARD_OWNER_PWM_TIMG8);
    }
    if ((timer == TIMG12) && (channel == DL_TIMER_CC_1_INDEX)) {
        return board_resource_claim(
            ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_PWM_TIMG12);
    }
    return ML_STATUS_OK;
}

static pwm_state_t g_pwm_states[] = {
    {TIMG0, 5000000UL, 0U, 0U, false},
    {TIMG6, 10000000UL, 0U, 0U, false},
    {TIMG7, 10000000UL, 0U, 0U, false},
    {TIMG8, 5000000UL, 0U, 0U, false},
    {TIMG12, 10000000UL, 0U, 0U, false}
};

static pwm_state_t *pwm_find_state(GPTIMER_Regs *timer)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(g_pwm_states) / sizeof(g_pwm_states[0])); ++i) {
        if (g_pwm_states[i].timer == timer) {
            return &g_pwm_states[i];
        }
    }
    return 0;
}

static uint32_t pwm_channel_output_mask(DL_TIMER_CC_INDEX channel)
{
    if (channel == DL_TIMER_CC_0_INDEX) {
        return DL_TIMER_CC0_OUTPUT;
    }
    if (channel == DL_TIMER_CC_1_INDEX) {
        return DL_TIMER_CC1_OUTPUT;
    }
    return 0U;
}

static ml_status_t pwm_configure_pin(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel)
{
    if ((timer == TIMG0) && (channel == DL_TIMER_CC_0_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG0_CH0_IOMUX, ML_PWM_TIMG0_CH0_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG0_CH0_PORT, ML_PWM_TIMG0_CH0_PIN);
    } else if ((timer == TIMG0) && (channel == DL_TIMER_CC_1_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG0_CH1_IOMUX, ML_PWM_TIMG0_CH1_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG0_CH1_PORT, ML_PWM_TIMG0_CH1_PIN);
    } else if ((timer == TIMG6) && (channel == DL_TIMER_CC_0_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG6_CH0_IOMUX, ML_PWM_TIMG6_CH0_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG6_CH0_PORT, ML_PWM_TIMG6_CH0_PIN);
    } else if ((timer == TIMG6) && (channel == DL_TIMER_CC_1_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG6_CH1_IOMUX, ML_PWM_TIMG6_CH1_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG6_CH1_PORT, ML_PWM_TIMG6_CH1_PIN);
    } else if ((timer == TIMG7) && (channel == DL_TIMER_CC_0_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG7_CH0_IOMUX, ML_PWM_TIMG7_CH0_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG7_CH0_PORT, ML_PWM_TIMG7_CH0_PIN);
    } else if ((timer == TIMG7) && (channel == DL_TIMER_CC_1_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG7_CH1_IOMUX, ML_PWM_TIMG7_CH1_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG7_CH1_PORT, ML_PWM_TIMG7_CH1_PIN);
    } else if ((timer == TIMG8) && (channel == DL_TIMER_CC_0_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG8_CH0_IOMUX, ML_PWM_TIMG8_CH0_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG8_CH0_PORT, ML_PWM_TIMG8_CH0_PIN);
    } else if ((timer == TIMG8) && (channel == DL_TIMER_CC_1_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG8_CH1_IOMUX, ML_PWM_TIMG8_CH1_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG8_CH1_PORT, ML_PWM_TIMG8_CH1_PIN);
    } else if ((timer == TIMG12) && (channel == DL_TIMER_CC_0_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG12_CH0_IOMUX, ML_PWM_TIMG12_CH0_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG12_CH0_PORT, ML_PWM_TIMG12_CH0_PIN);
    } else if ((timer == TIMG12) && (channel == DL_TIMER_CC_1_INDEX)) {
        DL_GPIO_initPeripheralOutputFunction(
            ML_PWM_TIMG12_CH1_IOMUX, ML_PWM_TIMG12_CH1_FUNCTION);
        DL_GPIO_enableOutput(ML_PWM_TIMG12_CH1_PORT, ML_PWM_TIMG12_CH1_PIN);
    } else {
        return ML_STATUS_UNSUPPORTED;
    }
    return ML_STATUS_OK;
}

ml_status_t pwm_pin_init(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel,
    uint16_t frequency_hz, uint8_t *prescale, uint32_t *period)
{
    pwm_state_t *state = pwm_find_state(timer);
    uint32_t divider;
    uint32_t total_counts;
    ml_status_t status;

    if ((state == 0) || (frequency_hz == 0U) || (prescale == 0) ||
        (period == 0) || (pwm_channel_output_mask(channel) == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (state->initialized && (state->frequency_hz != frequency_hz)) {
        return ML_STATUS_BUSY;
    }

    total_counts = state->divided_clock_hz / frequency_hz;
    if (total_counts == 0U) {
        return ML_STATUS_UNSUPPORTED;
    }
    divider = (total_counts + 65535U) / 65536U;
    if ((divider == 0U) || (divider > 256U)) {
        return ML_STATUS_UNSUPPORTED;
    }

    *period = (total_counts / divider) - 1U;
    *prescale = (uint8_t) (divider - 1U);

    status = pwm_claim_pin_resource(timer, channel);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = pwm_configure_pin(timer, channel);
    return status;
}

ml_status_t pwm_init(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t frequency_hz)
{
    pwm_state_t *state = pwm_find_state(timer);
    uint8_t prescale;
    uint32_t period;
    uint32_t output_mask = pwm_channel_output_mask(channel);
    ml_status_t status;
    DL_TimerG_ClockConfig clock_config;
    DL_TimerG_PWMConfig pwm_config;

    status = pwm_pin_init(timer, channel, frequency_hz, &prescale, &period);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (state == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (state->initialized && (state->frequency_hz != frequency_hz)) {
        return ML_STATUS_BUSY;
    }

    if (!state->initialized) {
        clock_config.clockSel = DL_TIMER_CLOCK_BUSCLK;
        clock_config.divideRatio = DL_TIMER_CLOCK_DIVIDE_8;
        clock_config.prescale = prescale;

        pwm_config.pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP;
        pwm_config.period = period;
        pwm_config.startTimer = DL_TIMER_START;

        DL_TimerG_reset(timer);
        DL_TimerG_enablePower(timer);
        DL_TimerG_setClockConfig(timer, &clock_config);
        DL_TimerG_initPWMMode(timer, &pwm_config);
        state->frequency_hz = frequency_hz;
        state->initialized = true;
    }

    DL_TimerG_setCaptureCompareValue(timer, 0U, channel);
    DL_TimerG_setCaptureCompareOutCtl(timer, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        channel);
    DL_TimerG_setCaptCompUpdateMethod(
        timer, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, channel);

    state->channel_mask |= output_mask;
    DL_Timer_setCCPDirection(timer, state->channel_mask);
    DL_TimerG_enableClock(timer);

    return ML_STATUS_OK;
}

ml_status_t pwm_update(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint32_t duty)
{
    pwm_state_t *state = pwm_find_state(timer);
    uint32_t period;
    uint32_t compare;

    if ((state == 0) || !state->initialized ||
        ((state->channel_mask & pwm_channel_output_mask(channel)) == 0U)) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (duty > ML_PWM_DUTY_MAX) {
        duty = ML_PWM_DUTY_MAX;
    }

    period = timer->COUNTERREGS.LOAD + 1U;
    compare = (uint32_t) (((uint64_t) duty * period) / ML_PWM_DUTY_MAX);
    DL_TimerG_setCaptureCompareValue(timer, compare, channel);
    return ML_STATUS_OK;
}
