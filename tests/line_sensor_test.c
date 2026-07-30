#include "line_sensor.h"

#include <stdio.h>

#include "ml_board.h"
#include "ml_gpio.h"

GPIO_Regs g_test_gpioa;
GPIO_Regs g_test_gpiob;

static int g_failures;
static uint8_t g_raw_bits;
static uint8_t g_input_up_count;
static uint32_t g_gpio_init_calls;
static int32_t g_gpio_fail_call = -1;
static bool g_saw_pa27_input;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

ml_status_t gpio_init(
    GPIO_Regs *gpio, uint32_t pins, GPIOn_enum gpion,
    GPIO_Mode_enum mode)
{
    ++g_gpio_init_calls;
    if ((g_gpio_fail_call >= 0) &&
        (g_gpio_init_calls == (uint32_t) g_gpio_fail_call)) {
        return ML_STATUS_TIMEOUT;
    }
    if (mode == IN_UP) {
        ++g_input_up_count;
    }
    if ((gpio == GPIOA) && (pins == DL_GPIO_PIN_27) &&
        (gpion == PA27) && (mode == IN_UP)) {
        g_saw_pa27_input = true;
    }
    return ML_STATUS_OK;
}

uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins)
{
    if ((gpio == GPIOA) && (pins == ML_C1_PIN)) {
        return (g_raw_bits & 0x01U) != 0U;
    }
    if ((gpio == GPIOA) && (pins == ML_C2_PIN)) {
        return (g_raw_bits & 0x02U) != 0U;
    }
    if ((gpio == GPIOB) && (pins == ML_C3_PIN)) {
        return (g_raw_bits & 0x04U) != 0U;
    }
    if ((gpio == ML_C8_PORT) && (pins == ML_C8_PIN)) {
        return (g_raw_bits & 0x08U) != 0U;
    }
    return 0U;
}

void delay_ms(uint32_t delay)
{
    (void) delay;
}

void delay_us(uint32_t delay)
{
    (void) delay;
}

void SysTick_Init(void)
{
}

void SysTick_Wait(uint32_t delay)
{
    (void) delay;
}

static void test_white_high_black_low(void)
{
    line_sample_t line;

    g_raw_bits = 0x0FU;
    g_input_up_count = 0U;
    g_gpio_init_calls = 0U;
    g_gpio_fail_call = -1;
    g_saw_pa27_input = false;
    check(line_sensor_init() == ML_STATUS_OK &&
        g_input_up_count == 4U && g_saw_pa27_input,
        "all LF04 inputs use pull-ups and right outer is PA27");
    check(line_sensor_calibrate_white(10U, 0U) == ML_STATUS_OK &&
        line_sensor_white_levels() == 0x0FU,
        "high-level white background validates fixed polarity");

    g_raw_bits = 0x0EU;
    line = line_sensor_read();
    check(line.black_bits == 0x01U,
        "low-going PA31 maps to left-outer B1");
    g_raw_bits = 0x0DU;
    line = line_sensor_read();
    check(line.black_bits == 0x02U,
        "low-going PA12 maps to left-inner B2");
    g_raw_bits = 0x0BU;
    line = line_sensor_read();
    check(line.black_bits == 0x04U,
        "low-going PB8 maps to right-inner B4");
    g_raw_bits = 0x07U;
    line = line_sensor_read();
    check(line.black_bits == 0x08U,
        "low-going PA27 maps to right-outer B8");

    g_raw_bits = 0x09U;
    {
        uint8_t before = g_input_up_count;

        line = line_sensor_read();
        check(g_input_up_count == (uint8_t) (before + 4U),
            "every LF04 sample reasserts all four pull-up inputs");
    }
    check(line.black_bits == 0x06U && !line.lost && !line.io_fault,
        "low-going A12 and PB8 remain distinct active inputs");
}

static void test_input_reassert_failure(void)
{
    line_sample_t line;
    uint32_t calls_before;

    g_raw_bits = 0x0FU;
    g_gpio_fail_call = -1;
    check(line_sensor_init() == ML_STATUS_OK,
        "LF04 initializes before runtime IO-fault injection");
    calls_before = g_gpio_init_calls;
    g_gpio_fail_call = (int32_t) g_gpio_init_calls + 3;
    line = line_sensor_read();
    check(line.io_fault && line.lost && line.black_bits == 0U &&
        g_gpio_init_calls == calls_before + 4U,
        "a failed input reassertion still visits four inputs and cannot become B15");
    g_gpio_fail_call = -1;
    line = line_sensor_read();
    check(!line.io_fault && line.raw_bits == 0x0FU,
        "LF04 sampling recovers after GPIO input configuration succeeds");
}

static void test_fixed_white_rejects_bad_baselines(void)
{
    line_sample_t line;

    g_raw_bits = 0x07U;
    check(line_sensor_init() == ML_STATUS_OK &&
        line_sensor_calibrate_white(10U, 0U) == ML_STATUS_TIMEOUT &&
        line_sensor_white_levels() == LINE_SENSOR_WHITE_LEVELS_EXPECTED,
        "startup R7 cannot replace the fixed WF baseline");
    line = line_sensor_read();
    check(line.black_bits == 0x08U,
        "transient R7 is reported as right-outer black, not stored as white");

    g_raw_bits = 0x0FU;
    line = line_sensor_read();
    check(line.raw_bits == 0x0FU && line.black_bits == 0U,
        "recovered RF always normalizes to B0 with fixed polarity");

    g_raw_bits = 0x00U;
    check(line_sensor_calibrate_white(10U, 0U) == ML_STATUS_TIMEOUT &&
        line_sensor_white_levels() == LINE_SENSOR_WHITE_LEVELS_EXPECTED,
        "white-low hardware polarity is explicitly rejected");
    line = line_sensor_read();
    check(line.black_bits == 0x0FU,
        "all-low input remains B15 after rejected validation");
}

static void test_white_stability_gate(void)
{
    line_sensor_white_guard_t guard;
    bool ready = false;
    uint8_t sample;

    line_sensor_white_guard_reset(&guard);
    for (sample = 0U;
         sample < (LINE_SENSOR_WHITE_STABLE_SAMPLES - 1U);
         ++sample) {
        ready = line_sensor_white_guard_update(&guard, 0x0FU);
    }
    check(!ready &&
        guard.consecutive_samples == LINE_SENSOR_WHITE_STABLE_SAMPLES - 1U,
        "nine stable RF samples do not release the white gate");
    ready = line_sensor_white_guard_update(&guard, 0x07U);
    check(!ready && guard.consecutive_samples == 0U,
        "any non-RF sample resets the white gate");
    for (sample = 0U; sample < LINE_SENSOR_WHITE_STABLE_SAMPLES; ++sample) {
        ready = line_sensor_white_guard_update(&guard, 0x0FU);
        if (sample + 1U < LINE_SENSOR_WHITE_STABLE_SAMPLES) {
            check(!ready, "white gate remains closed before sample ten");
        }
    }
    check(ready &&
        guard.consecutive_samples == LINE_SENSOR_WHITE_STABLE_SAMPLES,
        "the tenth consecutive RF sample releases the white gate");
    check(!line_sensor_white_guard_update(0, 0x0FU),
        "a null white gate cannot report ready");
}

static void test_all_black_patterns(void)
{
    line_sample_t line;
    uint8_t bits;

    g_raw_bits = 0x0FU;
    check(line_sensor_init() == ML_STATUS_OK &&
        line_sensor_calibrate_white(4U, 0U) == ML_STATUS_OK,
        "all-pattern test validates fixed active-low LF04 polarity");
    for (bits = 1U; bits <= 0x0FU; ++bits) {
        g_raw_bits = (uint8_t) ((~bits) & 0x0FU);
        line = line_sensor_read();
        check(line.black_bits == bits && !line.lost,
            "B1 through B15 are all valid line samples");
    }
    g_raw_bits = 0x0FU;
    line = line_sensor_read();
    check(line.black_bits == 0U && line.lost,
        "B0 is the only lost-line pattern");
}

int main(void)
{
    test_white_high_black_low();
    test_fixed_white_rejects_bad_baselines();
    test_white_stability_gate();
    test_all_black_patterns();
    test_input_reassert_failure();
    if (g_failures == 0) {
        puts("line sensor tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
