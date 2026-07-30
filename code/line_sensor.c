#include "line_sensor.h"

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_gpio.h"

static bool g_initialized;

static ml_status_t line_sensor_configure_inputs(void)
{
    ml_status_t status = ML_STATUS_OK;
    ml_status_t current_status;

    current_status = gpio_init(ML_C1_PORT, ML_C1_PIN,
        (GPIOn_enum) ML_C1_IOMUX, IN_UP);
    if (current_status != ML_STATUS_OK) {
        status = current_status;
    }
    current_status = gpio_init(ML_C2_PORT, ML_C2_PIN,
        (GPIOn_enum) ML_C2_IOMUX, IN_UP);
    if ((status == ML_STATUS_OK) &&
        (current_status != ML_STATUS_OK)) {
        status = current_status;
    }
    current_status = gpio_init(ML_C3_PORT, ML_C3_PIN,
        (GPIOn_enum) ML_C3_IOMUX, IN_UP);
    if ((status == ML_STATUS_OK) &&
        (current_status != ML_STATUS_OK)) {
        status = current_status;
    }
    current_status = gpio_init(ML_C8_PORT, ML_C8_PIN,
        (GPIOn_enum) ML_C8_IOMUX, IN_UP);
    if ((status == ML_STATUS_OK) &&
        (current_status != ML_STATUS_OK)) {
        status = current_status;
    }
    return status;
}

static uint8_t line_sensor_read_raw(void)
{
    uint8_t bits = 0U;

    bits |= gpio_get(ML_C1_PORT, ML_C1_PIN) ? 0x01U : 0U;
    bits |= gpio_get(ML_C2_PORT, ML_C2_PIN) ? 0x02U : 0U;
    bits |= gpio_get(ML_C3_PORT, ML_C3_PIN) ? 0x04U : 0U;
    bits |= gpio_get(ML_C8_PORT, ML_C8_PIN) ? 0x08U : 0U;
    return bits;
}

ml_status_t line_sensor_init(void)
{
    ml_status_t status;

    g_initialized = false;
    status = line_sensor_configure_inputs();
    if (status == ML_STATUS_OK) {
        g_initialized = true;
    }
    return status;
}

ml_status_t line_sensor_reassert_inputs(void)
{
    if (!g_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    return line_sensor_configure_inputs();
}

ml_status_t line_sensor_calibrate_white(uint16_t samples, uint16_t delay_ms_each)
{
    uint16_t high_counts[4] = {0U, 0U, 0U, 0U};
    uint16_t sample;
    uint8_t index;
    uint8_t raw;
    uint8_t verified_levels = 0U;
    ml_status_t status;

    if (!g_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (samples == 0U) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    for (sample = 0U; sample < samples; ++sample) {
        status = line_sensor_reassert_inputs();
        if (status != ML_STATUS_OK) {
            return status;
        }
        raw = line_sensor_read_raw();
        for (index = 0U; index < 4U; ++index) {
            if ((raw & (uint8_t) (1U << index)) != 0U) {
                ++high_counts[index];
            }
        }
        if (delay_ms_each != 0U) {
            delay_ms(delay_ms_each);
        }
    }
    for (index = 0U; index < 4U; ++index) {
        if (((uint32_t) high_counts[index] * 2U) >= samples) {
            verified_levels |= (uint8_t) (1U << index);
        }
    }
    return verified_levels == LINE_SENSOR_WHITE_LEVELS_EXPECTED ?
        ML_STATUS_OK : ML_STATUS_TIMEOUT;
}

line_sample_t line_sensor_read(void)
{
    line_sample_t result;
    if (line_sensor_reassert_inputs() != ML_STATUS_OK) {
        result.raw_bits = 0U;
        result.black_bits = 0U;
        result.left_on = false;
        result.right_on = false;
        result.lost = true;
        result.io_fault = true;
        return result;
    }

    result.raw_bits = line_sensor_read_raw();
    result.black_bits = (uint8_t) ((~result.raw_bits) &
        LINE_SENSOR_WHITE_LEVELS_EXPECTED);
    result.left_on =
        (result.black_bits & LINE_SENSOR_LEFT_GROUP_MASK) != 0U;
    result.right_on =
        (result.black_bits & LINE_SENSOR_RIGHT_GROUP_MASK) != 0U;
    result.lost = !result.left_on && !result.right_on;
    result.io_fault = false;
    return result;
}

uint8_t line_sensor_white_levels(void)
{
    return LINE_SENSOR_WHITE_LEVELS_EXPECTED;
}

void line_sensor_white_guard_reset(line_sensor_white_guard_t *guard)
{
    if (guard != 0) {
        guard->consecutive_samples = 0U;
    }
}

bool line_sensor_white_guard_update(
    line_sensor_white_guard_t *guard, uint8_t raw_bits)
{
    if (guard == 0) {
        return false;
    }
    if ((raw_bits & LINE_SENSOR_WHITE_LEVELS_EXPECTED) !=
        LINE_SENSOR_WHITE_LEVELS_EXPECTED) {
        guard->consecutive_samples = 0U;
        return false;
    }
    if (guard->consecutive_samples < LINE_SENSOR_WHITE_STABLE_SAMPLES) {
        ++guard->consecutive_samples;
    }
    return guard->consecutive_samples >= LINE_SENSOR_WHITE_STABLE_SAMPLES;
}
