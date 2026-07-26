#include "line_sensor.h"

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_gpio.h"

static const float g_sensor_positions_mm[4] = {
    -40.25f, -7.25f, 7.25f, 40.25f
};

static uint8_t g_white_levels;
static float g_last_error_mm;
static bool g_initialized;

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

    status = gpio_init(ML_C1_PORT, ML_C1_PIN,
        (GPIOn_enum) ML_C1_IOMUX, IN_FLOAT);
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_C2_PORT, ML_C2_PIN,
            (GPIOn_enum) ML_C2_IOMUX, IN_FLOAT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_C3_PORT, ML_C3_PIN,
            (GPIOn_enum) ML_C3_IOMUX, IN_FLOAT);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_C8_PORT, ML_C8_PIN,
            (GPIOn_enum) ML_C8_IOMUX, IN_FLOAT);
    }
    if (status == ML_STATUS_OK) {
        g_white_levels = line_sensor_read_raw();
        g_last_error_mm = 0.0f;
        g_initialized = true;
    }
    return status;
}

ml_status_t line_sensor_calibrate_white(uint16_t samples, uint16_t delay_ms_each)
{
    uint16_t high_counts[4] = {0U, 0U, 0U, 0U};
    uint16_t sample;
    uint8_t index;
    uint8_t raw;

    if (!g_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    if (samples == 0U) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    for (sample = 0U; sample < samples; ++sample) {
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
    g_white_levels = 0U;
    for (index = 0U; index < 4U; ++index) {
        if (((uint32_t) high_counts[index] * 2U) >= samples) {
            g_white_levels |= (uint8_t) (1U << index);
        }
    }
    return ML_STATUS_OK;
}

line_sample_t line_sensor_read(void)
{
    line_sample_t result;
    uint8_t index;
    float sum = 0.0f;

    result.raw_bits = line_sensor_read_raw();
    result.black_bits = (uint8_t) ((result.raw_bits ^ g_white_levels) & 0x0FU);
    result.black_count = 0U;
    for (index = 0U; index < 4U; ++index) {
        if ((result.black_bits & (uint8_t) (1U << index)) != 0U) {
            sum += g_sensor_positions_mm[index];
            ++result.black_count;
        }
    }
    result.lost = result.black_count == 0U;
    if (!result.lost) {
        result.error_mm = sum / (float) result.black_count;
        g_last_error_mm = result.error_mm;
    } else {
        result.error_mm = g_last_error_mm;
    }
    result.transverse = result.black_count >= 3U;
    result.centered = !result.lost && !result.transverse &&
        (result.error_mm >= -10.0f) && (result.error_mm <= 10.0f) &&
        ((result.black_bits & 0x06U) != 0U);
    return result;
}

uint8_t line_sensor_white_levels(void)
{
    return g_white_levels;
}
