#include "ml_bmi088.h"

#include <stdbool.h>

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_gpio.h"

#define BMI088_ACCEL_CHIP_ID_REG     (0x00U)
#define BMI088_ACCEL_CHIP_ID_VALUE   (0x1EU)
#define BMI088_ACCEL_DATA_REG        (0x12U)
#define BMI088_ACCEL_CONF_REG        (0x40U)
#define BMI088_ACCEL_RANGE_REG       (0x41U)
#define BMI088_ACCEL_PWR_CONF_REG    (0x7CU)
#define BMI088_ACCEL_PWR_CTRL_REG    (0x7DU)
#define BMI088_ACCEL_SOFTRESET_REG   (0x7EU)

#define BMI088_GYRO_CHIP_ID_REG      (0x00U)
#define BMI088_GYRO_CHIP_ID_VALUE    (0x0FU)
#define BMI088_GYRO_DATA_REG         (0x02U)
#define BMI088_GYRO_RANGE_REG        (0x0FU)
#define BMI088_GYRO_BANDWIDTH_REG    (0x10U)
#define BMI088_GYRO_LPM1_REG         (0x11U)
#define BMI088_GYRO_SOFTRESET_REG    (0x14U)

#define BMI088_SOFTRESET_COMMAND     (0xB6U)
#define BMI088_ACCEL_CONF_100HZ      (0xA8U)
#define BMI088_ACCEL_RANGE_6G        (0x01U)
#define BMI088_ACCEL_POWER_ACTIVE    (0x00U)
#define BMI088_ACCEL_POWER_ENABLE    (0x04U)
#define BMI088_GYRO_RANGE_2000DPS    (0x00U)
#define BMI088_GYRO_BW_100HZ_32HZ    (0x07U)
#define BMI088_GYRO_POWER_NORMAL     (0x00U)

#define BMI088_ACCEL_SCALE_G         (6.0f / 32768.0f)
#define BMI088_GYRO_SCALE_DPS        (2000.0f / 32768.0f)
#define BMI088_SPI_HALF_PERIOD_US    (5U)
#define BMI088_SPI_CS_SETUP_US       (1U)
#define BMI088_GYRO_RECOVERY_TRIES   (3U)
#define BMI088_GYRO_MISO_PROBE_READS (3U)

static bool g_bmi088_initialized;
static bool g_bmi088_resource_claimed;
static bool g_bmi088_spi_pins_initialized;
static bmi088_diagnostic_t g_bmi088_diagnostic;

static void bmi088_clear_diagnostic(void)
{
    g_bmi088_diagnostic.stage = BMI088_DIAG_STAGE_NONE;
    g_bmi088_diagnostic.status = ML_STATUS_OK;
    g_bmi088_diagnostic.accel_chip_id = 0xFFU;
    g_bmi088_diagnostic.gyro_chip_id = 0xFFU;
    g_bmi088_diagnostic.gyro_id_before_reset = 0xFFU;
    g_bmi088_diagnostic.gyro_id_after_reset = 0xFFU;
    g_bmi088_diagnostic.gyro_id_miso_float = 0xFFU;
    g_bmi088_diagnostic.gyro_id_miso_pullup = 0xFFU;
    g_bmi088_diagnostic.gyro_id_miso_pulldown = 0xFFU;
    g_bmi088_diagnostic.gyro_miso_state = BMI088_GYRO_MISO_UNKNOWN;
    g_bmi088_diagnostic.register_address = 0U;
    g_bmi088_diagnostic.expected_value = 0U;
    g_bmi088_diagnostic.actual_value = 0U;
}

static void bmi088_set_failure(
    bmi088_diag_stage_t stage, ml_status_t status)
{
    g_bmi088_diagnostic.stage = stage;
    g_bmi088_diagnostic.status = status;
}

static uint8_t bmi088_spi_transfer(uint8_t transmit)
{
    uint8_t receive = 0U;
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        /* SPI mode 3: change on the falling edge, sample on rising. */
        gpio_set(ML_IMU_SPI_SCLK_PORT, ML_IMU_SPI_SCLK_PIN, 0U);
        gpio_set(ML_IMU_SPI_MOSI_PORT, ML_IMU_SPI_MOSI_PIN,
            (transmit & mask) != 0U);
        delay_us(BMI088_SPI_HALF_PERIOD_US);
        gpio_set(ML_IMU_SPI_SCLK_PORT, ML_IMU_SPI_SCLK_PIN, 1U);
        if (gpio_get(ML_IMU_SPI_MISO_PORT, ML_IMU_SPI_MISO_PIN) != 0U) {
            receive |= mask;
        }
        delay_us(BMI088_SPI_HALF_PERIOD_US);
    }
    return receive;
}

static void bmi088_accel_select(bool selected)
{
    gpio_set(ML_IMU_CS_A_PORT, ML_IMU_CS_A_PIN, selected ? 0U : 1U);
    if (selected) {
        delay_us(BMI088_SPI_CS_SETUP_US);
    }
}

static void bmi088_gyro_select(bool selected)
{
    gpio_set(ML_IMU_CS_G_PORT, ML_IMU_CS_G_PIN, selected ? 0U : 1U);
    if (selected) {
        delay_us(BMI088_SPI_CS_SETUP_US);
    }
}

static void bmi088_accel_write(uint8_t reg, uint8_t value)
{
    bmi088_accel_select(true);
    (void) bmi088_spi_transfer(reg & 0x7FU);
    (void) bmi088_spi_transfer(value);
    bmi088_accel_select(false);
}

static void bmi088_gyro_write(uint8_t reg, uint8_t value)
{
    bmi088_gyro_select(true);
    (void) bmi088_spi_transfer(reg & 0x7FU);
    (void) bmi088_spi_transfer(value);
    bmi088_gyro_select(false);
}

static void bmi088_accel_read(
    uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;

    bmi088_accel_select(true);
    (void) bmi088_spi_transfer(reg | 0x80U);
    /* The accelerometer inserts one dummy byte on every SPI read. */
    (void) bmi088_spi_transfer(0x00U);
    for (index = 0U; index < length; ++index) {
        data[index] = bmi088_spi_transfer(0x00U);
    }
    bmi088_accel_select(false);
}

static void bmi088_gyro_read(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;

    bmi088_gyro_select(true);
    (void) bmi088_spi_transfer(reg | 0x80U);
    for (index = 0U; index < length; ++index) {
        data[index] = bmi088_spi_transfer(0x00U);
    }
    bmi088_gyro_select(false);
}

static bool bmi088_probe_gyro_id_with_miso_mode(
    GPIO_Mode_enum mode, uint8_t *chip_id)
{
    uint8_t attempt;
    uint8_t actual = 0xFFU;
    uint8_t first = 0xFFU;
    bool stable = true;

    if (gpio_init(ML_IMU_SPI_MISO_PORT, ML_IMU_SPI_MISO_PIN,
        (GPIOn_enum) ML_IMU_SPI_MISO_IOMUX, mode) != ML_STATUS_OK) {
        *chip_id = 0xFFU;
        return false;
    }
    for (attempt = 0U; attempt < BMI088_GYRO_MISO_PROBE_READS;
        ++attempt) {
        bmi088_gyro_read(BMI088_GYRO_CHIP_ID_REG, &actual, 1U);
        if (attempt == 0U) {
            first = actual;
        } else if (actual != first) {
            stable = false;
        }
        delay_ms(10U);
    }
    *chip_id = actual;
    return stable;
}

static ml_status_t bmi088_probe_gyro_miso(void)
{
    bool float_stable;
    bool pullup_stable;
    bool pulldown_stable;
    GPIO_Mode_enum valid_mode = IN_FLOAT;

    float_stable = bmi088_probe_gyro_id_with_miso_mode(IN_FLOAT,
        &g_bmi088_diagnostic.gyro_id_miso_float);
    pullup_stable = bmi088_probe_gyro_id_with_miso_mode(IN_UP,
        &g_bmi088_diagnostic.gyro_id_miso_pullup);
    pulldown_stable = bmi088_probe_gyro_id_with_miso_mode(IN_DOWN,
        &g_bmi088_diagnostic.gyro_id_miso_pulldown);

    if (float_stable &&
        (g_bmi088_diagnostic.gyro_id_miso_float ==
         BMI088_GYRO_CHIP_ID_VALUE)) {
        valid_mode = IN_FLOAT;
    } else if (pullup_stable &&
        (g_bmi088_diagnostic.gyro_id_miso_pullup ==
         BMI088_GYRO_CHIP_ID_VALUE)) {
        valid_mode = IN_UP;
    } else if (pulldown_stable &&
        (g_bmi088_diagnostic.gyro_id_miso_pulldown ==
         BMI088_GYRO_CHIP_ID_VALUE)) {
        valid_mode = IN_DOWN;
    } else if (!float_stable || !pullup_stable || !pulldown_stable) {
        g_bmi088_diagnostic.gyro_miso_state =
            BMI088_GYRO_MISO_UNSTABLE;
    } else if ((g_bmi088_diagnostic.gyro_id_miso_pullup == 0xFFU) &&
        (g_bmi088_diagnostic.gyro_id_miso_pulldown == 0x00U)) {
        g_bmi088_diagnostic.gyro_miso_state = BMI088_GYRO_MISO_HIGH_Z;
    } else if ((g_bmi088_diagnostic.gyro_id_miso_pullup == 0x00U) &&
        (g_bmi088_diagnostic.gyro_id_miso_pulldown == 0x00U)) {
        g_bmi088_diagnostic.gyro_miso_state =
            BMI088_GYRO_MISO_STUCK_LOW;
    } else if ((g_bmi088_diagnostic.gyro_id_miso_pullup == 0xFFU) &&
        (g_bmi088_diagnostic.gyro_id_miso_pulldown == 0xFFU)) {
        g_bmi088_diagnostic.gyro_miso_state =
            BMI088_GYRO_MISO_STUCK_HIGH;
    } else {
        g_bmi088_diagnostic.gyro_miso_state =
            BMI088_GYRO_MISO_UNSTABLE;
    }

    if ((float_stable &&
         (g_bmi088_diagnostic.gyro_id_miso_float ==
          BMI088_GYRO_CHIP_ID_VALUE)) ||
        (pullup_stable &&
         (g_bmi088_diagnostic.gyro_id_miso_pullup ==
          BMI088_GYRO_CHIP_ID_VALUE)) ||
        (pulldown_stable &&
         (g_bmi088_diagnostic.gyro_id_miso_pulldown ==
          BMI088_GYRO_CHIP_ID_VALUE))) {
        (void) gpio_init(ML_IMU_SPI_MISO_PORT, ML_IMU_SPI_MISO_PIN,
            (GPIOn_enum) ML_IMU_SPI_MISO_IOMUX, valid_mode);
        g_bmi088_diagnostic.gyro_chip_id = BMI088_GYRO_CHIP_ID_VALUE;
        g_bmi088_diagnostic.gyro_id_after_reset =
            BMI088_GYRO_CHIP_ID_VALUE;
        g_bmi088_diagnostic.gyro_miso_state = BMI088_GYRO_MISO_VALID;
        return ML_STATUS_OK;
    }

    (void) gpio_init(ML_IMU_SPI_MISO_PORT, ML_IMU_SPI_MISO_PIN,
        (GPIOn_enum) ML_IMU_SPI_MISO_IOMUX, IN_FLOAT);
    return ML_STATUS_DEVICE_NOT_FOUND;
}

static ml_status_t bmi088_check_identity(bmi088_diag_stage_t stage)
{
    bmi088_accel_read(BMI088_ACCEL_CHIP_ID_REG,
        &g_bmi088_diagnostic.accel_chip_id, 1U);
    bmi088_gyro_read(BMI088_GYRO_CHIP_ID_REG,
        &g_bmi088_diagnostic.gyro_chip_id, 1U);
    if ((g_bmi088_diagnostic.accel_chip_id !=
         BMI088_ACCEL_CHIP_ID_VALUE) ||
        (g_bmi088_diagnostic.gyro_chip_id !=
         BMI088_GYRO_CHIP_ID_VALUE)) {
        bmi088_set_failure(stage, ML_STATUS_DEVICE_NOT_FOUND);
        return ML_STATUS_DEVICE_NOT_FOUND;
    }
    return ML_STATUS_OK;
}

static ml_status_t bmi088_probe_initial_identity(void)
{
    uint8_t attempt;

    bmi088_accel_read(BMI088_ACCEL_CHIP_ID_REG,
        &g_bmi088_diagnostic.accel_chip_id, 1U);
    bmi088_gyro_read(BMI088_GYRO_CHIP_ID_REG,
        &g_bmi088_diagnostic.gyro_chip_id, 1U);
    if (g_bmi088_diagnostic.accel_chip_id !=
        BMI088_ACCEL_CHIP_ID_VALUE) {
        bmi088_set_failure(
            BMI088_DIAG_STAGE_ID_INITIAL, ML_STATUS_DEVICE_NOT_FOUND);
        return ML_STATUS_DEVICE_NOT_FOUND;
    }
    if (g_bmi088_diagnostic.gyro_chip_id == BMI088_GYRO_CHIP_ID_VALUE) {
        return ML_STATUS_OK;
    }

    g_bmi088_diagnostic.gyro_id_before_reset =
        g_bmi088_diagnostic.gyro_chip_id;
    bmi088_gyro_write(
        BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_COMMAND);
    delay_ms(50U);
    for (attempt = 0U; attempt < BMI088_GYRO_RECOVERY_TRIES; ++attempt) {
        bmi088_gyro_read(BMI088_GYRO_CHIP_ID_REG,
            &g_bmi088_diagnostic.gyro_chip_id, 1U);
        g_bmi088_diagnostic.gyro_id_after_reset =
            g_bmi088_diagnostic.gyro_chip_id;
        if (g_bmi088_diagnostic.gyro_chip_id ==
            BMI088_GYRO_CHIP_ID_VALUE) {
            return ML_STATUS_OK;
        }
        delay_ms(10U);
    }

    if (bmi088_probe_gyro_miso() == ML_STATUS_OK) {
        return ML_STATUS_OK;
    }

    bmi088_set_failure(
        BMI088_DIAG_STAGE_GYRO_RECOVERY, ML_STATUS_DEVICE_NOT_FOUND);
    return ML_STATUS_DEVICE_NOT_FOUND;
}

static ml_status_t bmi088_init_gpio(GPIO_Regs *port, uint32_t pin,
    GPIOn_enum iomux, GPIO_Mode_enum mode, bmi088_diag_stage_t stage)
{
    ml_status_t status = gpio_init(port, pin, iomux, mode);

    if (status != ML_STATUS_OK) {
        bmi088_set_failure(stage, status);
    }
    return status;
}

static ml_status_t bmi088_verify_accel_register(
    uint8_t reg, uint8_t expected)
{
    uint8_t actual;

    bmi088_accel_read(reg, &actual, 1U);
    if (actual != expected) {
        bmi088_set_failure(
            BMI088_DIAG_STAGE_ACCEL_CONFIG, ML_STATUS_DEVICE_NOT_FOUND);
        g_bmi088_diagnostic.register_address = reg;
        g_bmi088_diagnostic.expected_value = expected;
        g_bmi088_diagnostic.actual_value = actual;
        return ML_STATUS_DEVICE_NOT_FOUND;
    }
    return ML_STATUS_OK;
}

static ml_status_t bmi088_verify_gyro_register(
    uint8_t reg, uint8_t expected)
{
    uint8_t actual;

    bmi088_gyro_read(reg, &actual, 1U);
    if (actual != expected) {
        bmi088_set_failure(
            BMI088_DIAG_STAGE_GYRO_CONFIG, ML_STATUS_DEVICE_NOT_FOUND);
        g_bmi088_diagnostic.register_address = reg;
        g_bmi088_diagnostic.expected_value = expected;
        g_bmi088_diagnostic.actual_value = actual;
        return ML_STATUS_DEVICE_NOT_FOUND;
    }
    return ML_STATUS_OK;
}

static ml_status_t bmi088_verify_configuration(void)
{
    ml_status_t status = bmi088_verify_accel_register(
        BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_POWER_ENABLE);

    if (status == ML_STATUS_OK) {
        status = bmi088_verify_accel_register(
            BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_POWER_ACTIVE);
    }
    if (status == ML_STATUS_OK) {
        status = bmi088_verify_accel_register(
            BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_100HZ);
    }
    if (status == ML_STATUS_OK) {
        status = bmi088_verify_accel_register(
            BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G);
    }
    if (status == ML_STATUS_OK) {
        status = bmi088_verify_gyro_register(
            BMI088_GYRO_LPM1_REG, BMI088_GYRO_POWER_NORMAL);
    }
    if (status == ML_STATUS_OK) {
        status = bmi088_verify_gyro_register(
            BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
    }
    if (status == ML_STATUS_OK) {
        status = bmi088_verify_gyro_register(
            BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_100HZ_32HZ);
    }
    return status;
}

static ml_status_t bmi088_spi_init(void)
{
    ml_status_t status;

    status = board_resource_claim(
        ML_BOARD_RESOURCE_PB22, ML_BOARD_OWNER_IMU_SPI);
    if (status != ML_STATUS_OK) {
        bmi088_set_failure(BMI088_DIAG_STAGE_RESOURCE, status);
        return status;
    }
    g_bmi088_resource_claimed = true;

    status = bmi088_init_gpio(ML_IMU_CS_A_PORT, ML_IMU_CS_A_PIN,
        (GPIOn_enum) ML_IMU_CS_A_IOMUX, OUT,
        BMI088_DIAG_STAGE_GPIO_CS_A);
    if (status == ML_STATUS_OK) {
        gpio_set(ML_IMU_CS_A_PORT, ML_IMU_CS_A_PIN, 1U);
        status = bmi088_init_gpio(ML_IMU_CS_G_PORT, ML_IMU_CS_G_PIN,
            (GPIOn_enum) ML_IMU_CS_G_IOMUX, OUT,
            BMI088_DIAG_STAGE_GPIO_CS_G);
    }
    if (status == ML_STATUS_OK) {
        gpio_set(ML_IMU_CS_G_PORT, ML_IMU_CS_G_PIN, 1U);
        status = bmi088_init_gpio(
            ML_IMU_SPI_SCLK_PORT, ML_IMU_SPI_SCLK_PIN,
            (GPIOn_enum) ML_IMU_SPI_SCLK_IOMUX, OUT,
            BMI088_DIAG_STAGE_GPIO_SCLK);
    }
    if (status == ML_STATUS_OK) {
        gpio_set(ML_IMU_SPI_SCLK_PORT, ML_IMU_SPI_SCLK_PIN, 1U);
        status = bmi088_init_gpio(
            ML_IMU_SPI_MOSI_PORT, ML_IMU_SPI_MOSI_PIN,
            (GPIOn_enum) ML_IMU_SPI_MOSI_IOMUX, OUT,
            BMI088_DIAG_STAGE_GPIO_MOSI);
    }
    if (status == ML_STATUS_OK) {
        gpio_set(ML_IMU_SPI_MOSI_PORT, ML_IMU_SPI_MOSI_PIN, 1U);
        status = bmi088_init_gpio(
            ML_IMU_SPI_MISO_PORT, ML_IMU_SPI_MISO_PIN,
            (GPIOn_enum) ML_IMU_SPI_MISO_IOMUX, IN_FLOAT,
            BMI088_DIAG_STAGE_GPIO_MISO);
    }
    if (status == ML_STATUS_OK) {
        g_bmi088_spi_pins_initialized = true;
    }
    return status;
}

static int16_t bmi088_decode_word(const uint8_t *bytes)
{
    return (int16_t) (((uint16_t) bytes[1] << 8) | bytes[0]);
}

ml_status_t bmi088_init(void)
{
    ml_status_t status;

    if (g_bmi088_initialized) {
        return ML_STATUS_OK;
    }

    bmi088_clear_diagnostic();
    status = bmi088_spi_init();
    if (status == ML_STATUS_OK) {
        delay_ms(10U);
        status = bmi088_probe_initial_identity();
    }
    if (status == ML_STATUS_OK) {
        bmi088_accel_write(
            BMI088_ACCEL_SOFTRESET_REG, BMI088_SOFTRESET_COMMAND);
        bmi088_gyro_write(
            BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_COMMAND);
        delay_ms(30U);
        status = bmi088_check_identity(BMI088_DIAG_STAGE_ID_POST_RESET);
    }

    if (status == ML_STATUS_OK) {
        bmi088_accel_write(
            BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_POWER_ENABLE);
        delay_ms(5U);
        bmi088_accel_write(
            BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_POWER_ACTIVE);
        delay_ms(5U);
        bmi088_accel_write(
            BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_100HZ);
        bmi088_accel_write(
            BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G);

        bmi088_gyro_write(
            BMI088_GYRO_LPM1_REG, BMI088_GYRO_POWER_NORMAL);
        delay_ms(30U);
        bmi088_gyro_write(
            BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
        bmi088_gyro_write(
            BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_100HZ_32HZ);
        delay_ms(10U);
        status = bmi088_verify_configuration();
    }

    if (status == ML_STATUS_OK) {
        g_bmi088_initialized = true;
        bmi088_clear_diagnostic();
    } else if (g_bmi088_resource_claimed) {
        board_resource_release(
            ML_BOARD_RESOURCE_PB22, ML_BOARD_OWNER_IMU_SPI);
        g_bmi088_resource_claimed = false;
    }
    return status;
}

ml_status_t bmi088_read(bmi088_data_t *data)
{
    uint8_t accel_raw[6];
    uint8_t gyro_raw[6];
    ml_status_t status;

    if (data == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_bmi088_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    bmi088_clear_diagnostic();
    status = bmi088_check_identity(BMI088_DIAG_STAGE_ID_RUNTIME);
    if (status != ML_STATUS_OK) {
        return status;
    }
    bmi088_accel_read(BMI088_ACCEL_DATA_REG, accel_raw, sizeof(accel_raw));
    bmi088_gyro_read(BMI088_GYRO_DATA_REG, gyro_raw, sizeof(gyro_raw));

    data->accel_x_raw = bmi088_decode_word(&accel_raw[0]);
    data->accel_y_raw = bmi088_decode_word(&accel_raw[2]);
    data->accel_z_raw = bmi088_decode_word(&accel_raw[4]);
    data->gyro_x_raw = bmi088_decode_word(&gyro_raw[0]);
    data->gyro_y_raw = bmi088_decode_word(&gyro_raw[2]);
    data->gyro_z_raw = bmi088_decode_word(&gyro_raw[4]);

    data->accel_x_g = (float) data->accel_x_raw * BMI088_ACCEL_SCALE_G;
    data->accel_y_g = (float) data->accel_y_raw * BMI088_ACCEL_SCALE_G;
    data->accel_z_g = (float) data->accel_z_raw * BMI088_ACCEL_SCALE_G;
    data->gyro_x_dps = (float) data->gyro_x_raw * BMI088_GYRO_SCALE_DPS;
    data->gyro_y_dps = (float) data->gyro_y_raw * BMI088_GYRO_SCALE_DPS;
    data->gyro_z_dps = (float) data->gyro_z_raw * BMI088_GYRO_SCALE_DPS;
    return ML_STATUS_OK;
}

ml_status_t bmi088_get_diagnostic(bmi088_diagnostic_t *diagnostic)
{
    if (diagnostic == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *diagnostic = g_bmi088_diagnostic;
    return ML_STATUS_OK;
}

ml_status_t bmi088_diagnostic_set_cs_g(bool high)
{
    if (!g_bmi088_spi_pins_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }
    gpio_set(ML_IMU_CS_G_PORT, ML_IMU_CS_G_PIN, high ? 1U : 0U);
    return ML_STATUS_OK;
}
