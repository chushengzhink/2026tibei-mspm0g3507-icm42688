#include "icm42688.h"

#include "ml_board.h"
#include "ml_delay.h"
#include "ml_i2c.h"

#define ICM42688_REG_DEVICE_CONFIG (0x11U)
#define ICM42688_REG_ACCEL_DATA_X1 (0x1FU)
#define ICM42688_REG_PWR_MGMT0 (0x4EU)
#define ICM42688_REG_GYRO_CONFIG0 (0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0 (0x50U)
#define ICM42688_REG_WHO_AM_I (0x75U)
#define ICM42688_REG_BANK_SEL (0x76U)

#define ICM42688_SOFT_RESET (0x01U)
#define ICM42688_BANK_0 (0x00U)
#define ICM42688_ACCEL_4G_100HZ (0x48U)
#define ICM42688_GYRO_1000DPS_100HZ (0x28U)
#define ICM42688_ACCEL_GYRO_LOW_NOISE (0x0FU)

#define ICM42688_ACCEL_SCALE_G (4.0f / 32768.0f)
#define ICM42688_GYRO_SCALE_DPS (1000.0f / 32768.0f)

static ml_soft_i2c_bus_t g_icm42688_i2c_bus = {
    ML_ICM42688_I2C_PORT,
    ML_ICM42688_I2C_SCL_PIN,
    ML_ICM42688_I2C_SCL_IOMUX,
    ML_ICM42688_I2C_SDA_PIN,
    ML_ICM42688_I2C_SDA_IOMUX,
    ML_SOFT_I2C_HALF_PERIOD_US,
    ML_SOFT_I2C_TIMEOUT_US,
    false
};

static bool g_icm42688_initialized;

static ml_status_t icm42688_write_register(uint8_t reg, uint8_t value)
{
    uint8_t payload[2];

    payload[0] = reg;
    payload[1] = value;
    return soft_i2c_write(&g_icm42688_i2c_bus,
        ICM42688_I2C_ADDRESS, payload, 2U);
}

static ml_status_t icm42688_read_registers(
    uint8_t reg, uint8_t *data, uint32_t length)
{
    if ((data == 0) || (length == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    return soft_i2c_write_read(&g_icm42688_i2c_bus,
        ICM42688_I2C_ADDRESS, &reg, 1U, data, length);
}

static int16_t icm42688_decode_int16(const uint8_t *bytes)
{
    uint16_t value = ((uint16_t) bytes[0] << 8) | bytes[1];

    return (int16_t) value;
}

ml_status_t icm42688_init(void)
{
    uint8_t who_am_i = 0U;
    ml_status_t status;

    g_icm42688_initialized = false;
    status = soft_i2c_init(&g_icm42688_i2c_bus);
    if (status != ML_STATUS_OK) {
        return status;
    }

    status = icm42688_write_register(
        ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET);
    if (status != ML_STATUS_OK) {
        return status;
    }
    delay_ms(2U);

    status = icm42688_write_register(ICM42688_REG_BANK_SEL, ICM42688_BANK_0);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = icm42688_read_registers(ICM42688_REG_WHO_AM_I, &who_am_i, 1U);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        return ML_STATUS_DEVICE_NOT_FOUND;
    }

    status = icm42688_write_register(
        ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_4G_100HZ);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = icm42688_write_register(
        ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_1000DPS_100HZ);
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = icm42688_write_register(
        ICM42688_REG_PWR_MGMT0, ICM42688_ACCEL_GYRO_LOW_NOISE);
    if (status != ML_STATUS_OK) {
        return status;
    }
    delay_ms(50U);

    g_icm42688_initialized = true;
    return ML_STATUS_OK;
}

ml_status_t icm42688_read(icm42688_data_t *data)
{
    uint8_t buffer[12];
    icm42688_data_t sample;
    ml_status_t status;

    if (data == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    if (!g_icm42688_initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    status = icm42688_read_registers(
        ICM42688_REG_ACCEL_DATA_X1, buffer, sizeof(buffer));
    if (status != ML_STATUS_OK) {
        return status;
    }

    sample.accel_x_g =
        (float) icm42688_decode_int16(&buffer[0]) * ICM42688_ACCEL_SCALE_G;
    sample.accel_y_g =
        (float) icm42688_decode_int16(&buffer[2]) * ICM42688_ACCEL_SCALE_G;
    sample.accel_z_g =
        (float) icm42688_decode_int16(&buffer[4]) * ICM42688_ACCEL_SCALE_G;
    sample.gyro_x_dps =
        (float) icm42688_decode_int16(&buffer[6]) * ICM42688_GYRO_SCALE_DPS;
    sample.gyro_y_dps =
        (float) icm42688_decode_int16(&buffer[8]) * ICM42688_GYRO_SCALE_DPS;
    sample.gyro_z_dps =
        (float) icm42688_decode_int16(&buffer[10]) * ICM42688_GYRO_SCALE_DPS;

    *data = sample;
    return ML_STATUS_OK;
}
