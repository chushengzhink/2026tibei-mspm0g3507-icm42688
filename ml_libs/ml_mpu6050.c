#include "ml_mpu6050.h"

#include "ml_delay.h"
#include "ml_i2c.h"

mpu6050_data_t g_mpu6050_data;
int16_t ax, ay, az, gx, gy, gz;

static int16_t mpu6050_decode_word(const uint8_t *bytes)
{
    return (int16_t) (((uint16_t) bytes[0] << 8) | bytes[1]);
}

ml_status_t mpu6050_write_register(uint8_t reg, uint8_t value)
{
    uint8_t packet[2];

    packet[0] = reg;
    packet[1] = value;
    return soft_i2c_write(
        &g_mpu6050_i2c_bus, MPU6050_ADDR, packet, sizeof(packet));
}

ml_status_t mpu6050_read_register(uint8_t reg, uint8_t *value)
{
    if (value == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    return soft_i2c_write_read(
        &g_mpu6050_i2c_bus, MPU6050_ADDR, &reg, 1U, value, 1U);
}

ml_status_t MPU6050_Write(uint8_t addr, uint8_t dat)
{
    return mpu6050_write_register(addr, dat);
}

uint8_t MPU6050_Read(uint8_t addr)
{
    uint8_t value = 0U;

    (void) mpu6050_read_register(addr, &value);
    return value;
}

ml_status_t MPU6050_Init(void)
{
    uint8_t identity;
    ml_status_t status = I2C_Init();

    if (status != ML_STATUS_OK) {
        return status;
    }
    status = mpu6050_read_register(MPU6050_REG_WHO_AM_I, &identity);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (identity != MPU6050_WHO_AM_I_VALUE) {
        return ML_STATUS_DEVICE_NOT_FOUND;
    }

    status = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x80U);
    if (status != ML_STATUS_OK) {
        return status;
    }
    delay_ms(100U);
    status = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x02U);
    if (status == ML_STATUS_OK) {
        status = mpu6050_write_register(MPU6050_REG_PWR_MGMT_2, 0x00U);
    }
    if (status == ML_STATUS_OK) {
        status = mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x27U);
    }
    if (status == ML_STATUS_OK) {
        status = mpu6050_write_register(MPU6050_REG_CONFIG, 0x00U);
    }
    if (status == ML_STATUS_OK) {
        status = mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, 0x18U);
    }
    if (status == ML_STATUS_OK) {
        status = mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, 0x18U);
    }
    return status;
}

ml_status_t mpu6050_read_data(mpu6050_data_t *data)
{
    uint8_t first_register = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t raw[14];
    ml_status_t status;

    if (data == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = soft_i2c_write_read(&g_mpu6050_i2c_bus, MPU6050_ADDR,
        &first_register, 1U, raw, sizeof(raw));
    if (status != ML_STATUS_OK) {
        return status;
    }

    data->accel_x = mpu6050_decode_word(&raw[0]);
    data->accel_y = mpu6050_decode_word(&raw[2]);
    data->accel_z = mpu6050_decode_word(&raw[4]);
    data->temperature_raw = mpu6050_decode_word(&raw[6]);
    data->gyro_x = mpu6050_decode_word(&raw[8]);
    data->gyro_y = mpu6050_decode_word(&raw[10]);
    data->gyro_z = mpu6050_decode_word(&raw[12]);
    data->temperature_c = ((float) data->temperature_raw / 340.0f) + 36.53f;
    return ML_STATUS_OK;
}

ml_status_t MPU6050_GetData(void)
{
    ml_status_t status = mpu6050_read_data(&g_mpu6050_data);

    if (status == ML_STATUS_OK) {
        ax = g_mpu6050_data.accel_x;
        ay = g_mpu6050_data.accel_y;
        az = g_mpu6050_data.accel_z;
        gx = g_mpu6050_data.gyro_x;
        gy = g_mpu6050_data.gyro_y;
        gz = g_mpu6050_data.gyro_z;
    }
    return status;
}
