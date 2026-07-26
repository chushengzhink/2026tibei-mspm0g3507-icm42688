#include "ml_i2c.h"

#include "ml_delay.h"

ml_soft_i2c_bus_t g_mpu6050_i2c_bus = {
    ML_MPU6050_I2C_PORT,
    ML_MPU6050_I2C_SCL_PIN,
    ML_MPU6050_I2C_SCL_IOMUX,
    ML_MPU6050_I2C_SDA_PIN,
    ML_MPU6050_I2C_SDA_IOMUX,
    ML_SOFT_I2C_HALF_PERIOD_US,
    ML_SOFT_I2C_TIMEOUT_US,
    false
};

static ml_status_t g_legacy_write_status = ML_STATUS_NOT_INITIALIZED;

static void soft_i2c_delay(const ml_soft_i2c_bus_t *bus)
{
    delay_us(bus->half_period_us);
}

static void soft_i2c_drive_low(
    ml_soft_i2c_bus_t *bus, uint32_t pin, uint32_t iomux)
{
    DL_GPIO_initDigitalOutput(iomux);
    bus->port->DOUTCLR31_0 = pin;
    DL_GPIO_enableOutput(bus->port, pin);
}

static void soft_i2c_release(
    ml_soft_i2c_bus_t *bus, uint32_t pin, uint32_t iomux)
{
    DL_GPIO_disableOutput(bus->port, pin);
    DL_GPIO_initDigitalInputFeatures(iomux, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static bool soft_i2c_read_line(
    const ml_soft_i2c_bus_t *bus, uint32_t pin)
{
    return (bus->port->DIN31_0 & pin) != 0U;
}

static ml_status_t soft_i2c_release_scl(ml_soft_i2c_bus_t *bus)
{
    uint32_t elapsed = 0U;

    soft_i2c_release(bus, bus->scl_pin, bus->scl_iomux);
    while (!soft_i2c_read_line(bus, bus->scl_pin)) {
        if (elapsed >= bus->timeout_us) {
            return ML_STATUS_TIMEOUT;
        }
        delay_us(1U);
        ++elapsed;
    }
    soft_i2c_delay(bus);
    return ML_STATUS_OK;
}

static ml_status_t soft_i2c_start_condition(ml_soft_i2c_bus_t *bus)
{
    ml_status_t status;

    if ((bus == 0) || !bus->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    status = soft_i2c_release_scl(bus);
    if (status != ML_STATUS_OK) {
        return status;
    }
    if (!soft_i2c_read_line(bus, bus->sda_pin)) {
        return ML_STATUS_BUSY;
    }
    soft_i2c_drive_low(bus, bus->sda_pin, bus->sda_iomux);
    soft_i2c_delay(bus);
    soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
    return ML_STATUS_OK;
}

static ml_status_t soft_i2c_stop_condition(ml_soft_i2c_bus_t *bus)
{
    ml_status_t status;

    if ((bus == 0) || !bus->initialized) {
        return ML_STATUS_NOT_INITIALIZED;
    }

    soft_i2c_drive_low(bus, bus->sda_pin, bus->sda_iomux);
    soft_i2c_delay(bus);
    status = soft_i2c_release_scl(bus);
    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    soft_i2c_delay(bus);
    if ((status == ML_STATUS_OK) &&
        !soft_i2c_read_line(bus, bus->sda_pin)) {
        return ML_STATUS_BUSY;
    }
    return status;
}

static ml_status_t soft_i2c_write_byte(
    ml_soft_i2c_bus_t *bus, uint8_t byte)
{
    uint8_t bit;
    ml_status_t status;
    bool acknowledged;

    for (bit = 0U; bit < 8U; ++bit) {
        if ((byte & 0x80U) != 0U) {
            soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
        } else {
            soft_i2c_drive_low(bus, bus->sda_pin, bus->sda_iomux);
        }
        soft_i2c_delay(bus);
        status = soft_i2c_release_scl(bus);
        if (status != ML_STATUS_OK) {
            return status;
        }
        soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
        byte <<= 1;
    }

    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    status = soft_i2c_release_scl(bus);
    if (status != ML_STATUS_OK) {
        return status;
    }
    acknowledged = !soft_i2c_read_line(bus, bus->sda_pin);
    soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
    soft_i2c_delay(bus);
    return acknowledged ? ML_STATUS_OK : ML_STATUS_NO_ACK;
}

static ml_status_t soft_i2c_read_bits(
    ml_soft_i2c_bus_t *bus, uint8_t *byte)
{
    uint8_t bit;
    uint8_t value = 0U;
    ml_status_t status;

    if (byte == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    for (bit = 0U; bit < 8U; ++bit) {
        value <<= 1;
        status = soft_i2c_release_scl(bus);
        if (status != ML_STATUS_OK) {
            return status;
        }
        if (soft_i2c_read_line(bus, bus->sda_pin)) {
            value |= 1U;
        }
        soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
        soft_i2c_delay(bus);
    }
    *byte = value;
    return ML_STATUS_OK;
}

static ml_status_t soft_i2c_send_ack_bit(
    ml_soft_i2c_bus_t *bus, bool acknowledge)
{
    ml_status_t status;

    if (acknowledge) {
        soft_i2c_drive_low(bus, bus->sda_pin, bus->sda_iomux);
    } else {
        soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    }
    soft_i2c_delay(bus);
    status = soft_i2c_release_scl(bus);
    soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    soft_i2c_delay(bus);
    return status;
}

ml_status_t soft_i2c_recover(ml_soft_i2c_bus_t *bus)
{
    uint8_t pulse;
    ml_status_t status = ML_STATUS_OK;

    if (bus == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    for (pulse = 0U; pulse < 9U; ++pulse) {
        soft_i2c_drive_low(bus, bus->scl_pin, bus->scl_iomux);
        soft_i2c_delay(bus);
        status = soft_i2c_release_scl(bus);
        if (status != ML_STATUS_OK) {
            return status;
        }
        if (soft_i2c_read_line(bus, bus->sda_pin)) {
            break;
        }
    }
    bus->initialized = true;
    return soft_i2c_stop_condition(bus);
}

ml_status_t soft_i2c_init(ml_soft_i2c_bus_t *bus)
{
    ml_status_t status;

    if ((bus == 0) || ((bus->port != GPIOA) && (bus->port != GPIOB)) ||
        (bus->scl_pin == 0U) || (bus->sda_pin == 0U) ||
        (bus->scl_pin == bus->sda_pin) || (bus->half_period_us == 0U) ||
        (bus->timeout_us == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    bus->initialized = true;
    soft_i2c_release(bus, bus->scl_pin, bus->scl_iomux);
    soft_i2c_release(bus, bus->sda_pin, bus->sda_iomux);
    soft_i2c_delay(bus);

    if (!soft_i2c_read_line(bus, bus->scl_pin) ||
        !soft_i2c_read_line(bus, bus->sda_pin)) {
        status = soft_i2c_recover(bus);
        if (status != ML_STATUS_OK) {
            bus->initialized = false;
        }
        return status;
    }
    return ML_STATUS_OK;
}

ml_status_t soft_i2c_write(ml_soft_i2c_bus_t *bus, uint8_t address_7bit,
    const uint8_t *data, uint32_t length)
{
    uint32_t index;
    ml_status_t status;

    if ((bus == 0) || (address_7bit > 0x7FU) ||
        ((length != 0U) && (data == 0))) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    status = soft_i2c_start_condition(bus);
    if (status == ML_STATUS_BUSY) {
        status = soft_i2c_recover(bus);
        if (status == ML_STATUS_OK) {
            status = soft_i2c_start_condition(bus);
        }
    }
    if (status != ML_STATUS_OK) {
        return status;
    }

    status = soft_i2c_write_byte(bus, (uint8_t) (address_7bit << 1));
    for (index = 0U; (status == ML_STATUS_OK) && (index < length); ++index) {
        status = soft_i2c_write_byte(bus, data[index]);
    }
    {
        ml_status_t stop_status = soft_i2c_stop_condition(bus);

        if (status == ML_STATUS_OK) {
            status = stop_status;
        }
    }
    return status;
}

ml_status_t soft_i2c_write_read(ml_soft_i2c_bus_t *bus,
    uint8_t address_7bit, const uint8_t *write_data, uint32_t write_length,
    uint8_t *read_data, uint32_t read_length)
{
    uint32_t index;
    ml_status_t status;

    if ((bus == 0) || (address_7bit > 0x7FU) ||
        ((write_length != 0U) && (write_data == 0)) ||
        ((read_length != 0U) && (read_data == 0)) || (read_length == 0U)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    status = soft_i2c_start_condition(bus);
    if (status == ML_STATUS_BUSY) {
        status = soft_i2c_recover(bus);
        if (status == ML_STATUS_OK) {
            status = soft_i2c_start_condition(bus);
        }
    }
    if (status != ML_STATUS_OK) {
        return status;
    }
    status = soft_i2c_write_byte(bus, (uint8_t) (address_7bit << 1));
    for (index = 0U;
         (status == ML_STATUS_OK) && (index < write_length); ++index) {
        status = soft_i2c_write_byte(bus, write_data[index]);
    }
    if (status == ML_STATUS_OK) {
        status = soft_i2c_start_condition(bus);
    }
    if (status == ML_STATUS_OK) {
        status = soft_i2c_write_byte(
            bus, (uint8_t) ((address_7bit << 1) | 1U));
    }
    for (index = 0U;
         (status == ML_STATUS_OK) && (index < read_length); ++index) {
        status = soft_i2c_read_bits(bus, &read_data[index]);
        if (status == ML_STATUS_OK) {
            status = soft_i2c_send_ack_bit(bus, index + 1U < read_length);
        }
    }
    {
        ml_status_t stop_status = soft_i2c_stop_condition(bus);

        if (status == ML_STATUS_OK) {
            status = stop_status;
        }
    }
    return status;
}

ml_status_t I2C_Init(void)
{
    return soft_i2c_init(&g_mpu6050_i2c_bus);
}

ml_status_t I2C_Start(void)
{
    return soft_i2c_start_condition(&g_mpu6050_i2c_bus);
}

ml_status_t I2C_Stop(void)
{
    return soft_i2c_stop_condition(&g_mpu6050_i2c_bus);
}

ml_status_t I2C_SendByte(uint8_t byte)
{
    g_legacy_write_status =
        soft_i2c_write_byte(&g_mpu6050_i2c_bus, byte);
    return g_legacy_write_status;
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t byte = 0U;

    (void) soft_i2c_read_bits(&g_mpu6050_i2c_bus, &byte);
    return byte;
}

ml_status_t I2C_SendAck(void)
{
    return soft_i2c_send_ack_bit(&g_mpu6050_i2c_bus, true);
}

ml_status_t I2C_NotSendAck(void)
{
    return soft_i2c_send_ack_bit(&g_mpu6050_i2c_bus, false);
}

uint8_t I2C_WaitAck(void)
{
    return (g_legacy_write_status == ML_STATUS_OK) ? 0U : 1U;
}
