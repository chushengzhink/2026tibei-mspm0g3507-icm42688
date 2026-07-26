#ifndef ML_I2C_H
#define ML_I2C_H

#include "ml_gpio.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t scl_pin;
    uint32_t scl_iomux;
    uint32_t sda_pin;
    uint32_t sda_iomux;
    uint32_t half_period_us;
    uint32_t timeout_us;
    bool initialized;
} ml_soft_i2c_bus_t;

#define I2C_GPIO         ML_MPU6050_I2C_PORT
#define I2C_SCL_GPIO_Pin ML_MPU6050_I2C_SCL_PIN
#define I2C_SDA_GPIO_Pin ML_MPU6050_I2C_SDA_PIN
#define I2C_SCL          PA1
#define I2C_SDA          PA0

extern ml_soft_i2c_bus_t g_mpu6050_i2c_bus;

ml_status_t soft_i2c_init(ml_soft_i2c_bus_t *bus);
ml_status_t soft_i2c_recover(ml_soft_i2c_bus_t *bus);
ml_status_t soft_i2c_write(ml_soft_i2c_bus_t *bus, uint8_t address_7bit,
    const uint8_t *data, uint32_t length);
ml_status_t soft_i2c_write_read(ml_soft_i2c_bus_t *bus,
    uint8_t address_7bit, const uint8_t *write_data, uint32_t write_length,
    uint8_t *read_data, uint32_t read_length);

ml_status_t I2C_Init(void);
ml_status_t I2C_Start(void);
ml_status_t I2C_Stop(void);
ml_status_t I2C_SendByte(uint8_t byte);
uint8_t I2C_ReceiveByte(void);
ml_status_t I2C_SendAck(void);
ml_status_t I2C_NotSendAck(void);
uint8_t I2C_WaitAck(void);

#endif
