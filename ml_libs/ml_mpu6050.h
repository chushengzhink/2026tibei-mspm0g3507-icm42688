#ifndef ML_MPU6050_H
#define ML_MPU6050_H

#include <stdint.h>

#include "ml_common.h"

#define MPU6050_ADDR              (0x68U)
#define MPU6050_WHO_AM_I_VALUE    (0x68U)

#define MPU6050_REG_SMPLRT_DIV    (0x19U)
#define MPU6050_REG_CONFIG        (0x1AU)
#define MPU6050_REG_GYRO_CONFIG   (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG  (0x1CU)
#define MPU6050_REG_INT_ENABLE    (0x38U)
#define MPU6050_REG_ACCEL_XOUT_H  (0x3BU)
#define MPU6050_REG_TEMP_OUT_H    (0x41U)
#define MPU6050_REG_GYRO_XOUT_H   (0x43U)
#define MPU6050_REG_PWR_MGMT_1    (0x6BU)
#define MPU6050_REG_PWR_MGMT_2    (0x6CU)
#define MPU6050_REG_WHO_AM_I      (0x75U)

/* Legacy register names retained for existing application code. */
#define SMPLRT_DIV       MPU6050_REG_SMPLRT_DIV
#define CONFIG           MPU6050_REG_CONFIG
#define GYRO_CONFIG      MPU6050_REG_GYRO_CONFIG
#define ACCEL_CONFIG     MPU6050_REG_ACCEL_CONFIG
#define ACCEL_XOUT_H     MPU6050_REG_ACCEL_XOUT_H
#define ACCEL_XOUT_L     (0x3CU)
#define ACCEL_YOUT_H     (0x3DU)
#define ACCEL_YOUT_L     (0x3EU)
#define ACCEL_ZOUT_H     (0x3FU)
#define ACCEL_ZOUT_L     (0x40U)
#define TEMP_OUT_H       MPU6050_REG_TEMP_OUT_H
#define TEMP_OUT_L       (0x42U)
#define GYRO_XOUT_H      MPU6050_REG_GYRO_XOUT_H
#define GYRO_XOUT_L      (0x44U)
#define GYRO_YOUT_H      (0x45U)
#define GYRO_YOUT_L      (0x46U)
#define GYRO_ZOUT_H      (0x47U)
#define GYRO_ZOUT_L      (0x48U)
#define PWR_MGMT_1       MPU6050_REG_PWR_MGMT_1
#define PWR_MGMT_2       MPU6050_REG_PWR_MGMT_2
#define INT_ENABLE       MPU6050_REG_INT_ENABLE
#define WHO_AM_I         MPU6050_REG_WHO_AM_I

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float temperature_c;
} mpu6050_data_t;

extern mpu6050_data_t g_mpu6050_data;
extern int16_t ax, ay, az, gx, gy, gz;

ml_status_t mpu6050_write_register(uint8_t reg, uint8_t value);
ml_status_t mpu6050_read_register(uint8_t reg, uint8_t *value);
ml_status_t mpu6050_read_data(mpu6050_data_t *data);

ml_status_t MPU6050_Write(uint8_t addr, uint8_t dat);
uint8_t MPU6050_Read(uint8_t addr);
ml_status_t MPU6050_Init(void);
ml_status_t MPU6050_GetData(void);

#endif
