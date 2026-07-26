#ifndef ML_BMI088_H
#define ML_BMI088_H

#include <stdint.h>

#include "ml_common.h"

typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} bmi088_data_t;

typedef enum {
    BMI088_DIAG_STAGE_NONE = 0,
    BMI088_DIAG_STAGE_RESOURCE,
    BMI088_DIAG_STAGE_GPIO_CS_A,
    BMI088_DIAG_STAGE_GPIO_CS_G,
    BMI088_DIAG_STAGE_GPIO_SCLK,
    BMI088_DIAG_STAGE_GPIO_MOSI,
    BMI088_DIAG_STAGE_GPIO_MISO,
    BMI088_DIAG_STAGE_ID_INITIAL,
    BMI088_DIAG_STAGE_GYRO_RECOVERY,
    BMI088_DIAG_STAGE_ID_POST_RESET,
    BMI088_DIAG_STAGE_ACCEL_CONFIG,
    BMI088_DIAG_STAGE_GYRO_CONFIG,
    BMI088_DIAG_STAGE_ID_RUNTIME
} bmi088_diag_stage_t;

typedef enum {
    BMI088_GYRO_MISO_UNKNOWN = 0,
    BMI088_GYRO_MISO_VALID,
    BMI088_GYRO_MISO_HIGH_Z,
    BMI088_GYRO_MISO_STUCK_LOW,
    BMI088_GYRO_MISO_STUCK_HIGH,
    BMI088_GYRO_MISO_UNSTABLE
} bmi088_gyro_miso_state_t;

typedef struct {
    bmi088_diag_stage_t stage;
    ml_status_t status;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint8_t gyro_id_before_reset;
    uint8_t gyro_id_after_reset;
    uint8_t gyro_id_miso_float;
    uint8_t gyro_id_miso_pullup;
    uint8_t gyro_id_miso_pulldown;
    bmi088_gyro_miso_state_t gyro_miso_state;
    uint8_t register_address;
    uint8_t expected_value;
    uint8_t actual_value;
} bmi088_diagnostic_t;

ml_status_t bmi088_init(void);
ml_status_t bmi088_read(bmi088_data_t *data);
ml_status_t bmi088_get_diagnostic(bmi088_diagnostic_t *diagnostic);
ml_status_t bmi088_diagnostic_set_cs_g(bool high);

#endif
