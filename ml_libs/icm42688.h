#ifndef ICM42688_H
#define ICM42688_H

#include "ml_common.h"

#define ICM42688_I2C_ADDRESS (0x68U)
#define ICM42688_WHO_AM_I_VALUE (0x47U)

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} icm42688_data_t;

ml_status_t icm42688_init(void);
ml_status_t icm42688_read(icm42688_data_t *data);

#endif
