#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include "FusionAhrs.h"
#include "icm42688.h"

#define IMU_ATTITUDE_CALIBRATION_SAMPLES (300U)

typedef enum {
    IMU_ATTITUDE_AXIS_X = 0,
    IMU_ATTITUDE_AXIS_Y,
    IMU_ATTITUDE_AXIS_Z
} imu_attitude_axis_t;

typedef struct {
    uint8_t source_axis[3];
    int8_t axis_sign[3];
} imu_attitude_config_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
} imu_attitude_angles_t;

typedef enum {
    IMU_ATTITUDE_CALIBRATION_IN_PROGRESS = 0,
    IMU_ATTITUDE_CALIBRATION_COMPLETE,
    IMU_ATTITUDE_CALIBRATION_RESTARTED,
    IMU_ATTITUDE_CALIBRATION_INVALID
} imu_attitude_calibration_status_t;

typedef struct {
    FusionAhrs ahrs;
    imu_attitude_config_t config;
    float gyro_bias_dps[3];
    float last_gyro_dps[3];
    float gyro_mean_dps[3];
    float gyro_m2[3];
    float accel_sum_g[3];
    uint16_t calibration_samples;
    bool initialized;
    bool calibrated;
} imu_attitude_t;

extern const imu_attitude_config_t imu_attitude_default_config;

ml_status_t imu_attitude_init(
    imu_attitude_t *context, const imu_attitude_config_t *config);
imu_attitude_calibration_status_t imu_attitude_calibration_update(
    imu_attitude_t *context, const icm42688_data_t *sample);
ml_status_t imu_attitude_update(imu_attitude_t *context,
    const icm42688_data_t *sample, float dt_s,
    imu_attitude_angles_t *angles);
uint16_t imu_attitude_calibration_progress(
    const imu_attitude_t *context);
ml_status_t imu_attitude_get_gyro_bias(
    const imu_attitude_t *context, float bias_dps[3]);
ml_status_t imu_attitude_get_body_gyro_dps(
    const imu_attitude_t *context, float gyro_dps[3]);

#endif
