#ifndef ICM42688_SERVICE_H
#define ICM42688_SERVICE_H

#include "ml_board.h"

typedef enum {
    IMU_ATTITUDE_AXIS_X = 0,
    IMU_ATTITUDE_AXIS_Y,
    IMU_ATTITUDE_AXIS_Z
} imu_attitude_axis_t;

typedef struct {
    imu_attitude_axis_t body_axis[3];
    int8_t body_sign[3];
    uint16_t calibration_samples_required;
} imu_attitude_config_t;

typedef struct {
    imu_attitude_config_t axis_config;
    GPTIMER_Regs *timer;
    uint8_t timer_priority;
} icm42688_service_config_t;

typedef struct { uint8_t unused; } icm42688_service_t;

typedef struct {
    struct { float yaw_deg; } angles;
    float body_gyro_z_dps;
    uint32_t timestamp_ms;
} icm42688_service_output_t;

typedef enum {
    ICM42688_SERVICE_EVENT_NONE = 0,
    ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE,
    ICM42688_SERVICE_EVENT_ANGLES_UPDATED
} icm42688_service_event_t;

ml_status_t icm42688_service_init(
    icm42688_service_t *context,
    const icm42688_service_config_t *config);
icm42688_service_event_t icm42688_service_poll(
    icm42688_service_t *context,
    icm42688_service_output_t *output);

#endif
