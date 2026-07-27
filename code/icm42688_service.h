#ifndef ICM42688_SERVICE_H
#define ICM42688_SERVICE_H

#include "imu_attitude.h"
#include "ml_tim.h"

typedef enum {
    ICM42688_SERVICE_STATE_UNINITIALIZED = 0,
    ICM42688_SERVICE_STATE_CALIBRATING,
    ICM42688_SERVICE_STATE_READY,
    ICM42688_SERVICE_STATE_SENSOR_INIT_ERROR,
    ICM42688_SERVICE_STATE_ATTITUDE_INIT_ERROR,
    ICM42688_SERVICE_STATE_TIMER_HARDWARE_ERROR,
    ICM42688_SERVICE_STATE_TIMER_INTERRUPT_ERROR,
    ICM42688_SERVICE_STATE_SENSOR_READ_ERROR,
    ICM42688_SERVICE_STATE_ATTITUDE_UPDATE_ERROR
} icm42688_service_state_t;

typedef enum {
    ICM42688_SERVICE_EVENT_NONE = 0,
    ICM42688_SERVICE_EVENT_CALIBRATION_PROGRESS,
    ICM42688_SERVICE_EVENT_CALIBRATION_RESTARTED,
    ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE,
    ICM42688_SERVICE_EVENT_ANGLES_UPDATED,
    ICM42688_SERVICE_EVENT_READ_ERROR,
    ICM42688_SERVICE_EVENT_READ_RECOVERED,
    ICM42688_SERVICE_EVENT_TIMING_RESET,
    ICM42688_SERVICE_EVENT_UPDATE_ERROR,
    ICM42688_SERVICE_EVENT_UPDATE_RECOVERED
} icm42688_service_event_t;

typedef struct {
    imu_attitude_config_t axis_config;
    GPTIMER_Regs *timer;
    uint8_t timer_priority;
} icm42688_service_config_t;

typedef struct {
    imu_attitude_angles_t angles;
    uint16_t calibration_samples;
    uint32_t timestamp_ms;
} icm42688_service_output_t;

typedef struct {
    imu_attitude_t attitude;
    icm42688_service_config_t config;
    imu_attitude_angles_t angles;
    volatile uint32_t milliseconds;
    uint32_t last_sample_ms;
    ml_status_t last_status;
    icm42688_service_state_t state;
    bool initialized;
    bool calibrated;
    bool read_error;
    bool update_error;
} icm42688_service_t;

ml_status_t icm42688_service_init(icm42688_service_t *context,
    const icm42688_service_config_t *config);
icm42688_service_event_t icm42688_service_poll(
    icm42688_service_t *context, icm42688_service_output_t *output);
icm42688_service_state_t icm42688_service_get_state(
    const icm42688_service_t *context);
ml_status_t icm42688_service_get_last_status(
    const icm42688_service_t *context);

#endif
