#ifndef H5_APP_H
#define H5_APP_H

#include "h5_mission.h"
#include "ml_common.h"

typedef enum {
    H5_APP_CALIBRATING = 0,
    H5_APP_SETUP,
    H5_APP_READY,
    H5_APP_RUNNING,
    H5_APP_FINISHED,
    H5_APP_RECENTERING,
    H5_APP_EXPORT_READY,
    H5_APP_FAULT
} h5_app_state_t;

typedef enum {
    H5_APP_FAULT_NONE = 0,
    H5_APP_FAULT_INIT,
    H5_APP_FAULT_LINE_GPIO,
    H5_APP_FAULT_CHASSIS,
    H5_APP_FAULT_VISION,
    H5_APP_FAULT_BALL_ERROR,
    H5_APP_FAULT_BALL_CONTROL,
    H5_APP_FAULT_LAP_GATE,
    H5_APP_FAULT_EMERGENCY
} h5_app_fault_t;

typedef struct {
    h5_app_state_t state;
    h5_app_fault_t fault;
    h5_mode_t mode;
    float target_cm;
    float ball_position_cm;
    float ball_error_cm;
    float maximum_score_error_cm;
    uint32_t score_elapsed_ms;
    uint16_t telemetry_count;
    bool imu_ready;
    bool white_ready;
    bool vision_ready;
    bool ball_settled;
    bool ball_violation;
    bool score_point_passed;
    bool export_allowed;
    bool oled_frozen;
} h5_app_status_t;

ml_status_t h5_app_init(void);
void h5_app_poll(void);
ml_status_t h5_app_get_status(h5_app_status_t *status);

#endif

