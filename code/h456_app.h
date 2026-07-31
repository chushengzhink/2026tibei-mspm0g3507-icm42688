#ifndef H456_APP_H
#define H456_APP_H

#include "h456_mission.h"
#include "ml_common.h"

typedef enum {
    H456_APP_CALIBRATING = 0,
    H456_APP_SETUP,
    H456_APP_READY,
    H456_APP_RUNNING,
    H456_APP_FINISHED,
    H456_APP_RECENTERING,
    H456_APP_EXPORT_READY,
    H456_APP_FAULT
} h456_app_state_t;

typedef enum {
    H456_APP_FAULT_NONE = 0,
    H456_APP_FAULT_INIT,
    H456_APP_FAULT_LINE_GPIO,
    H456_APP_FAULT_CHASSIS,
    H456_APP_FAULT_VISION,
    H456_APP_FAULT_BALL_ERROR,
    H456_APP_FAULT_BALL_CONTROL,
    H456_APP_FAULT_LAP_GATE,
    H456_APP_FAULT_EMERGENCY
} h456_app_fault_t;

typedef struct {
    h456_app_state_t state;
    h456_app_fault_t fault;
    h456_mode_t mode;
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
} h456_app_status_t;

ml_status_t h456_app_init(void);
void h456_app_poll(void);
ml_status_t h456_app_get_status(h456_app_status_t *status);

#endif
