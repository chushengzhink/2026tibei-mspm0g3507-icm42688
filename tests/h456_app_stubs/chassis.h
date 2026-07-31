#ifndef CHASSIS_H
#define CHASSIS_H

#include "chassis_config.h"
#include "ml_common.h"

typedef enum {
    CHASSIS_RESULT_IDLE = 0,
    CHASSIS_RESULT_RUNNING,
    CHASSIS_RESULT_COMPLETE,
    CHASSIS_RESULT_CANCELLED,
    CHASSIS_RESULT_FAULT
} chassis_result_t;

typedef struct {
    float left_distance_mm;
    float right_distance_mm;
} chassis_pose_t;

typedef struct {
    chassis_result_t result;
    chassis_pose_t pose;
    float target_left_mm_s;
    float target_right_mm_s;
    float measured_left_mm_s;
    float measured_right_mm_s;
    float fused_heading_deg;
    uint32_t timestamp_ms;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    bool emergency_stop_latched;
} chassis_status_t;

ml_status_t chassis_init(const chassis_config_t *config);
void chassis_poll(void);
chassis_status_t chassis_get_status(void);
void chassis_set_imu_sample(float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid);
void chassis_reset_pose(float x_mm, float y_mm, float heading_deg);
ml_status_t chassis_set_velocity(float linear_mm_s, float angular_rad_s);
ml_status_t chassis_update_velocity(float linear_mm_s, float angular_rad_s);
void chassis_stop(void);
void chassis_emergency_stop(void);

#endif
