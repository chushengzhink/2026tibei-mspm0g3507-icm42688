#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"
#include "chassis_odometry.h"
#include "ml_common.h"

typedef enum {
    CHASSIS_MODE_IDLE = 0,
    CHASSIS_MODE_WHEEL_SPEED,
    CHASSIS_MODE_VELOCITY,
    CHASSIS_MODE_MOVE,
    CHASSIS_MODE_ROTATE,
    CHASSIS_MODE_ARC
} chassis_mode_t;

typedef enum {
    CHASSIS_RESULT_IDLE = 0,
    CHASSIS_RESULT_RUNNING,
    CHASSIS_RESULT_COMPLETE,
    CHASSIS_RESULT_CANCELLED,
    CHASSIS_RESULT_FAULT
} chassis_result_t;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_ENCODER,
    CHASSIS_FAULT_STALL,
    CHASSIS_FAULT_MOTOR_DRIVER,
    CHASSIS_FAULT_EMERGENCY_STOP
} chassis_fault_t;

typedef struct {
    chassis_mode_t mode;
    chassis_result_t result;
    chassis_fault_t fault;
    chassis_pose_t pose;
    float target_left_mm_s;
    float target_right_mm_s;
    float measured_left_mm_s;
    float measured_right_mm_s;
    float command_progress;
    float command_remaining;
    float imu_yaw_deg;
    float encoder_heading_deg;
    float fused_heading_deg;
    float fused_yaw_rate_dps;
    int32_t encoder_total_left;
    int32_t encoder_total_right;
    uint32_t encoder_invalid_left;
    uint32_t encoder_invalid_right;
    uint32_t timestamp_ms;
    uint16_t pwm_left_count;
    uint16_t pwm_right_count;
    bool emergency_stop_latched;
    bool heading_fusion_active;
} chassis_status_t;

ml_status_t chassis_init(const chassis_config_t *config);
ml_status_t chassis_set_wheel_speed(
    float left_mm_s, float right_mm_s);
/* Update active direct wheel targets without resetting PID or stall state. */
ml_status_t chassis_update_wheel_speed(
    float left_mm_s, float right_mm_s);
ml_status_t chassis_set_velocity(
    float linear_mm_s, float angular_rad_s);
/* Update an active velocity command without resetting PID or stall state. */
ml_status_t chassis_update_velocity(
    float linear_mm_s, float angular_rad_s);
ml_status_t chassis_move_mm(
    float distance_mm, float max_speed_mm_s);
ml_status_t chassis_rotate_deg(
    float angle_deg, float max_angular_deg_s);
ml_status_t chassis_arc(
    float radius_mm, float angle_deg, float max_speed_mm_s);
void chassis_stop(void);
void chassis_emergency_stop(void);
void chassis_reset_pose(float x_mm, float y_mm, float heading_deg);
chassis_status_t chassis_get_status(void);
ml_status_t chassis_idle_capture_start(void);
void chassis_idle_capture_stop(void);
bool chassis_idle_capture_active(void);
ml_status_t chassis_capture_telemetry_now(void);

/* Call continuously from the main loop; work runs when the 20 ms tick is due. */
void chassis_poll(void);
void chassis_set_imu_yaw(float yaw_deg);
void chassis_set_imu_sample(float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid);

#endif
