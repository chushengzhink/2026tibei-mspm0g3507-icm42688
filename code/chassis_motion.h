#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "chassis.h"
#include "chassis_heading_fusion.h"

typedef struct {
    chassis_config_t config;
    chassis_odometry_t odometry;
    chassis_heading_fusion_t heading_fusion;
    chassis_status_t status;
    float command_target;
    float command_limit;
    float arc_radius_mm;
    float start_left_mm;
    float start_right_mm;
    float start_fused_heading_rad;
    float requested_left_mm_s;
    float requested_right_mm_s;
    float profile_speed_mm_s;
    uint16_t left_stall_cycles;
    uint16_t right_stall_cycles;
    uint8_t completion_count;
    bool initialized;
} chassis_motion_t;

ml_status_t chassis_motion_init(
    chassis_motion_t *motion, const chassis_config_t *config);
ml_status_t chassis_motion_set_wheel_speed(
    chassis_motion_t *motion, float left_mm_s, float right_mm_s,
    chassis_mode_t mode);
ml_status_t chassis_motion_update_wheel_speed(
    chassis_motion_t *motion, float left_mm_s, float right_mm_s,
    chassis_mode_t mode);
ml_status_t chassis_motion_move(
    chassis_motion_t *motion, float distance_mm, float max_speed_mm_s);
ml_status_t chassis_motion_rotate(
    chassis_motion_t *motion, float angle_rad, float max_angular_rad_s);
ml_status_t chassis_motion_arc(chassis_motion_t *motion,
    float radius_mm, float angle_rad, float max_speed_mm_s);
void chassis_motion_stop(chassis_motion_t *motion, bool emergency);
ml_status_t chassis_motion_reset_pose(chassis_motion_t *motion,
    float x_mm, float y_mm, float heading_rad);
ml_status_t chassis_motion_update(chassis_motion_t *motion,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    uint8_t elapsed_cycles);
void chassis_motion_set_imu_sample(chassis_motion_t *motion,
    float yaw_deg, float body_gyro_z_dps,
    uint32_t timestamp_ms, bool valid);

#endif
