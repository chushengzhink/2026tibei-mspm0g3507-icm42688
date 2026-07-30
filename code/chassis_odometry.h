#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdint.h>

#include "ml_common.h"

typedef struct {
    float x_mm;
    float y_mm;
    float heading_rad;
    float encoder_heading_rad;
    float left_distance_mm;
    float right_distance_mm;
    int32_t left_ticks;
    int32_t right_ticks;
} chassis_pose_t;

typedef struct {
    float left_mm_per_tick;
    float right_mm_per_tick;
    float effective_track_mm;
    chassis_pose_t pose;
    bool initialized;
} chassis_odometry_t;

ml_status_t chassis_odometry_init(chassis_odometry_t *odometry,
    float left_mm_per_tick, float right_mm_per_tick,
    float effective_track_mm);
ml_status_t chassis_odometry_reset(chassis_odometry_t *odometry,
    float x_mm, float y_mm, float heading_rad);
ml_status_t chassis_odometry_update(chassis_odometry_t *odometry,
    int32_t left_delta_ticks, int32_t right_delta_ticks);
ml_status_t chassis_odometry_update_with_heading(
    chassis_odometry_t *odometry,
    int32_t left_delta_ticks, int32_t right_delta_ticks,
    float fused_heading_rad);
chassis_pose_t chassis_odometry_get(const chassis_odometry_t *odometry);

#endif
