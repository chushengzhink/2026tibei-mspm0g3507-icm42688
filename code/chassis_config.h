#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

typedef struct {
    float mm_per_tick;
    float kp;
    float ki;
    float kd;
    float feedforward;
    float pid_output_limit;
    float pid_integral_limit;
    int8_t encoder_sign;
    int8_t motor_sign;
} chassis_wheel_config_t;

typedef struct {
    chassis_wheel_config_t left;
    chassis_wheel_config_t right;
    float effective_track_mm;
    float speed_filter_alpha;
    float acceleration_mm_s2;
    float straight_correction_kp;
    float maximum_wheel_speed_mm_s;
    float minimum_profile_speed_mm_s;
    float distance_tolerance_mm;
    float angle_tolerance_deg;
    float stall_command_speed_mm_s;
    float heading_fusion_time_constant_s;
    float heading_rate_imu_weight;
    float imu_heading_sign;
    float imu_max_delta_deg;
    float heading_control_kp;
    float yaw_rate_control_kp;
    float heading_correction_limit_ratio;
    uint16_t control_period_ms;
    uint16_t imu_stale_ms;
    uint8_t completion_cycles;
    uint8_t stall_cycles;
} chassis_config_t;

#define CHASSIS_MG513X_PPR                 (13U)
#define CHASSIS_MG513X_GEAR_RATIO          (28U)
#define CHASSIS_ENCODER_DECODE_FACTOR      (4U)
#define CHASSIS_TICKS_PER_WHEEL_REVOLUTION (1456U)
#define CHASSIS_NOMINAL_WHEEL_DIAMETER_MM  (65.0f)
#define CHASSIS_DEFAULT_MM_PER_TICK         (0.1402497f)
#define CHASSIS_CONTROL_PERIOD_MS           (20U)
#define CHASSIS_SPEED_FILTER_ALPHA          (0.35f)
#define CHASSIS_VELOCITY_KP                 (700.0f)
#define CHASSIS_VELOCITY_KI                 (55.0f)
#define CHASSIS_VELOCITY_KD                 (0.0f)
#define CHASSIS_MOTOR_FEEDFORWARD           (6000.0f)
#define CHASSIS_PID_OUTPUT_LIMIT             (11500.0f)
#define CHASSIS_PID_INTEGRAL_LIMIT           (5750.0f)
#define CHASSIS_RACE_PID_OUTPUT_LIMIT         (14000.0f)
#define CHASSIS_RACE_PID_INTEGRAL_LIMIT       (7000.0f)
#define CHASSIS_HEADING_FUSION_TIME_CONSTANT_S (1.0f)
#define CHASSIS_HEADING_RATE_IMU_WEIGHT       (0.75f)
#define CHASSIS_IMU_HEADING_SIGN              (-1.0f)
#define CHASSIS_IMU_MAX_DELTA_DEG             (25.0f)
#define CHASSIS_HEADING_CONTROL_KP            (4.0f)
#define CHASSIS_YAW_RATE_CONTROL_KP           (0.20f)
#define CHASSIS_HEADING_CORRECTION_LIMIT_RATIO (0.25f)
#define CHASSIS_IMU_STALE_MS                  (100U)

/* Set to 1 only when re-running the non-motor SW6 mapping diagnostic. */
#define CHASSIS_SW6_MAPPING_DIAGNOSTIC       (0U)

extern const chassis_config_t g_chassis_default_config;
extern const chassis_config_t g_chassis_race_config;

#endif
