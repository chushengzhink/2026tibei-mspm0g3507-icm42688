#include "chassis_config.h"

const chassis_config_t g_chassis_default_config = {
    {
        0.1413727f, /* provisional: one forward/reverse 1 m pair */
        CHASSIS_VELOCITY_KP,
        CHASSIS_VELOCITY_KI,
        CHASSIS_VELOCITY_KD,
        CHASSIS_MOTOR_FEEDFORWARD,
        CHASSIS_PID_OUTPUT_LIMIT,
        CHASSIS_PID_INTEGRAL_LIMIT,
        -1, /* encoder_sign: physical forward must count positive */
        -1  /* motor_sign: positive command must drive physical forward */
    },
    {
        0.1434926f, /* provisional: one forward/reverse 1 m pair */
        CHASSIS_VELOCITY_KP,
        CHASSIS_VELOCITY_KI,
        CHASSIS_VELOCITY_KD,
        CHASSIS_MOTOR_FEEDFORWARD,
        CHASSIS_PID_OUTPUT_LIMIT,
        CHASSIS_PID_INTEGRAL_LIMIT,
        -1, /* encoder_sign: physical forward must count positive */
        1   /* motor_sign: positive command must drive physical forward */
    },
    214.2f,
    CHASSIS_SPEED_FILTER_ALPHA,
    400.0f,
    2.0f,
    500.0f,
    25.0f,
    2.0f,
    1.0f,
    30.0f,
    CHASSIS_HEADING_FUSION_TIME_CONSTANT_S,
    CHASSIS_HEADING_RATE_IMU_WEIGHT,
    CHASSIS_IMU_HEADING_SIGN,
    CHASSIS_IMU_MAX_DELTA_DEG,
    CHASSIS_HEADING_CONTROL_KP,
    CHASSIS_YAW_RATE_CONTROL_KP,
    CHASSIS_HEADING_CORRECTION_LIMIT_RATIO,
    CHASSIS_CONTROL_PERIOD_MS,
    CHASSIS_IMU_STALE_MS,
    3U,
    8U
};

const chassis_config_t g_chassis_race_config = {
    {
        0.1413727f,
        CHASSIS_VELOCITY_KP, CHASSIS_VELOCITY_KI,
        CHASSIS_VELOCITY_KD, CHASSIS_MOTOR_FEEDFORWARD,
        CHASSIS_RACE_PID_OUTPUT_LIMIT,
        CHASSIS_RACE_PID_INTEGRAL_LIMIT, -1, -1
    },
    {
        0.1434926f,
        CHASSIS_VELOCITY_KP, CHASSIS_VELOCITY_KI,
        CHASSIS_VELOCITY_KD, CHASSIS_MOTOR_FEEDFORWARD,
        CHASSIS_RACE_PID_OUTPUT_LIMIT,
        CHASSIS_RACE_PID_INTEGRAL_LIMIT, -1, 1
    },
    214.2f,
    CHASSIS_SPEED_FILTER_ALPHA,
    400.0f, 2.0f, 500.0f, 25.0f, 2.0f, 1.0f, 30.0f,
    CHASSIS_HEADING_FUSION_TIME_CONSTANT_S,
    CHASSIS_HEADING_RATE_IMU_WEIGHT,
    CHASSIS_IMU_HEADING_SIGN,
    CHASSIS_IMU_MAX_DELTA_DEG,
    CHASSIS_HEADING_CONTROL_KP,
    CHASSIS_YAW_RATE_CONTROL_KP,
    CHASSIS_HEADING_CORRECTION_LIMIT_RATIO,
    CHASSIS_CONTROL_PERIOD_MS,
    CHASSIS_IMU_STALE_MS,
    3U, 8U
};
