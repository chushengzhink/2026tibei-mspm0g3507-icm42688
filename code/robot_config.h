#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#define ROBOT_CONTROL_TICK_MS             (10U)
#define ROBOT_SPEED_TICK_DIVIDER          (2U)
#define ROBOT_HMI_UART                    (UART1)
#define ROBOT_HMI_BAUD                    (115200UL)
#define ROBOT_HMI_START_BYTE              (0x31U)
#define ROBOT_HMI_START_CR                (0x0DU)
#define ROBOT_HMI_START_LF                (0x0AU)
#define ROBOT_VISION_UART                 (UART2)
#define ROBOT_VISION_BAUD                 (115200UL)

#define ROBOT_APPROACH_SPEED_MM_S         (80.0f)
#define ROBOT_LINE_SPEED_MM_S             (120.0f)
#define ROBOT_TURN_SPEED_MM_S             (80.0f)
#define ROBOT_CIRCLE_SPEED_MM_S           (110.0f)
#define ROBOT_PARK_SPEED_MM_S             (80.0f)
#define ROBOT_MAX_APPROACH_MM             (450.0f)
#define ROBOT_RETURN_ENDPOINT_WINDOW_MM   (80.0f)

#define ROBOT_LINE_KP                     (1.50f)
#define ROBOT_LINE_KD                     (0.45f)
#define ROBOT_LINE_MAX_CORRECTION_MM_S    (55.0f)
#define ROBOT_CIRCLE_SYNC_KP              (220.0f)
#define ROBOT_CIRCLE_MAX_CORRECTION_MM_S  (25.0f)

#define ROBOT_VELOCITY_KP                 (2800.0f)
#define ROBOT_VELOCITY_KI                 (220.0f)
#define ROBOT_VELOCITY_KD                 (0.0f)
#define ROBOT_MOTOR_FEEDFORWARD           (9000.0f)

#define ROBOT_MISSION_TIMEOUT_TICKS       (17000UL)
#define ROBOT_VISION_TIMEOUT_TICKS        (800UL)

typedef struct {
    uint16_t radius_mm;
    float left_distance_gain;
    float right_distance_gain;
} circle_calibration_point_t;

typedef struct {
    float left_mm_per_tick;
    float right_mm_per_tick;
    float effective_track_mm;
    float sensor_to_axle_mm;
    float right_turn_gain;
    float left_turn_gain;
    int8_t left_encoder_sign;
    int8_t right_encoder_sign;
    int8_t left_motor_sign;
    int8_t right_motor_sign;
    const circle_calibration_point_t *circle_points;
    uint8_t circle_point_count;
} robot_calibration_t;

extern const robot_calibration_t g_robot_calibration;

void robot_circle_gains(
    uint16_t radius_mm, float *left_gain, float *right_gain);

#endif
