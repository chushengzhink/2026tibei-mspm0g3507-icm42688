#ifndef Q3_BALL_CONFIG_H
#define Q3_BALL_CONFIG_H

#include "ml_board.h"

/* Q3 deliberately has no chassis, encoder, IMU or line-sensor dependency. */
#define Q3_VISION_UART                     UART2
#define Q3_VISION_UART_BAUD                (115200UL)
#define Q3_VISION_UART_PRIORITY            (1U)
#define Q3_SERVO_TIMER                     TIMA1
#define Q3_SERVO_CHANNEL                   DL_TIMER_CC_1_INDEX
#define Q3_TIMEBASE_TIMER                  TIMG6
#define Q3_TIMEBASE_PRIORITY               (2U)

#define Q3_SERVO_FREQUENCY_HZ              (50U)
#define Q3_SERVO_MINIMUM_US                (1300U)
#define Q3_SERVO_SAFE_US                   (1500U)
#define Q3_SERVO_NEUTRAL_DEFAULT_US        (1525U)
#define Q3_SERVO_MAXIMUM_US                (1700U)
#define Q3_SERVO_MAX_SLEW_US_PER_S         (2000UL)

#define Q3_CONTROL_PERIOD_MS               (10U)
#define Q3_VISION_MINIMUM_SCORE            (0.20f)
#define Q3_VISION_SOFT_HOLD_MS             (60U)
#define Q3_VISION_TIMEOUT_MS               (150U)
#define Q3_VISION_REACQUIRE_FRAMES         (3U)
#define Q3_MAX_CAPTURE_INTERVAL_MS         (250U)
#define Q3_MAX_OBSERVER_RESIDUAL_CM        (4.0f)
#define Q3_OBSERVER_ALPHA                  (0.65f)
#define Q3_OBSERVER_BETA                   (0.12f)

#define Q3_BOOT_SETTLE_MS                  (250U)
#define Q3_BOOT_PROBE_MS                   (120U)
#define Q3_BOOT_RETURN_MS                  (160U)
#define Q3_BOOT_TIMEOUT_MS                 (4500U)
#define Q3_BOOT_POSITION_LIMIT_CM          (1.20f)
#define Q3_BOOT_DIRECTION_MINIMUM_CM       (0.08f)
#define Q3_BOOT_RESPONSE_SCALE_MINIMUM     (0.65f)
#define Q3_BOOT_RESPONSE_SCALE_MAXIMUM     (1.45f)
#define Q3_BOOT_NEUTRAL_TRIM_LIMIT_US      (15.0f)
#define Q3_READY_ERROR_CM                  (0.50f)
#define Q3_READY_SPEED_CM_S                (0.60f)
#define Q3_READY_CONFIRM_MS                (300U)

#define Q3_PLUS_BRAKE_TARGET_CM            (4.70f)
#define Q3_PLUS_VALID_MINIMUM_CM           (4.20f)
#define Q3_PLUS_VALID_MAXIMUM_CM           (5.80f)
#define Q3_MINUS_CONTROL_TARGET_CM         (-4.70f)
#define Q3_FINAL_HOLD_TARGET_CM            (-5.00f)
#define Q3_MINUS_VALID_MINIMUM_CM          (-5.80f)
#define Q3_MINUS_VALID_MAXIMUM_CM          (-4.20f)
#define Q3_FINAL_CAPTURE_SPEED_CM_S        (1.0f)
#define Q3_FINAL_CAPTURE_MS                (300U)
#define Q3_SEQUENCE_TIMEOUT_MS             (5000U)
#define Q3_REVERSAL_TIMEOUT_MS             (220U)
#define Q3_REVERSAL_EXIT_SPEED_CM_S        (0.60f)

#define Q3_DRIVE_POSITION_GAIN_US_PER_CM   (8.0f)
#define Q3_DRIVE_SPEED_GAIN_US_PER_CM_S    (7.0f)
#define Q3_PLUS_DRIVE_MIN_COMMAND_US       (120.0f)
#define Q3_PLUS_URGENT_RESCUE_POSITION_CM  (2.0f)
#define Q3_PLUS_URGENT_RESCUE_ELAPSED_MS   (1800U)
#define Q3_BRAKE_SPEED_GAIN_US_PER_CM_S    (10.0f)
#define Q3_CAPTURE_POSITION_GAIN_US_PER_CM (12.0f)
#define Q3_CAPTURE_SPEED_GAIN_US_PER_CM_S  (9.0f)
#define Q3_CAPTURE_INNER_ERROR_CM          (0.35f)
#define Q3_NORMAL_COMMAND_LIMIT_US         (175.0f)
#define Q3_CAPTURE_COMMAND_LIMIT_US        (75.0f)
#define Q3_FINAL_CAPTURE_ENTRY_SPEED_CM_S  (1.5f)
#define Q3_FINAL_REBRAKE_SPEED_CM_S        (2.0f)
#define Q3_FINAL_POSITION_GAIN_US_PER_CM   (18.0f)
#define Q3_FINAL_SPEED_GAIN_US_PER_CM_S    (18.0f)
#define Q3_FINAL_EDGE_MARGIN_CM            (0.25f)
#define Q3_FINAL_EDGE_BIAS_US              (35.0f)
#define Q3_FINAL_COMMAND_LIMIT_US          (140.0f)
#define Q3_FINAL_FUNNEL_COMMAND_LIMIT_US   (175.0f)
#define Q3_BRAKE_MARGIN_CM                 (0.20f)

#define Q3_STALL_ARM_MS                    (180U)
#define Q3_STALL_PROGRESS_CM               (0.12f)
#define Q3_STALL_SPEED_CM_S                (0.30f)
#define Q3_RESCUE_KICK_MS                  (120U)
#define Q3_RESCUE_ROCK_MS                  (50U)
#define Q3_RESCUE_BURST_MS                 (160U)
#define Q3_RESCUE_ROCK_LIMIT_CM            (0.25f)
#define Q3_RESCUE_MAXIMUM_ATTEMPTS         (2U)
#define Q3_RESCUE_KICK_MARGIN_US           (10.0f)

#define Q3_MAP_LIMIT_CM                    (6.0f)
#define Q3_MAP_TARGET_SPEED_CM_S           (1.20f)
#define Q3_MAP_CENTER_ERROR_CM             (0.40f)
#define Q3_MAP_CENTER_SPEED_CM_S           (0.60f)
#define Q3_MAP_CENTER_CONFIRM_MS           (300U)
#define Q3_MAP_TIMEOUT_MS                  (90000UL)
#define Q3_MAP_VALID_ROLL_SPEED_CM_S       (0.35f)

#define Q3_MEASUREMENT_MINIMUM_CM          (-12.5f)
#define Q3_MEASUREMENT_MAXIMUM_CM          (12.5f)

#define Q3_TELEMETRY_UART                  UART0
#define Q3_TELEMETRY_UART_BAUD             (115200UL)
#define Q3_TELEMETRY_UART_PRIORITY         (3U)
#define Q3_OLED_POWER_SETTLE_MS            (250U)

#if (Q3_SERVO_MINIMUM_US < 1300U) || (Q3_SERVO_MAXIMUM_US > 1700U)
#error Q3 servo bounds must remain within the verified 1300--1700 us range
#endif

#if (Q3_SERVO_SAFE_US < Q3_SERVO_MINIMUM_US) || \
    (Q3_SERVO_SAFE_US > Q3_SERVO_MAXIMUM_US)
#error Q3 safe pulse must stay inside the absolute servo range
#endif

#endif
