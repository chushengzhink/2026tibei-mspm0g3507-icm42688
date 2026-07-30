#ifndef BALL_BALANCE_CONFIG_H
#define BALL_BALANCE_CONFIG_H

#include "ml_board.h"

#ifndef BALL_BALANCE_ALLOW_SEQUENCE
#define BALL_BALANCE_ALLOW_SEQUENCE       (0)
#endif

/* Dedicated H3 resources. Values below remain unverified until bench tests. */
#define BALL_VISION_UART                  UART2
#define BALL_VISION_UART_BAUD             (115200UL)
#define BALL_VISION_UART_PRIORITY         (1U)
#define BALL_SERVO_TIMER                  TIMA1
#define BALL_SERVO_CHANNEL                DL_TIMER_CC_1_INDEX
#define BALL_TIMEBASE_TIMER               TIMG6
#define BALL_TIMEBASE_PRIORITY            (2U)

#define BALL_SERVO_FREQUENCY_HZ           (50U)
#define BALL_SERVO_MINIMUM_US             (1300U)
#define BALL_SERVO_CENTER_US              (1500U)
#define BALL_SERVO_MAXIMUM_US             (1700U)
#define BALL_SERVO_MAX_SLEW_US_PER_S      (2000UL)
#define BALL_MANUAL_MAX_OFFSET_US         (50)

/* Verified 2026-07-30: 1450 us lowers the +cm side. */
#define BALL_CONTROL_DIRECTION            (-1.0f)

/* Unverified starting calibration for a 320 x 224 model input. */
#define BALL_IMAGE_WIDTH_PX               (320)
#define BALL_IMAGE_HEIGHT_PX              (224)
#define BALL_AXIS_NEGATIVE_X_PX           (20.0f)
#define BALL_AXIS_NEGATIVE_Y_PX           (112.0f)
#define BALL_AXIS_NEGATIVE_CM             (-12.0f)
#define BALL_AXIS_POSITIVE_X_PX           (300.0f)
#define BALL_AXIS_POSITIVE_Y_PX           (112.0f)
#define BALL_AXIS_POSITIVE_CM             (12.0f)
#define BALL_MEASUREMENT_MINIMUM_CM       (-12.5f)
#define BALL_MEASUREMENT_MAXIMUM_CM       (12.5f)

#define BALL_MINIMUM_SCORE                (0.40f)
#define BALL_VISION_TIMEOUT_MS            (150U)
#define BALL_REACQUIRE_FRAME_COUNT        (3U)
#define BALL_MAX_CAPTURE_INTERVAL_MS      (250U)
#define BALL_MAX_OBSERVER_RESIDUAL_CM     (4.0f)
#define BALL_OBSERVER_ALPHA               (0.65f)
#define BALL_OBSERVER_BETA                (0.12f)

/* Unverified controller starting values. Tune one class at a time. */
#define BALL_KP_US_PER_CM                 (40.0f)
#define BALL_KV_US_PER_CM_PER_S           (25.0f)
#define BALL_KI_US_PER_CM_S               (5.0f)
#define BALL_INTEGRAL_LIMIT_CM_S          (8.0f)
#define BALL_CONTROL_LIMIT_US             (200.0f)
#define BALL_TARGET_MINIMUM_CM            (-11.5f)
#define BALL_TARGET_MAXIMUM_CM            (11.5f)

#define BALL_SEQUENCE_PLUS_CM             (5.0f)
#define BALL_SEQUENCE_MINUS_CM            (-5.0f)
#define BALL_SEQUENCE_PLUS_ERROR_CM       (0.8f)
#define BALL_SEQUENCE_FINAL_ERROR_CM      (1.0f)
#define BALL_SEQUENCE_PLUS_SETTLE_MS      (200U)
#define BALL_SEQUENCE_FINAL_SETTLE_MS     (500U)
#define BALL_SEQUENCE_TIMEOUT_MS          (5000U)

#endif
