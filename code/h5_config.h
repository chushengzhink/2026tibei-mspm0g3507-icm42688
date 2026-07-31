#ifndef H5_CONFIG_H
#define H5_CONFIG_H

/* H question 5: independent vehicle + center-ball configuration. */
#define H5_TARGET_CM                         (0.0f)
#define H5_CRUISE_SPEED_MM_S                 (240.0f)
#define H5_CURVE_SPEED_MM_S                  (210.0f)
#define H5_ACCELERATION_MM_S2                (150.0f)
#define H5_PASS_PROGRESS_MM                  (5932.0f)
#define H5_PASS_GATE_ARM_PROGRESS_MM        (5580.0f)
#define H5_PASS_MAX_OVERRUN_MM               (50.0f)
#define H5_TIME_LIMIT_MS                     (30000U)
#define H5_CONTROL_PERIOD_MS                 (20U)
#define H5_TELEMETRY_PERIOD_MS               (50U)

#define H5_BALL_READY_ERROR_CM               (0.8f)
#define H5_BALL_READY_SPEED_CM_S             (1.0f)
#define H5_BALL_READY_SETTLE_MS              (500U)
#define H5_BALL_SCORE_ERROR_CM               (1.0f)
#define H5_BALL_SCORE_IMMEDIATE_CM           (1.2f)
#define H5_BALL_SCORE_CONFIRM_MS             (500U)
#define H5_BALL_ABORT_ERROR_CM               (2.0f)
#define H5_BALL_ABORT_HOLD_MS                (100U)
#define H5_LAUNCH_BIAS_US                    (60.0f)
#define H5_LAUNCH_BIAS_MS                    (4000U)

#define H5_LINE_CORRECTION_RATIO             (0.20f)
#define H5_LINE_CORRECTION_MAX_MM_S         (70.0f)
#define H5_LINE_OUTER_CORRECTION_RATIO       (0.28f)
#define H5_LINE_OUTER_CORRECTION_MAX_MM_S   (70.0f)
#define H5_LINE_CURVE_HOLD_RATIO             (0.18f)
#define H5_LINE_CURVE_HOLD_MAX_MM_S         (60.0f)

#define H5_HEADING_GATE_TARGET_DEG           (360.0f)
#define H5_HEADING_GATE_TOLERANCE_DEG       (5.0f)
#define H5_HEADING_GATE_CONFIRM_CYCLES      (3U)

#endif
