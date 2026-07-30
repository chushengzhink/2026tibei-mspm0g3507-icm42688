#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include "ml_common.h"

typedef enum {
    BALL_BALANCE_DISABLED = 0,
    BALL_BALANCE_MANUAL_SERVO,
    BALL_BALANCE_WAITING_FOR_VISION,
    BALL_BALANCE_ACTIVE,
    BALL_BALANCE_VISION_LOST
} ball_balance_state_t;

typedef enum {
    BALL_SEQUENCE_IDLE = 0,
    BALL_SEQUENCE_TO_PLUS_5_CM,
    BALL_SEQUENCE_TO_MINUS_5_CM,
    BALL_SEQUENCE_COMPLETE,
    BALL_SEQUENCE_TIMEOUT,
    BALL_SEQUENCE_ABORTED,
    BALL_SEQUENCE_VISION_LOST
} ball_balance_sequence_state_t;

typedef enum {
    BALL_CONTROL_DISABLED = 0,
    BALL_CONTROL_CASCADE,
    BALL_CONTROL_SPEED_TEST
} ball_balance_control_mode_t;

typedef struct {
    ball_balance_state_t state;
    ball_balance_sequence_state_t sequence_state;
    ball_balance_control_mode_t control_mode;
    bool enabled;
    bool vision_ready;
    bool sequence_started_once;
    float target_cm;
    float position_cm;
    float velocity_cm_per_s;
    float error_cm;
    float integral_cm_s;
    float target_velocity_cm_per_s;
    float speed_error_cm_per_s;
    float control_output_us;
    int16_t raw_center_x_px;
    int16_t raw_center_y_px;
    float raw_score;
    int16_t manual_servo_offset_us;
    uint16_t servo_current_us;
    uint16_t servo_target_us;
    uint32_t uptime_ms;
    uint32_t sequence_elapsed_ms;
    uint32_t vision_frame_interval_ms;
    uint32_t vision_age_ms;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t format_errors;
    uint32_t observer_outliers;
    uint32_t uart_overflows;
} ball_balance_status_t;

ml_status_t ball_balance_init(void);
void ball_balance_process(void);
ml_status_t ball_balance_enable(bool enable);
ml_status_t ball_balance_enable_speed_test(bool enable);
ml_status_t ball_balance_set_target_cm(float target_cm);
ml_status_t ball_balance_set_manual_servo_offset_us(int16_t offset_us);
ml_status_t ball_balance_start_pm5_sequence(void);
ml_status_t ball_balance_abort_sequence(void);
ml_status_t ball_balance_get_status(ball_balance_status_t *status);

#endif
