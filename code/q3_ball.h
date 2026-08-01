#ifndef Q3_BALL_H
#define Q3_BALL_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"
#include "q3_ball_profile.h"

typedef enum {
    Q3_STATE_WAIT_VISION = 0,
    Q3_STATE_BOOT_SETTLE,
    Q3_STATE_BOOT_PROBE_PLUS,
    Q3_STATE_BOOT_RETURN_PLUS,
    Q3_STATE_BOOT_PROBE_MINUS,
    Q3_STATE_BOOT_RECENTER,
    Q3_STATE_READY,
    Q3_STATE_PLUS_DRIVE,
    Q3_STATE_PLUS_BRAKE,
    Q3_STATE_REVERSAL,
    Q3_STATE_MINUS_DRIVE,
    Q3_STATE_MINUS_BRAKE,
    Q3_STATE_FINAL_CAPTURE,
    Q3_STATE_COMPLETE,
    Q3_STATE_MAP_ARMED,
    Q3_STATE_MAP_TO_PLUS,
    Q3_STATE_MAP_TO_MINUS,
    Q3_STATE_MAP_RETURN_CENTER,
    Q3_STATE_MAP_COMPLETE,
    Q3_STATE_TIMEOUT,
    Q3_STATE_ABORTED,
    Q3_STATE_VISION_FAULT,
    Q3_STATE_PROFILE_FAULT,
    Q3_STATE_CALIBRATION_FAULT
} q3_state_t;

typedef enum {
    Q3_MODE_PROFILE = 0,
    Q3_MODE_WAVEFORM,
    Q3_MODE_MANUAL
} q3_mode_t;

#define Q3_MODE_ADAPTIVE Q3_MODE_PROFILE

typedef enum {
    Q3_RESCUE_NONE = 0,
    Q3_RESCUE_KICK,
    Q3_RESCUE_ROCK,
    Q3_RESCUE_BURST,
    Q3_RESCUE_HOLD
} q3_rescue_stage_t;

typedef enum {
    Q3_CORE_INIT_START = 0,
    Q3_CORE_INIT_CORE = Q3_CORE_INIT_START,
    Q3_CORE_INIT_UART2,
    Q3_CORE_INIT_SERVO,
    Q3_CORE_INIT_TIMG6,
    Q3_CORE_INIT_SAFE,
    Q3_CORE_INIT_COMPLETE
} q3_core_init_stage_t;

typedef void (*q3_core_init_progress_t)(q3_core_init_stage_t stage,
    void *context);

typedef enum {
    Q3_CAL_FAULT_NONE = 0,
    Q3_CAL_FAULT_BOOT_TIMEOUT,
    Q3_CAL_FAULT_POSITION_LIMIT,
    Q3_CAL_FAULT_PLUS_DIRECTION,
    Q3_CAL_FAULT_MINUS_DIRECTION,
    Q3_CAL_FAULT_RECENTER_TIMEOUT
} q3_cal_fault_reason_t;

typedef enum {
    Q3_VISION_DIAG_NO_RX = 0,
    Q3_VISION_DIAG_LOST,
    Q3_VISION_DIAG_LOW_SCORE,
    Q3_VISION_DIAG_SLOW_FRAME,
    Q3_VISION_DIAG_WAIT_STREAK,
    Q3_VISION_DIAG_MOVE_TO_O,
    Q3_VISION_DIAG_OK
} q3_vision_diag_t;

typedef struct {
    float position_cm;
    float balance_command_us;
    float rolling_plus_us;
    float rolling_minus_us;
    float breakaway_plus_us;
    float breakaway_minus_us;
    float acceleration_plus_cm_s2;
    float acceleration_minus_cm_s2;
    uint8_t valid_mask;
} q3_calibration_point_t;

typedef struct {
    q3_state_t state;
    q3_mode_t mode;
    q3_rescue_stage_t rescue_stage;
    q3_state_t cal_fault_state;
    q3_cal_fault_reason_t cal_fault_reason;
    q3_vision_diag_t vision_diag;
    bool initialized;
    bool vision_ready;
    bool profile_valid;
    bool sequence_started;
    bool sequence_completed;
    bool plus_captured;
    bool final_captured;
    bool final_capture_latched;
    bool brake_active;
    bool servo_settled;
    int8_t axis_sign;
    uint8_t profile_index;
    uint8_t rescue_attempts;
    uint8_t vision_valid_streak;
    float neutral_us;
    float response_scale;
    float target_cm;
    float position_cm;
    float velocity_cm_per_s;
    float error_cm;
    float control_output_us;
    float predicted_stop_cm;
    float stall_progress_cm;
    int16_t raw_center_x_px;
    int16_t raw_center_y_px;
    float raw_score;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    uint32_t uptime_ms;
    uint32_t state_elapsed_ms;
    uint32_t sequence_elapsed_ms;
    uint32_t vision_age_ms;
    uint32_t vision_frame_interval_ms;
    uint32_t vision_last_diag_interval_ms;
    uint32_t stall_elapsed_ms;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t format_errors;
    uint32_t observer_outliers;
    uint32_t uart_overflows;
} q3_ball_status_t;

ml_status_t q3_ball_init(void);
ml_status_t q3_ball_init_with_progress(q3_core_init_progress_t progress,
    void *context);
q3_core_init_stage_t q3_ball_get_init_stage(void);
void q3_ball_process(void);
ml_status_t q3_ball_start(void);
ml_status_t q3_ball_abort(void);
ml_status_t q3_ball_get_status(q3_ball_status_t *status);
ml_status_t q3_ball_set_mode(q3_mode_t mode);
ml_status_t q3_ball_set_manual_pulse(uint16_t pulse_us);
ml_status_t q3_ball_arm_map_calibration(void);
ml_status_t q3_ball_start_map_calibration(void);
uint8_t q3_ball_calibration_count(void);
ml_status_t q3_ball_get_calibration_point(uint8_t index,
    q3_calibration_point_t *point);

#endif
