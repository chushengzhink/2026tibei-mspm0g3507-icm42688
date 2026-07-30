#ifndef MOTOR_VELOCITY_H
#define MOTOR_VELOCITY_H

#include <stdint.h>

#include "ml_common.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float feedforward;
    float output_limit;
    float integral_limit;
} motor_velocity_wheel_config_t;

typedef struct {
    motor_velocity_wheel_config_t motor_a;
    motor_velocity_wheel_config_t motor_b;
    float filter_alpha;
} motor_velocity_config_t;

typedef struct {
    float filtered_a_ticks;
    float filtered_b_ticks;
    uint16_t duty_a_count;
    uint16_t duty_b_count;
} motor_velocity_measurement_t;

ml_status_t motor_velocity_init(const motor_velocity_config_t *config);
ml_status_t motor_velocity_reset(void);
ml_status_t motor_velocity_update(
    float target_a_ticks,
    float target_b_ticks,
    float actual_a_ticks,
    float actual_b_ticks,
    int8_t motor_a_sign,
    int8_t motor_b_sign);
ml_status_t motor_velocity_get_measurement(
    motor_velocity_measurement_t *measurement);

/* Compatibility entry points retained for older applications. */
ml_status_t pid_control(void);
void motor_target_set(int spe1, int spe2);

#endif
