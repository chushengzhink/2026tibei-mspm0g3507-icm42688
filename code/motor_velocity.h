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
} motor_velocity_config_t;

ml_status_t motor_velocity_init(
    const motor_velocity_config_t *config);
ml_status_t motor_velocity_reset(void);
ml_status_t motor_velocity_update(
    float target_a_ticks,
    float target_b_ticks,
    int32_t actual_a_ticks,
    int32_t actual_b_ticks,
    int8_t motor_a_sign,
    int8_t motor_b_sign);

#endif
