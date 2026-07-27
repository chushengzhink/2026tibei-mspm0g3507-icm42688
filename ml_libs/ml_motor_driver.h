#ifndef ML_MOTOR_DRIVER_H
#define ML_MOTOR_DRIVER_H

#include <stdint.h>

#include "ml_common.h"

typedef enum {
    ML_MOTOR_A = 0,
    ML_MOTOR_B
} ml_motor_id_t;

ml_status_t ml_motor_driver_init(void);
ml_status_t ml_motor_driver_set_duty(
    ml_motor_id_t motor, int32_t duty);
ml_status_t ml_motor_driver_stop(ml_motor_id_t motor);
ml_status_t ml_motor_driver_stop_all(void);

extern uint8_t motorA_dir;
extern uint8_t motorB_dir;

#endif
