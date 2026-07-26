#ifndef ML_MOTOR_H
#define ML_MOTOR_H

#include <stdint.h>

#include "ml_common.h"

ml_status_t motor_init(void);
ml_status_t motorA_duty(int32_t duty);
ml_status_t motorB_duty(int32_t duty);
ml_status_t encoder_init(void);
ml_status_t encoder_get_and_clear(int32_t *count1, int32_t *count2);

extern volatile int32_t Encoder_count1;
extern volatile int32_t Encoder_count2;
extern uint8_t motorA_dir;
extern uint8_t motorB_dir;

#endif
