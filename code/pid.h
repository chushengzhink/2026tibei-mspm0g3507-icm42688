#ifndef PID_H
#define PID_H

#include <stdint.h>

#include "ml_common.h"

typedef enum {
    POSITION_PID = 0,
    DELTA_PID
} pid_mode_t;

typedef struct {
    float target;
    float now;
    float error[3];
    float p;
    float i;
    float d;
    float pout;
    float dout;
    float iout;
    float out;
    uint32_t pid_mode;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    bool initialized;
} pid_t;

ml_status_t pid_init(
    pid_t *pid, uint32_t mode, float p, float i, float d);
ml_status_t pid_reset(pid_t *pid);
ml_status_t pid_set_limits(pid_t *pid, float output_min, float output_max,
    float integral_min, float integral_max);
ml_status_t pid_cal(pid_t *pid);
ml_status_t pidout_limit(pid_t *pid);
ml_status_t pid_control(void);
void motor_target_set(int spe1, int spe2);

extern pid_t motorA;
extern pid_t motorB;

#endif
