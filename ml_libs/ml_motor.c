#include "ml_motor.h"

ml_status_t motor_init(void)
{
    return ml_motor_driver_init();
}

ml_status_t motorA_duty(int32_t duty)
{
    return ml_motor_driver_set_duty(ML_MOTOR_A, duty);
}

ml_status_t motorB_duty(int32_t duty)
{
    return ml_motor_driver_set_duty(ML_MOTOR_B, duty);
}

ml_status_t encoder_init(void)
{
    return ml_encoder_init();
}

ml_status_t encoder_get_and_clear(int32_t *count1, int32_t *count2)
{
    return ml_encoder_read_and_clear(count1, count2);
}
