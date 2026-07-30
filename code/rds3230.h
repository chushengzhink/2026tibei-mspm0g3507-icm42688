#ifndef RDS3230_H
#define RDS3230_H

#include "ml_board.h"

typedef struct {
    GPTIMER_Regs *timer;
    DL_TIMER_CC_INDEX channel;
    uint16_t frequency_hz;
    uint16_t minimum_us;
    uint16_t center_us;
    uint16_t maximum_us;
    uint16_t current_us;
    uint16_t target_us;
    uint32_t maximum_slew_us_per_s;
    uint32_t last_update_ms;
    bool time_initialized;
    bool initialized;
} rds3230_t;

ml_status_t rds3230_init(rds3230_t *servo, GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX channel, uint16_t frequency_hz,
    uint16_t minimum_us, uint16_t center_us, uint16_t maximum_us,
    uint32_t maximum_slew_us_per_s);
ml_status_t rds3230_set_target_us(rds3230_t *servo, uint16_t pulse_us);
ml_status_t rds3230_set_center(rds3230_t *servo);
ml_status_t rds3230_update(rds3230_t *servo, uint32_t now_ms);
uint16_t rds3230_get_current_us(const rds3230_t *servo);
uint16_t rds3230_get_target_us(const rds3230_t *servo);

#endif
