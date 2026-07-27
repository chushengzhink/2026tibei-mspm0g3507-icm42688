#ifndef ML_ENCODER_H
#define ML_ENCODER_H

#include <stdint.h>

#include "ml_common.h"

typedef enum {
    ML_ENCODER_A = 0,
    ML_ENCODER_B
} ml_encoder_id_t;

ml_status_t ml_encoder_init(void);
ml_status_t ml_encoder_read_and_clear(
    int32_t *count_a, int32_t *count_b);

/* Kept public for source compatibility with the original driver. */
extern volatile int32_t Encoder_count1;
extern volatile int32_t Encoder_count2;

#endif
