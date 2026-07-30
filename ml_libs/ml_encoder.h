#ifndef ML_ENCODER_H
#define ML_ENCODER_H

#include <stdint.h>

#include "ml_common.h"

typedef enum {
    ML_ENCODER_A = 0,
    ML_ENCODER_B
} ml_encoder_id_t;

typedef struct {
    int32_t total_count_a;
    int32_t total_count_b;
    uint32_t invalid_transitions_a;
    uint32_t invalid_transitions_b;
    uint8_t state_a;
    uint8_t state_b;
    uint8_t live_state_a;
    uint8_t live_state_b;
    uint32_t phase_a_edges_a;
    uint32_t phase_b_edges_a;
    uint32_t phase_a_edges_b;
    uint32_t phase_b_edges_b;
} ml_encoder_diagnostics_t;

ml_status_t ml_encoder_init(void);
ml_status_t ml_encoder_read_and_clear(
    int32_t *count_a, int32_t *count_b);
ml_status_t ml_encoder_get_diagnostics(
    ml_encoder_diagnostics_t *diagnostics);

/* Kept public for source compatibility with the original driver. */
extern volatile int32_t Encoder_count1;
extern volatile int32_t Encoder_count2;

#endif
