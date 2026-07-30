#ifndef ML_QUADRATURE_H
#define ML_QUADRATURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t state;
    int8_t polarity;
    uint32_t invalid_transitions;
    bool initialized;
} ml_quadrature_t;

void ml_quadrature_init(
    ml_quadrature_t *decoder, uint8_t phase_a, uint8_t phase_b,
    int8_t polarity);
int8_t ml_quadrature_update(
    ml_quadrature_t *decoder, uint8_t phase_a, uint8_t phase_b);

#endif
