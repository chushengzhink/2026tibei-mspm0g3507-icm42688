#include "ml_quadrature.h"

#define ML_QUADRATURE_INVALID (2)

static const int8_t g_transition_table[16] = {
     0,  1, -1, ML_QUADRATURE_INVALID,
    -1,  0, ML_QUADRATURE_INVALID,  1,
     1, ML_QUADRATURE_INVALID,  0, -1,
    ML_QUADRATURE_INVALID, -1,  1,  0
};

static uint8_t quadrature_state(uint8_t phase_a, uint8_t phase_b)
{
    return (uint8_t) (((phase_a != 0U) ? 2U : 0U) |
        ((phase_b != 0U) ? 1U : 0U));
}

void ml_quadrature_init(
    ml_quadrature_t *decoder, uint8_t phase_a, uint8_t phase_b,
    int8_t polarity)
{
    if (decoder == 0) {
        return;
    }
    decoder->state = quadrature_state(phase_a, phase_b);
    decoder->polarity = (polarity < 0) ? -1 : 1;
    decoder->invalid_transitions = 0U;
    decoder->initialized = true;
}

int8_t ml_quadrature_update(
    ml_quadrature_t *decoder, uint8_t phase_a, uint8_t phase_b)
{
    uint8_t next_state;
    int8_t transition;

    if ((decoder == 0) || !decoder->initialized) {
        return 0;
    }
    next_state = quadrature_state(phase_a, phase_b);
    transition = g_transition_table[
        ((uint32_t) decoder->state << 2U) | next_state];
    decoder->state = next_state;
    if (transition == ML_QUADRATURE_INVALID) {
        if (decoder->invalid_transitions < UINT32_MAX) {
            ++decoder->invalid_transitions;
        }
        return 0;
    }
    return (int8_t) (transition * decoder->polarity);
}
