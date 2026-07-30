#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "ml_quadrature.h"

static void test_all_transitions(void)
{
    static const int8_t expected[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    uint8_t from;
    uint8_t to;

    for (from = 0U; from < 4U; ++from) {
        for (to = 0U; to < 4U; ++to) {
            ml_quadrature_t decoder;
            uint32_t invalid_expected =
                (((from ^ to) == 3U) ? 1U : 0U);
            int8_t delta;

            ml_quadrature_init(&decoder,
                (uint8_t) ((from >> 1U) & 1U),
                (uint8_t) (from & 1U), 1);
            delta = ml_quadrature_update(&decoder,
                (uint8_t) ((to >> 1U) & 1U),
                (uint8_t) (to & 1U));
            assert(delta == expected[(from << 2U) | to]);
            assert(decoder.invalid_transitions == invalid_expected);
        }
    }
}

static void test_complete_cycles(void)
{
    static const uint8_t forward[] = {1U, 3U, 2U, 0U};
    static const uint8_t reverse[] = {2U, 3U, 1U, 0U};
    ml_quadrature_t decoder;
    int32_t count = 0;
    uint8_t index;

    ml_quadrature_init(&decoder, 0U, 0U, 1);
    for (index = 0U; index < 4U; ++index) {
        count += ml_quadrature_update(&decoder,
            (uint8_t) ((forward[index] >> 1U) & 1U),
            (uint8_t) (forward[index] & 1U));
    }
    assert(count == 4);
    for (index = 0U; index < 4U; ++index) {
        count += ml_quadrature_update(&decoder,
            (uint8_t) ((reverse[index] >> 1U) & 1U),
            (uint8_t) (reverse[index] & 1U));
    }
    assert(count == 0);

    ml_quadrature_init(&decoder, 0U, 0U, -1);
    assert(ml_quadrature_update(&decoder, 0U, 1U) == -1);
}

int main(void)
{
    test_all_transitions();
    test_complete_cycles();
    puts("quadrature tests passed");
    return 0;
}
