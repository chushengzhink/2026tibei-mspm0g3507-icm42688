#include "q3_ball_profile.h"

#include <float.h>

#include "q3_ball_profile_generated.h"

static const q3_ball_profile_t g_profile = {
    Q3_PROFILE_VERSION,
    Q3_PROFILE_GENERATED_AXIS_SIGN,
    Q3_PROFILE_GENERATED_NEUTRAL_US,
    Q3_PROFILE_GENERATED_PROBE_CM,
    Q3_PROFILE_GENERATED_POINTS
};

static bool q3_profile_finite(float value)
{
    return (value == value) && (value >= -FLT_MAX) &&
        (value <= FLT_MAX);
}

static float q3_profile_lerp(float left, float right, float fraction)
{
    return left + (right - left) * fraction;
}

const q3_ball_profile_t *q3_profile_get(void)
{
    return &g_profile;
}

bool q3_profile_validate(const q3_ball_profile_t *profile)
{
    uint8_t index;

    if ((profile == 0) || (profile->version != Q3_PROFILE_VERSION) ||
        ((profile->axis_sign != -1) && (profile->axis_sign != 1)) ||
        !q3_profile_finite(profile->neutral_us) ||
        (profile->neutral_us < 1300.0f) ||
        (profile->neutral_us > 1700.0f) ||
        !q3_profile_finite(profile->probe_displacement_cm) ||
        (profile->probe_displacement_cm < 0.05f) ||
        (profile->probe_displacement_cm > 1.0f)) {
        return false;
    }
    for (index = 0U; index < Q3_PROFILE_POINT_COUNT; ++index) {
        const q3_profile_point_t *point = &profile->point[index];
        float expected = Q3_PROFILE_MINIMUM_CM +
            (float) index * Q3_PROFILE_STEP_CM;

        if (!q3_profile_finite(point->position_cm) ||
            ((point->position_cm - expected) > 0.01f) ||
            ((expected - point->position_cm) > 0.01f) ||
            !q3_profile_finite(point->balance_command_us) ||
            !q3_profile_finite(point->rolling_plus_us) ||
            !q3_profile_finite(point->rolling_minus_us) ||
            !q3_profile_finite(point->breakaway_plus_us) ||
            !q3_profile_finite(point->breakaway_minus_us) ||
            !q3_profile_finite(point->acceleration_plus_cm_s2) ||
            !q3_profile_finite(point->acceleration_minus_cm_s2) ||
            (point->balance_command_us < -60.0f) ||
            (point->balance_command_us > 60.0f) ||
            (point->rolling_plus_us < 5.0f) ||
            (point->rolling_minus_us < 5.0f) ||
            (point->breakaway_plus_us < point->rolling_plus_us) ||
            (point->breakaway_minus_us < point->rolling_minus_us) ||
            (point->breakaway_plus_us > 250.0f) ||
            (point->breakaway_minus_us > 250.0f) ||
            (point->acceleration_plus_cm_s2 < 1.0f) ||
            (point->acceleration_minus_cm_s2 < 1.0f) ||
            (point->acceleration_plus_cm_s2 > 80.0f) ||
            (point->acceleration_minus_cm_s2 > 80.0f)) {
            return false;
        }
    }
    return true;
}

bool q3_profile_sample(const q3_ball_profile_t *profile,
    float position_cm, q3_profile_sample_t *sample)
{
    float coordinate;
    float fraction;
    uint8_t lower;
    uint8_t upper;
    const q3_profile_point_t *left;
    const q3_profile_point_t *right;

    if ((sample == 0) || !q3_profile_validate(profile) ||
        !q3_profile_finite(position_cm)) {
        return false;
    }
    coordinate = (position_cm - Q3_PROFILE_MINIMUM_CM) /
        Q3_PROFILE_STEP_CM;
    if (coordinate <= 0.0f) {
        lower = 0U;
        upper = 0U;
        fraction = 0.0f;
    } else if (coordinate >= (float) (Q3_PROFILE_POINT_COUNT - 1U)) {
        lower = Q3_PROFILE_POINT_COUNT - 1U;
        upper = lower;
        fraction = 0.0f;
    } else {
        lower = (uint8_t) coordinate;
        upper = (uint8_t) (lower + 1U);
        fraction = coordinate - (float) lower;
    }
    left = &profile->point[lower];
    right = &profile->point[upper];
    sample->lower_index = lower;
    sample->balance_command_us = q3_profile_lerp(
        left->balance_command_us, right->balance_command_us, fraction);
    sample->rolling_plus_us = q3_profile_lerp(
        left->rolling_plus_us, right->rolling_plus_us, fraction);
    sample->rolling_minus_us = q3_profile_lerp(
        left->rolling_minus_us, right->rolling_minus_us, fraction);
    sample->breakaway_plus_us = q3_profile_lerp(
        left->breakaway_plus_us, right->breakaway_plus_us, fraction);
    sample->breakaway_minus_us = q3_profile_lerp(
        left->breakaway_minus_us, right->breakaway_minus_us, fraction);
    sample->acceleration_plus_cm_s2 = q3_profile_lerp(
        left->acceleration_plus_cm_s2,
        right->acceleration_plus_cm_s2, fraction);
    sample->acceleration_minus_cm_s2 = q3_profile_lerp(
        left->acceleration_minus_cm_s2,
        right->acceleration_minus_cm_s2, fraction);
    return true;
}
