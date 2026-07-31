#include <assert.h>
#include <math.h>
#include <string.h>

#include "q3_ball_profile.h"

static void test_generated_profile(void)
{
    const q3_ball_profile_t *profile = q3_profile_get();
    q3_profile_sample_t sample;

    assert(q3_profile_validate(profile));
    assert(profile->axis_sign == -1 || profile->axis_sign == 1);
    assert(q3_profile_sample(profile, -20.0f, &sample));
    assert(sample.lower_index == 0U);
    assert(q3_profile_sample(profile, 20.0f, &sample));
    assert(sample.lower_index == Q3_PROFILE_POINT_COUNT - 1U);
    assert(q3_profile_sample(profile, 0.5f, &sample));
    assert(sample.lower_index == 6U);
    assert(sample.breakaway_plus_us >= sample.rolling_plus_us);
    assert(sample.breakaway_minus_us >= sample.rolling_minus_us);
}

static void test_interpolation_and_rejection(void)
{
    q3_ball_profile_t profile = *q3_profile_get();
    q3_profile_sample_t sample;

    profile.point[6].rolling_plus_us = 20.0f;
    profile.point[7].rolling_plus_us = 40.0f;
    assert(q3_profile_sample(&profile, 0.5f, &sample));
    assert(fabsf(sample.rolling_plus_us - 30.0f) < 0.01f);

    profile.point[7].breakaway_plus_us = 5.0f;
    assert(!q3_profile_validate(&profile));
    assert(!q3_profile_sample(&profile, 0.0f, &sample));
    assert(!q3_profile_sample(q3_profile_get(), 0.0f, 0));
}

int main(void)
{
    test_generated_profile();
    test_interpolation_and_rejection();
    return 0;
}
