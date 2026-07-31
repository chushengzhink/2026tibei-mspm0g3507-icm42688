#ifndef Q3_BALL_PROFILE_H
#define Q3_BALL_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define Q3_PROFILE_POINT_COUNT (13U)
#define Q3_PROFILE_MINIMUM_CM  (-6.0f)
#define Q3_PROFILE_STEP_CM     (1.0f)
#define Q3_PROFILE_VERSION     (1UL)

typedef struct {
    float position_cm;
    float balance_command_us;
    float rolling_plus_us;
    float rolling_minus_us;
    float breakaway_plus_us;
    float breakaway_minus_us;
    float acceleration_plus_cm_s2;
    float acceleration_minus_cm_s2;
} q3_profile_point_t;

typedef struct {
    uint32_t version;
    int8_t axis_sign;
    float neutral_us;
    float probe_displacement_cm;
    q3_profile_point_t point[Q3_PROFILE_POINT_COUNT];
} q3_ball_profile_t;

typedef struct {
    uint8_t lower_index;
    float balance_command_us;
    float rolling_plus_us;
    float rolling_minus_us;
    float breakaway_plus_us;
    float breakaway_minus_us;
    float acceleration_plus_cm_s2;
    float acceleration_minus_cm_s2;
} q3_profile_sample_t;

const q3_ball_profile_t *q3_profile_get(void);
bool q3_profile_validate(const q3_ball_profile_t *profile);
bool q3_profile_sample(const q3_ball_profile_t *profile,
    float position_cm, q3_profile_sample_t *sample);

#endif
