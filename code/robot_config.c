#include "robot_config.h"

/*
 * MG310 theoretical starting point for the current 1x decoder:
 * 13 PPR * 20:1 gearbox = 260 phase-A falling edges per wheel revolution.
 * With the nominal 48 mm wheel, PI * 48 / 260 = 0.57999 mm/tick.
 * Replace the independent left/right values, sensor_to_axle_mm and all
 * correction gains with measurements from the finished car before scored runs.
 */
static const circle_calibration_point_t g_circle_points[] = {
    {300U, 1.0000f, 1.0000f},
    {350U, 1.0000f, 1.0000f},
    {400U, 1.0000f, 1.0000f},
    {450U, 1.0000f, 1.0000f},
    {500U, 1.0000f, 1.0000f},
    {550U, 1.0000f, 1.0000f},
    {600U, 1.0000f, 1.0000f}
};

const robot_calibration_t g_robot_calibration = {
    0.57999f,
    0.57999f,
    128.6f,
    100.0f,
    1.0000f,
    1.0000f,
    1,
    1,
    1,
    1,
    g_circle_points,
    (uint8_t) (sizeof(g_circle_points) / sizeof(g_circle_points[0]))
};

void robot_circle_gains(
    uint16_t radius_mm, float *left_gain, float *right_gain)
{
    uint8_t index;
    const circle_calibration_point_t *low;
    const circle_calibration_point_t *high;
    float fraction;

    if ((left_gain == 0) || (right_gain == 0) ||
        (g_robot_calibration.circle_point_count == 0U)) {
        return;
    }

    low = &g_robot_calibration.circle_points[0];
    high = low;
    if (radius_mm <= low->radius_mm) {
        *left_gain = low->left_distance_gain;
        *right_gain = low->right_distance_gain;
        return;
    }

    for (index = 1U;
         index < g_robot_calibration.circle_point_count; ++index) {
        high = &g_robot_calibration.circle_points[index];
        if (radius_mm <= high->radius_mm) {
            fraction = ((float) radius_mm - (float) low->radius_mm) /
                ((float) high->radius_mm - (float) low->radius_mm);
            *left_gain = low->left_distance_gain +
                ((high->left_distance_gain - low->left_distance_gain) *
                 fraction);
            *right_gain = low->right_distance_gain +
                ((high->right_distance_gain - low->right_distance_gain) *
                 fraction);
            return;
        }
        low = high;
    }

    *left_gain = low->left_distance_gain;
    *right_gain = low->right_distance_gain;
}
