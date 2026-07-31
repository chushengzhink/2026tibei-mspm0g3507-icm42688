#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

typedef struct {
    float effective_track_mm;
    float maximum_wheel_speed_mm_s;
    uint16_t control_period_ms;
} chassis_config_t;

extern const chassis_config_t g_chassis_race_config;

#endif
