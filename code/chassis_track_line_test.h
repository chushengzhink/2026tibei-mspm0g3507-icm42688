#ifndef CHASSIS_TRACK_LINE_TEST_H
#define CHASSIS_TRACK_LINE_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

#define CHASSIS_TRACK_LINE_TEST_SPEED_COUNT (3U)

typedef enum {
    CHASSIS_TRACK_LINE_TEST_READY = 0,
    CHASSIS_TRACK_LINE_TEST_RUNNING,
    CHASSIS_TRACK_LINE_TEST_BRAKING,
    CHASSIS_TRACK_LINE_TEST_COMPLETE
} chassis_track_line_test_state_t;

typedef struct {
    float speed_mm_s[CHASSIS_TRACK_LINE_TEST_SPEED_COUNT];
    float distance_mm;
    float acceleration_mm_s2;
    float stop_speed_mm_s;
    uint16_t control_period_ms;
    uint8_t stopped_cycles_required;
} chassis_track_line_test_config_t;

typedef struct {
    chassis_track_line_test_config_t config;
    chassis_track_line_test_state_t state;
    float start_distance_mm;
    float progress_mm;
    float commanded_speed_mm_s;
    uint32_t start_time_ms;
    uint32_t stop_time_ms;
    uint8_t speed_index;
    uint8_t stopped_cycles;
    bool initialized;
} chassis_track_line_test_t;

typedef struct {
    chassis_track_line_test_state_t state;
    float requested_speed_mm_s;
    float selected_speed_mm_s;
    float progress_mm;
    uint32_t elapsed_ms;
    uint8_t speed_index;
    bool can_start;
    bool command_stop;
    bool finished;
} chassis_track_line_test_output_t;

extern const chassis_track_line_test_config_t
    g_chassis_track_line_test_default_config;

ml_status_t chassis_track_line_test_init(
    chassis_track_line_test_t *test,
    const chassis_track_line_test_config_t *config);
void chassis_track_line_test_speed_up(chassis_track_line_test_t *test);
void chassis_track_line_test_speed_down(chassis_track_line_test_t *test);
bool chassis_track_line_test_can_start(
    const chassis_track_line_test_t *test);
ml_status_t chassis_track_line_test_start(
    chassis_track_line_test_t *test, float center_distance_mm,
    uint32_t now_ms);
ml_status_t chassis_track_line_test_update(
    chassis_track_line_test_t *test, float center_distance_mm,
    float measured_left_mm_s, float measured_right_mm_s,
    uint32_t now_ms, chassis_track_line_test_output_t *output);
#endif
