#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_control.h"

typedef enum {
    MOTION_FIND_A_SEARCH = 0,
    MOTION_FIND_A_ON_LINE,
    MOTION_FIND_A_TO_AXLE
} motion_find_a_stage_t;

typedef enum {
    MOTION_LINE_END_SEARCH = 0,
    MOTION_LINE_END_TO_AXLE
} motion_line_end_stage_t;

typedef struct {
    float left_speed_mm_s;
    float right_speed_mm_s;
    int32_t left_delta_ticks;
    int32_t right_delta_ticks;
    bool apply_speeds;
} motion_engine_output_t;

typedef struct {
    const robot_calibration_t *calibration;
    motion_mode_t mode;
    motion_result_t result;
    motion_fault_t fault;
    volatile uint32_t uptime_ticks;
    float left_distance_mm;
    float right_distance_mm;
    float target_distance_mm;
    float command_speed_mm_s;
    float current_base_speed_mm_s;
    float last_line_error_mm;
    line_sample_t line;
    uint16_t left_stall_updates;
    uint16_t right_stall_updates;
    uint16_t line_lost_ticks;
    uint16_t circle_sync_updates;
    motion_find_a_stage_t find_stage;
    uint8_t transverse_samples;
    uint8_t clear_samples;
    float line_entry_mm;
    float line_exit_mm;
    bool turn_right;
    bool align_to_line;
    uint8_t line_align_samples;
    motion_line_end_stage_t line_end_stage;
    uint8_t line_end_samples;
    float line_end_expected_mm;
    float turn_target_mm;
    float circle_left_target_mm;
    float circle_right_target_mm;
    bool stop_requested;
} motion_engine_t;

ml_status_t motion_engine_init(motion_engine_t *engine,
    const robot_calibration_t *calibration, line_sample_t initial_line);
ml_status_t motion_engine_start_find_a(
    motion_engine_t *engine, float speed_mm_s);
ml_status_t motion_engine_start_straight(
    motion_engine_t *engine, float distance_mm, float speed_mm_s);
ml_status_t motion_engine_start_line(motion_engine_t *engine,
    float distance_mm, float speed_mm_s, line_sample_t line);
ml_status_t motion_engine_start_line_to_end(motion_engine_t *engine,
    float expected_distance_mm, float speed_mm_s, line_sample_t line);
ml_status_t motion_engine_start_turn90(
    motion_engine_t *engine, bool turn_right, bool align_to_line);
ml_status_t motion_engine_start_circle(
    motion_engine_t *engine, uint16_t radius_mm);
void motion_engine_line_tick(
    motion_engine_t *engine, line_sample_t line);
void motion_engine_velocity_tick(motion_engine_t *engine,
    int32_t count_a, int32_t count_b, motion_engine_output_t *output);
void motion_engine_fail(motion_engine_t *engine, motion_fault_t fault);
void motion_engine_stop(motion_engine_t *engine);
bool motion_engine_take_stop_request(motion_engine_t *engine);
motion_status_t motion_engine_get_status(const motion_engine_t *engine);
uint32_t motion_engine_get_uptime_ticks(const motion_engine_t *engine);

#endif
