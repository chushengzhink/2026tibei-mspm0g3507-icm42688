#ifndef CHASSIS_SELF_TEST_VIEW_H
#define CHASSIS_SELF_TEST_VIEW_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_SELF_TEST_VIEW_COLUMNS (16U)

void chassis_self_test_view_encoder(
    uint8_t left_state, uint8_t right_state,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_bad(
    uint32_t left_bad, uint32_t right_bad,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_tick(
    char wheel, int32_t ticks,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_keys(
    uint8_t up, uint8_t left, uint8_t down, uint8_t center,
    uint8_t right,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_sw6(
    uint8_t channel_1, uint8_t channel_2, uint8_t channel_3,
    uint8_t channel_4, uint8_t channel_5,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_calibration(
    uint16_t samples,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_move(
    uint32_t gyro_tenths_dps, uint32_t accel_hundredths_g,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_status(
    uint32_t status,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_uart_divisor(
    uint32_t integer_divisor, uint32_t fractional_divisor,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_uart_tx(
    uint32_t banners, uint32_t timeouts,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_uart_rx(
    bool valid, uint8_t byte, uint8_t error_flags,
    uint32_t overflows,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_fusion_heading(
    float encoder_heading_deg, float fused_heading_deg,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_fusion_imu(
    float imu_yaw_deg, float fused_yaw_rate_dps,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);
void chassis_self_test_view_fusion_status(
    bool fusion_active, uint16_t sample_count,
    char line[CHASSIS_SELF_TEST_VIEW_COLUMNS + 1U]);

#endif
