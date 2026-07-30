#include "chassis_track_line_control.h"

#include <math.h>
#include <stdio.h>

static int g_failures;

static void check(int condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static int near_value(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static line_sample_t sample_for(uint8_t black_bits)
{
    line_sample_t sample;

    sample.raw_bits = (uint8_t) ((~black_bits) & 0x0FU);
    sample.black_bits = black_bits;
    sample.left_on =
        (black_bits & LINE_SENSOR_LEFT_GROUP_MASK) != 0U;
    sample.right_on =
        (black_bits & LINE_SENSOR_RIGHT_GROUP_MASK) != 0U;
    sample.lost = !sample.left_on && !sample.right_on;
    sample.io_fault = false;
    return sample;
}

static void test_all_group_patterns(void)
{
    static const chassis_track_line_state_t expected[16] = {
        CHASSIS_TRACK_LINE_LOST,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_CENTERED
    };
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t sample;
    uint8_t bits;

    check(chassis_track_line_control_init(&control,
              &g_chassis_track_line_control_default_config) ==
          ML_STATUS_OK,
        "line control initializes");
    for (bits = 0U; bits < 16U; ++bits) {
        chassis_track_line_control_reset(&control);
        sample = sample_for(bits);
        check(chassis_track_line_control_update(&control, &sample,
                  120.0f, 0.0f, &output) == ML_STATUS_OK,
            "every LF04 pattern updates");
        check(output.line_state == expected[bits],
            "every LF04 pattern follows two-group classification");
        check(output.line_valid == (bits != 0U),
            "B1-B15 are valid and only B0 is lost");
        check(output.recovering == (bits == 0U),
            "only B0 enters recovery");
    }
}

static void test_immediate_three_state_control(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t left = sample_for(0x02U);
    line_sample_t right = sample_for(0x04U);
    line_sample_t centered = sample_for(0x06U);

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &left,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 26.4f, 0.001f) &&
        near_value(output.left_mm_s, 93.6f, 0.001f) &&
        near_value(output.right_mm_s, 146.4f, 0.001f),
        "left group slows left and accelerates right");

    (void) chassis_track_line_control_update(&control, &right,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, -26.4f, 0.001f) &&
        near_value(output.left_mm_s, 146.4f, 0.001f) &&
        near_value(output.right_mm_s, 93.6f, 0.001f),
        "right group accelerates left and slows right");

    (void) chassis_track_line_control_update(&control, &centered,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        near_value(output.left_mm_s, 120.0f, 0.001f) &&
        near_value(output.right_mm_s, 120.0f, 0.001f),
        "both groups clear correction immediately");
}

static void test_each_physical_sensor_direction(void)
{
    static const uint8_t left_bits[2] = {0x01U, 0x02U};
    static const uint8_t right_bits[2] = {0x04U, 0x08U};
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t sample;
    uint8_t index;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    for (index = 0U; index < 2U; ++index) {
        chassis_track_line_control_reset(&control);
        sample = sample_for(left_bits[index]);
        (void) chassis_track_line_control_update(&control, &sample,
            120.0f, 0.0f, &output);
        check(output.left_mm_s < output.right_mm_s,
            "PA31/PA12 low makes the left wheel slower");

        chassis_track_line_control_reset(&control);
        sample = sample_for(right_bits[index]);
        (void) chassis_track_line_control_update(&control, &sample,
            120.0f, 0.0f, &output);
        check(output.left_mm_s > output.right_mm_s,
            "PB8/PA27 low makes the right wheel slower");
    }
}

static void test_speed_ratio_and_wheel_limit(void)
{
    static const float speeds[4] = {60.0f, 120.0f, 360.0f, 400.0f};
    static const float corrections[4] = {
        -13.2f, -26.4f, -79.2f, -88.0f
    };
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t right = sample_for(0x08U);
    uint8_t index;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &right,
        8.0f, 0.0f, &output);
    check(near_value(output.left_mm_s, 9.76f, 0.001f) &&
        near_value(output.right_mm_s, 6.24f, 0.001f),
        "8 mm/s startup keeps both wheels moving");

    for (index = 0U; index < 4U; ++index) {
        chassis_track_line_control_reset(&control);
        (void) chassis_track_line_control_update(&control, &right,
            speeds[index], 0.0f, &output);
        check(near_value(output.correction_mm_s,
                  corrections[index], 0.001f),
            "P correction is 22 percent of speed and capped at 90 mm/s");
    }
}

static void test_infrared_priority_over_route(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t left = sample_for(0x02U);
    line_sample_t right = sample_for(0x04U);
    line_sample_t centered = sample_for(0x06U);

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &right,
        360.0f, 0.72f, &output);
    check(near_value(output.left_mm_s, 439.2f, 0.001f) &&
        near_value(output.right_mm_s, 280.8f, 0.001f),
        "opposite route bias cannot reverse a right infrared command");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &right,
        360.0f, -0.72f, &output);
    check(near_value(output.left_mm_s, 450.0f, 0.001f) &&
        near_value(output.right_mm_s, 270.0f, 0.001f),
        "same-direction route bias reinforces right only to 90 mm/s");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &left,
        360.0f, -0.72f, &output);
    check(near_value(output.left_mm_s, 280.8f, 0.001f) &&
        near_value(output.right_mm_s, 439.2f, 0.001f),
        "opposite route bias cannot reverse a left infrared command");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &left,
        360.0f, 0.72f, &output);
    check(near_value(output.left_mm_s, 270.0f, 0.001f) &&
        near_value(output.right_mm_s, 450.0f, 0.001f),
        "same-direction route bias reinforces left only to 90 mm/s");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &centered,
        360.0f, 0.72f, &output);
    check(near_value(output.left_mm_s, 282.888f, 0.001f) &&
        near_value(output.right_mm_s, 437.112f, 0.001f),
        "centered infrared retains the encoder and IMU route bias");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &right,
        500.0f, 1.0f, &output);
    check(fabsf(output.left_mm_s) <= 500.001f &&
        fabsf(output.right_mm_s) <= 500.001f,
        "infrared-priority command respects the wheel limit");
}

static void test_lost_line_recovery(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t right = sample_for(0x04U);
    line_sample_t centered = sample_for(0x06U);
    line_sample_t lost = sample_for(0x00U);
    uint8_t cycle;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &right,
        360.0f, 0.72f, &output);
    for (cycle = 0U; cycle < 14U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            360.0f, 0.72f, &output);
    }
    check(output.lost_ms == 280U && output.recovering &&
        !output.lost_fault &&
        near_value(output.linear_mm_s, 120.0f, 0.001f) &&
        near_value(output.angular_rad_s, -0.2464986f, 0.001f) &&
        near_value(output.correction_mm_s, -26.4f, 0.001f),
        "B0 limits speed and keeps the last infrared direction authoritative");

    (void) chassis_track_line_control_update(&control, &centered,
        360.0f, 0.72f, &output);
    check(output.lost_ms == 0U && !output.recovering &&
        !output.lost_fault &&
        near_value(output.correction_mm_s, 0.0f, 0.001f),
        "valid line clears recovery before timeout");

    (void) chassis_track_line_control_update(&control, &lost,
        360.0f, 0.72f, &output);
    check(near_value(output.linear_mm_s, 120.0f, 0.001f) &&
        near_value(output.angular_rad_s, 0.24f, 0.001f) &&
        near_value(output.correction_mm_s, 0.0f, 0.001f),
        "B0 after centered follows route without guessing a side");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &right,
        360.0f, 0.72f, &output);
    for (cycle = 0U; cycle < 14U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            360.0f, 0.72f, &output);
    }
    check(output.lost_ms == 280U && !output.lost_fault,
        "B0 remains recoverable through 280 ms");
    (void) chassis_track_line_control_update(&control, &lost,
        360.0f, 0.72f, &output);
    check(output.lost_ms == 300U && output.lost_fault,
        "B0 latches line loss exactly at 300 ms");
}

static void test_invalid_input(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t sample = sample_for(0x06U);

    check(chassis_track_line_control_update(&control, &sample,
              120.0f, 0.0f, &output) == ML_STATUS_NOT_INITIALIZED,
        "update rejects uninitialized control");
    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    sample.io_fault = true;
    check(chassis_track_line_control_update(&control, &sample,
              120.0f, 0.0f, &output) == ML_STATUS_INVALID_ARGUMENT,
        "update rejects GPIO fault samples");
}

int main(void)
{
    test_all_group_patterns();
    test_immediate_three_state_control();
    test_each_physical_sensor_direction();
    test_speed_ratio_and_wheel_limit();
    test_infrared_priority_over_route();
    test_lost_line_recovery();
    test_invalid_input();
    if (g_failures == 0) {
        puts("chassis track line control tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
