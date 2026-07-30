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

static float clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static float expected_correction(
    uint8_t black_bits, float error, float speed_mm_s)
{
    bool outer_single = (black_bits == 0x01U) ||
        (black_bits == 0x08U);
    float ratio = outer_single ?
        g_chassis_track_line_control_default_config.
            outer_single_correction_ratio :
        g_chassis_track_line_control_default_config.correction_ratio;
    float limit = outer_single ?
        g_chassis_track_line_control_default_config.
            outer_single_maximum_correction_mm_s :
        g_chassis_track_line_control_default_config.
            maximum_correction_mm_s;
    float magnitude = speed_mm_s * ratio;

    if (magnitude > limit) {
        magnitude = limit;
    }
    return clamp_unit(error *
        g_chassis_track_line_control_default_config.kp) * magnitude;
}

static line_sample_t sample_for(uint8_t black_bits)
{
    line_sample_t sample;

    sample.raw_bits = (uint8_t) ((~black_bits) & 0x0FU);
    sample.black_bits = black_bits;
    sample.lost = black_bits == 0U;
    sample.io_fault = false;
    return sample;
}

static void test_all_centroid_patterns(void)
{
    static const float expected_error[16] = {
        0.0f,
        1.0f,
        29.0f / 161.0f,
        95.0f / 161.0f,
        -29.0f / 161.0f,
        66.0f / 161.0f,
        0.0f,
        1.0f / 3.0f,
        -1.0f,
        0.0f,
        -66.0f / 161.0f,
        29.0f / 483.0f,
        -95.0f / 161.0f,
        -29.0f / 483.0f,
        -1.0f / 3.0f,
        0.0f
    };
    static const chassis_track_line_state_t expected_state[16] = {
        CHASSIS_TRACK_LINE_LOST,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_CENTERED,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_LEFT,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_RIGHT,
        CHASSIS_TRACK_LINE_RIGHT,
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
        check(output.line_state == expected_state[bits],
            "every LF04 pattern follows its physical centroid");
        check(near_value(output.correction_mm_s,
                  expected_correction(bits, expected_error[bits],
                      120.0f), 0.001f),
            "every LF04 pattern has the expected normalized correction");
        check(output.line_valid == (bits != 0U),
            "B1-B15 are valid and only B0 is lost");
        check(output.recovering == (bits == 0U),
            "only B0 enters held-error recovery");
    }
}

static void test_each_physical_sensor_strength(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t outer_right = sample_for(0x08U);
    float inner_correction = 26.4f *
        g_chassis_track_line_control_default_config.kp *
        29.0f / 161.0f;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 42.0f, 0.001f) &&
        near_value(output.left_mm_s, 78.0f, 0.001f) &&
        near_value(output.right_mm_s, 162.0f, 0.001f),
        "PA31 outer-left gives boosted left correction");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &inner_left,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              inner_correction, 0.001f) &&
        output.left_mm_s < output.right_mm_s,
        "PA12 inner-left gives a smaller left correction");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &inner_right,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              -inner_correction, 0.001f) &&
        output.left_mm_s > output.right_mm_s,
        "PB8 inner-right gives a smaller right correction");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, -42.0f, 0.001f) &&
        near_value(output.left_mm_s, 162.0f, 0.001f) &&
        near_value(output.right_mm_s, 78.0f, 0.001f),
        "PA27 outer-right gives boosted right correction");
}

static void test_speed_ratio_and_wheel_limit(void)
{
    static const float speeds[5] = {
        60.0f, 120.0f, 350.0f, 360.0f, 400.0f
    };
    static const float corrections[5] = {
        -21.0f, -42.0f, -120.0f, -120.0f, -115.384615f
    };
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_right = sample_for(0x08U);
    uint8_t index;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    for (index = 0U; index < 5U; ++index) {
        chassis_track_line_control_reset(&control);
        (void) chassis_track_line_control_update(&control, &outer_right,
            speeds[index], 0.0f, &output);
        check(near_value(output.correction_mm_s,
                  corrections[index], 0.001f),
            "outer correction uses 35 percent and the 120 mm/s cap");
    }

    check(near_value(output.left_mm_s, 500.0f, 0.001f) &&
        near_value(output.right_mm_s, 269.230774f, 0.001f),
        "400 mm/s outer correction preserves proportional wheel limiting");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        500.0f, -1.0f, &output);
    check(fabsf(output.left_mm_s) <= 500.001f &&
        fabsf(output.right_mm_s) <= 500.001f,
        "centroid and route command respects the wheel limit");
}

static void test_infrared_priority_over_route(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t outer_right = sample_for(0x08U);
    line_sample_t normal_right = sample_for(0x0CU);
    line_sample_t centered = sample_for(0x06U);

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &outer_right,
        360.0f, 0.72f, &output);
    check(near_value(output.left_mm_s, 480.0f, 0.001f) &&
        near_value(output.right_mm_s, 240.0f, 0.001f),
        "opposite route bias cannot reverse a right centroid command");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        360.0f, -0.72f, &output);
    check(near_value(output.left_mm_s, 480.0f, 0.001f) &&
        near_value(output.right_mm_s, 240.0f, 0.001f),
        "same-direction route bias caps boosted right at 120 mm/s");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        360.0f, -0.72f, &output);
    check(near_value(output.left_mm_s, 240.0f, 0.001f) &&
        near_value(output.right_mm_s, 480.0f, 0.001f),
        "opposite route bias cannot reverse a left centroid command");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &normal_right,
        360.0f, -0.72f, &output);
    check(near_value(output.left_mm_s, 450.0f, 0.001f) &&
        near_value(output.right_mm_s, 270.0f, 0.001f),
        "non-outer route reinforcement remains capped at 90 mm/s");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &centered,
        360.0f, 0.72f, &output);
    check(near_value(output.left_mm_s, 282.888f, 0.001f) &&
        near_value(output.right_mm_s, 437.112f, 0.001f),
        "zero centroid retains encoder and IMU route bias");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &centered,
        360.0f, 2.0f, &output);
    check(near_value(output.left_mm_s, 270.0f, 0.001f) &&
        near_value(output.right_mm_s, 450.0f, 0.001f),
        "zero centroid route bias remains capped at 90 mm/s");
}

static void test_outer_boost_memory_transitions(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t outer_right = sample_for(0x08U);
    line_sample_t lost = sample_for(0x00U);
    float inner_correction = expected_correction(
        0x02U, 29.0f / 161.0f, 350.0f);

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 120.0f, 0.001f) &&
        control.last_valid_black_bits == 0x01U,
        "B1 to B0 retains outer-left boost");

    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              inner_correction, 0.001f) &&
        control.last_valid_black_bits == 0x02U,
        "B2 immediately exits remembered outer-left boost");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, -120.0f, 0.001f) &&
        control.last_valid_black_bits == 0x08U,
        "B8 to B0 retains outer-right boost");

    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              -inner_correction, 0.001f) &&
        control.last_valid_black_bits == 0x04U,
        "B4 immediately exits remembered outer-right boost");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s < 0.0f &&
        control.last_valid_black_bits == 0x04U,
        "new B4 reverses a remembered B1 direction immediately");
}

static void test_b0_holds_each_last_error(void)
{
    static const uint8_t patterns[5] = {
        0x01U, 0x02U, 0x06U, 0x04U, 0x08U
    };
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t sample;
    line_sample_t lost = sample_for(0x00U);
    line_sample_t centered = sample_for(0x06U);
    float expected_correction;
    float expected_left;
    float expected_right;
    uint8_t index;
    uint8_t cycle;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    for (index = 0U; index < 5U; ++index) {
        chassis_track_line_control_reset(&control);
        sample = sample_for(patterns[index]);
        (void) chassis_track_line_control_update(&control, &sample,
            350.0f, 0.0f, &output);
        expected_correction = output.correction_mm_s;
        expected_left = output.left_mm_s;
        expected_right = output.right_mm_s;
        for (cycle = 0U; cycle < 20U; ++cycle) {
            (void) chassis_track_line_control_update(&control, &lost,
                350.0f, 0.0f, &output);
        }
        check(output.lost_ms == 400U && output.recovering &&
            !output.line_valid &&
            near_value(output.linear_mm_s, 350.0f, 0.001f) &&
            near_value(output.correction_mm_s,
                expected_correction, 0.001f) &&
            near_value(output.left_mm_s, expected_left, 0.001f) &&
            near_value(output.right_mm_s, expected_right, 0.001f),
            "B0 holds each last centroid at full task speed past 300 ms");

        (void) chassis_track_line_control_update(&control, &centered,
            350.0f, 0.0f, &output);
        check(output.lost_ms == 0U && !output.recovering &&
            output.line_valid &&
            near_value(output.correction_mm_s, 0.0f, 0.001f),
            "a new valid centroid immediately clears held recovery");
    }
}

static void test_initial_b0_and_counter_saturation(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t lost = sample_for(0x00U);

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(output.line_state == CHASSIS_TRACK_LINE_LOST &&
        near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        near_value(output.left_mm_s, 350.0f, 0.001f) &&
        near_value(output.right_mm_s, 350.0f, 0.001f),
        "startup B0 defaults to centered straight motion");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.4f, &output);
    check(near_value(output.left_mm_s, 307.16f, 0.001f) &&
        near_value(output.right_mm_s, 392.84f, 0.001f),
        "startup B0 in formal race retains route assistance");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    control.lost_ms = UINT16_MAX - 10U;
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(output.lost_ms == UINT16_MAX && output.recovering &&
        near_value(output.correction_mm_s, 120.0f, 0.001f),
        "indefinite B0 timing saturates without changing held correction");
}

static void test_pid_history_is_not_reset_between_patterns(void)
{
    chassis_track_line_control_config_t config =
        g_chassis_track_line_control_default_config;
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t lost = sample_for(0x00U);
    float inner_error = 29.0f / 161.0f;
    float expected_pid;

    config.kd = 0.15f;
    (void) chassis_track_line_control_init(&control, &config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        120.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_left,
        120.0f, 0.0f, &output);
    expected_pid = config.kp * inner_error +
        0.15f * (inner_error - 1.0f);
    check(near_value(output.correction_mm_s,
              expected_pid * 26.4f, 0.002f),
        "valid pattern changes preserve PID error history");

    (void) chassis_track_line_control_update(&control, &lost,
        120.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              config.kp * inner_error * 26.4f, 0.002f),
        "B0 continues PID with the held last centroid error");
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
    test_all_centroid_patterns();
    test_each_physical_sensor_strength();
    test_speed_ratio_and_wheel_limit();
    test_infrared_priority_over_route();
    test_outer_boost_memory_transitions();
    test_b0_holds_each_last_error();
    test_initial_b0_and_counter_saturation();
    test_pid_history_is_not_reset_between_patterns();
    test_invalid_input();
    if (g_failures == 0) {
        puts("chassis track line control tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
