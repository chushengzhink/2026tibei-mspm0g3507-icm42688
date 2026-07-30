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

static chassis_track_line_control_config_t curve_memory_test_config(void)
{
    chassis_track_line_control_config_t config =
        g_chassis_track_line_control_line_only_config;

    config.curve_memory_enabled = true;
    return config;
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

    check(!g_chassis_track_line_control_default_config.
              curve_memory_enabled,
        "formal race configuration keeps curve memory disabled");
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
    float inner_error = 29.0f / 161.0f;
    float release_correction = 77.0f *
        ((1.2f * inner_error) +
         (0.20f * (inner_error - 1.0f)));

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 120.0f, 0.001f) &&
        control.last_valid_black_bits == 0x01U &&
        control.curve_memory_side == 0 &&
        near_value(control.curve_travel_mm, 0.0f, 0.001f),
        "B1 to B0 retains outer-left boost");

    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              release_correction, 0.001f) &&
        control.last_valid_black_bits == 0x02U,
        "B2 exits outer-left boost with derivative release damping");

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
              -release_correction, 0.001f) &&
        control.last_valid_black_bits == 0x04U,
        "B4 exits outer-right boost with derivative release damping");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        control.pending_cycles == 1U &&
        control.last_valid_black_bits == 0x04U,
        "first direct B1-to-B4 reversal is suppressed for one cycle");
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s < 0.0f &&
        control.pending_cycles == 0U,
        "second direct B4 sample confirms the reversed direction");
}

static void test_pd_damping_and_direct_reversal_confirmation(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t centered = sample_for(0x06U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t lost = sample_for(0x00U);
    float inner_error = 29.0f / 161.0f;
    float steady = 77.0f * 1.2f * inner_error;
    float center_damping = -77.0f * 0.20f * inner_error;
    float confirmed_right = -77.0f *
        ((1.2f * inner_error) + (0.20f * inner_error));

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_default_config);
    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, steady, 0.001f),
        "first valid B2 has steady P correction without derivative kick");

    (void) chassis_track_line_control_update(&control, &centered,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              center_damping, 0.001f),
        "B2 to centered adds a small opposite derivative damping pulse");
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s,
              confirmed_right, 0.001f),
        "centered to B4 is accepted immediately with symmetric PD");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        control.pending_cycles == 1U,
        "one direct B2-to-B4 sample is treated as a reversal glitch");
    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s > 0.0f &&
        control.pending_cycles == 0U,
        "returning to B2 cancels the pending reversal");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s < 0.0f && output.recovering &&
        control.pending_cycles == 0U &&
        control.accepted_side == -1,
        "B0 confirms and holds a pending direct right reversal");
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
            output.line_valid,
            "a new valid centroid immediately clears held recovery");
        (void) chassis_track_line_control_update(&control, &centered,
            350.0f, 0.0f, &output);
        check(near_value(output.correction_mm_s, 0.0f, 0.001f),
            "a steady centered sample clears the derivative pulse");
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

static void test_line_only_curve_memory_b0_taper(void)
{
    chassis_track_line_control_config_t config =
        curve_memory_test_config();
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t outer_right = sample_for(0x08U);
    line_sample_t lost = sample_for(0x00U);
    uint8_t cycle;

    (void) chassis_track_line_control_init(&control, &config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 120.0f, 0.001f) &&
        control.curve_memory_side == 1,
        "LF-only B1 immediately activates left curve memory");

    for (cycle = 0U; cycle < 30U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(output.lost_ms == 600U &&
        near_value(output.correction_mm_s, 120.0f, 0.001f),
        "remembered B0 keeps full outer boost through 600 ms");
    for (cycle = 0U; cycle < 15U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(output.lost_ms == 900U &&
        near_value(output.correction_mm_s, 114.25f, 0.001f),
        "remembered B0 tapers halfway at 900 ms");
    for (cycle = 0U; cycle < 15U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(output.lost_ms == 1200U &&
        near_value(output.correction_mm_s, 108.5f, 0.001f),
        "remembered B0 reaches the curve hold at 1200 ms");
    for (cycle = 0U; cycle < 100U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(near_value(output.correction_mm_s, 108.5f, 0.001f) &&
        near_value(output.left_mm_s, 241.5f, 0.001f) &&
        near_value(output.right_mm_s, 458.5f, 0.001f),
        "long remembered B0 holds the R500 curve bias indefinitely");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 60U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(near_value(output.correction_mm_s, -108.5f, 0.001f) &&
        control.curve_memory_side == -1,
        "B8 curve memory and long B0 hold mirror B1 exactly");
}

static void test_line_only_curve_memory_transitions(void)
{
    chassis_track_line_control_config_t config =
        curve_memory_test_config();
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t centered = sample_for(0x06U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t all_black = sample_for(0x0FU);
    line_sample_t lost = sample_for(0x00U);

    (void) chassis_track_line_control_init(&control, &config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_left,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s > 108.5f &&
        output.correction_mm_s <= 120.0f &&
        control.curve_memory_side == 1,
        "B1-B0-B2 combines curve hold with same-side PD residual");
    (void) chassis_track_line_control_update(&control, &centered,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 108.5f, 0.001f) &&
        control.curve_memory_side == 1,
        "B6 keeps the remembered curve centered between inner sensors");

    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s > 85.0f &&
        output.correction_mm_s < 95.0f &&
        control.curve_memory_side == 1,
        "an early opposite inner sample remains a curve PD residual");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        control.curve_memory_side == 1 &&
        control.pending_cycles == 1U,
        "first direct reversal suppresses both PD and old curve memory");
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s > 85.0f &&
        output.correction_mm_s < 95.0f &&
        control.curve_memory_side == 1 &&
        control.pending_cycles == 0U,
        "confirmed early opposite sensor cannot clear curve memory");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &all_black,
        350.0f, 0.0f, &output);
    check(control.curve_memory_side == 0,
        "B15 clears curve memory without changing controller state");
    (void) chassis_track_line_control_update(&control, &all_black,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f),
        "steady B15 returns to ordinary centered PD");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(control.curve_memory_side == 0 &&
        near_value(output.correction_mm_s, 0.0f, 0.001f),
        "initial B0 cannot invent curve memory");
}

static void test_line_only_curve_memory_speed_scaling(void)
{
    static const float speeds[5] = {
        60.0f, 120.0f, 200.0f, 280.0f, 350.0f
    };
    static const float boost[5] = {
        21.0f, 42.0f, 70.0f, 98.0f, 120.0f
    };
    static const float hold[5] = {
        18.6f, 37.2f, 62.0f, 86.8f, 108.5f
    };
    chassis_track_line_control_config_t config =
        curve_memory_test_config();
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t lost = sample_for(0x00U);
    uint8_t speed_index;
    uint8_t cycle;

    (void) chassis_track_line_control_init(&control, &config);
    for (speed_index = 0U; speed_index < 5U; ++speed_index) {
        chassis_track_line_control_reset(&control);
        (void) chassis_track_line_control_update(&control, &outer_left,
            speeds[speed_index], 0.0f, &output);
        check(near_value(output.correction_mm_s,
                  boost[speed_index], 0.001f),
            "each LF-only speed uses the configured outer boost ratio");
        for (cycle = 0U; cycle < 60U; ++cycle) {
            (void) chassis_track_line_control_update(&control, &lost,
                speeds[speed_index], 0.0f, &output);
        }
        check(near_value(output.correction_mm_s,
                  hold[speed_index], 0.001f) &&
            fabsf(output.left_mm_s) <= 500.001f &&
            fabsf(output.right_mm_s) <= 500.001f,
            "each LF-only speed reaches its proportional safe curve hold");
    }
}

static void test_line_only_failed_curve_sequence_replay(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t lost = sample_for(0x00U);
    uint16_t cycle;

    (void) chassis_track_line_control_init(&control,
        &g_chassis_track_line_control_line_only_config);
    check(!g_chassis_track_line_control_line_only_config.
              curve_memory_enabled,
        "LF-only production configuration disables curve memory");
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 120.0f, 0.001f) &&
        control.curve_memory_side == 0,
        "LF-only B1 keeps outer boost without latching curve memory");
    for (cycle = 0U; cycle < 200U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(near_value(output.correction_mm_s, 120.0f, 0.001f) &&
        near_value(control.curve_travel_mm, 0.0f, 0.001f),
        "long B0 holds the last B1 error without starting a travel gate");

    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(near_value(output.correction_mm_s, 0.0f, 0.001f) &&
        control.pending_cycles == 1U,
        "first B4 after remembered B1 is the suppressed reversal sample");
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s < 0.0f &&
        output.correction_mm_s >= -90.0f &&
        control.pending_cycles == 0U &&
        control.curve_memory_side == 0,
        "second B4 immediately takes ordinary negative PD control");
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(output.correction_mm_s < 0.0f &&
        output.correction_mm_s >= -90.0f &&
        control.curve_memory_side == 0,
        "B0 after confirmed B4 holds right correction, not positive curve memory");
}

static void test_line_only_curve_exit_confirmation(void)
{
    chassis_track_line_control_config_t config =
        curve_memory_test_config();
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t outer_right = sample_for(0x08U);
    line_sample_t inner_left = sample_for(0x02U);
    line_sample_t inner_right = sample_for(0x04U);
    line_sample_t all_black = sample_for(0x0FU);
    line_sample_t lost = sample_for(0x00U);
    uint16_t cycle;

    (void) chassis_track_line_control_init(&control, &config);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 172U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    check(near_value(control.curve_travel_mm, 1200.0f, 0.001f),
        "curve exit becomes eligible at 1200 mm requested travel");
    for (cycle = 0U; cycle < 5U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &inner_right,
            350.0f, 0.0f, &output);
    }
    check(control.curve_memory_side == 1 &&
        control.curve_exit_cycles == 4U,
        "four accepted opposite cycles cannot exit the remembered curve");
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    check(control.curve_memory_side == 0 &&
        control.curve_exit_cycles == 0U &&
        output.correction_mm_s < 0.0f,
        "fifth accepted opposite cycle exits to ordinary PD");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 172U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &inner_right,
        350.0f, 0.0f, &output);
    (void) chassis_track_line_control_update(&control, &lost,
        350.0f, 0.0f, &output);
    check(control.curve_memory_side == 1 &&
        control.curve_exit_cycles == 0U &&
        output.correction_mm_s > 0.0f,
        "B0 resets exit confirmation and cannot complete curve exit");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_right,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 172U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    for (cycle = 0U; cycle < 6U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &inner_left,
            350.0f, 0.0f, &output);
    }
    check(control.curve_memory_side == 0 &&
        output.correction_mm_s > 0.0f,
        "right curve exit guard mirrors the left curve exactly");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 172U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    for (cycle = 0U; cycle < 6U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &outer_right,
            350.0f, 0.0f, &output);
    }
    check(control.curve_memory_side == 1 &&
        control.curve_exit_cycles == 0U,
        "opposite outer single never acts as a curve exit signal");

    chassis_track_line_control_reset(&control);
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    for (cycle = 0U; cycle < 100U; ++cycle) {
        (void) chassis_track_line_control_update(&control, &lost,
            350.0f, 0.0f, &output);
    }
    (void) chassis_track_line_control_update(&control, &outer_left,
        350.0f, 0.0f, &output);
    check(control.curve_travel_mm > 700.0f &&
        control.curve_memory_side == 1,
        "repeated same-side B1 does not reset curve travel");
    (void) chassis_track_line_control_update(&control, &all_black,
        350.0f, 0.0f, &output);
    check(control.curve_memory_side == 0 &&
        near_value(control.curve_travel_mm, 0.0f, 0.001f),
        "B15 immediately clears curve memory and travel");
}

static void test_line_only_curve_exit_speed_scaling(void)
{
    static const float speeds[5] = {
        60.0f, 120.0f, 200.0f, 280.0f, 350.0f
    };
    static const uint16_t cycles_to_gate[5] = {
        1000U, 500U, 300U, 215U, 172U
    };
    chassis_track_line_control_config_t config =
        curve_memory_test_config();
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_output_t output;
    line_sample_t outer_left = sample_for(0x01U);
    line_sample_t lost = sample_for(0x00U);
    uint16_t cycle;
    uint8_t speed_index;

    (void) chassis_track_line_control_init(&control, &config);
    for (speed_index = 0U; speed_index < 5U; ++speed_index) {
        chassis_track_line_control_reset(&control);
        (void) chassis_track_line_control_update(&control, &outer_left,
            speeds[speed_index], 0.0f, &output);
        for (cycle = 1U;
             cycle < cycles_to_gate[speed_index]; ++cycle) {
            (void) chassis_track_line_control_update(&control, &lost,
                speeds[speed_index], 0.0f, &output);
        }
        check(control.curve_travel_mm < 1200.0f,
            "curve gate stays closed one cycle early at every speed");
        (void) chassis_track_line_control_update(&control, &lost,
            speeds[speed_index], 0.0f, &output);
        check(near_value(control.curve_travel_mm, 1200.0f, 0.001f),
            "curve gate opens by requested distance at every speed");
    }
}

static void test_invalid_input(void)
{
    chassis_track_line_control_t control = {0};
    chassis_track_line_control_config_t config =
        g_chassis_track_line_control_default_config;
    chassis_track_line_control_output_t output;
    line_sample_t sample = sample_for(0x06U);

    check(chassis_track_line_control_update(&control, &sample,
              120.0f, 0.0f, &output) == ML_STATUS_NOT_INITIALIZED,
        "update rejects uninitialized control");
    config.reverse_confirm_cycles = 0U;
    check(chassis_track_line_control_init(&control, &config) ==
          ML_STATUS_INVALID_ARGUMENT,
        "initialization rejects zero reversal confirmation cycles");
    config = g_chassis_track_line_control_default_config;
    config.curve_exit_minimum_travel_mm = 0.0f;
    check(chassis_track_line_control_init(&control, &config) ==
          ML_STATUS_INVALID_ARGUMENT,
        "initialization rejects a zero curve exit distance");
    config = g_chassis_track_line_control_default_config;
    config.curve_exit_confirm_cycles = 0U;
    check(chassis_track_line_control_init(&control, &config) ==
          ML_STATUS_INVALID_ARGUMENT,
        "initialization rejects zero curve exit confirmation cycles");
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
    test_pd_damping_and_direct_reversal_confirmation();
    test_b0_holds_each_last_error();
    test_initial_b0_and_counter_saturation();
    test_pid_history_is_not_reset_between_patterns();
    test_line_only_curve_memory_b0_taper();
    test_line_only_curve_memory_transitions();
    test_line_only_curve_memory_speed_scaling();
    test_line_only_failed_curve_sequence_replay();
    test_line_only_curve_exit_confirmation();
    test_line_only_curve_exit_speed_scaling();
    test_invalid_input();
    if (g_failures == 0) {
        puts("chassis track line control tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
