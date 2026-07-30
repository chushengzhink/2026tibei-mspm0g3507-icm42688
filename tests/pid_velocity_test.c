#include "motor_velocity.h"
#include "pid.h"

#include <math.h>
#include <stdio.h>

#include "ml_encoder.h"
#include "ml_motor_driver.h"

static int g_failures;
static int32_t g_duty[2];
static int32_t g_encoder_a;
static int32_t g_encoder_b;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

ml_status_t ml_motor_driver_set_duty(
    ml_motor_id_t motor, int32_t duty)
{
    if ((motor != ML_MOTOR_A) && (motor != ML_MOTOR_B)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    g_duty[motor] = duty;
    return ML_STATUS_OK;
}

ml_status_t ml_motor_driver_stop_all(void)
{
    g_duty[0] = 0;
    g_duty[1] = 0;
    return ML_STATUS_OK;
}

ml_status_t ml_encoder_read_and_clear(
    int32_t *count_a, int32_t *count_b)
{
    if ((count_a == 0) || (count_b == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *count_a = g_encoder_a;
    *count_b = g_encoder_b;
    g_encoder_a = 0;
    g_encoder_b = 0;
    return ML_STATUS_OK;
}

static void test_pid(void)
{
    pid_t pid = {0};

    check(pid_init(0, POSITION_PID, 1.0f, 0.0f, 0.0f) ==
        ML_STATUS_INVALID_ARGUMENT, "PID rejects null context");
    check(pid_init(&pid, POSITION_PID, 2.0f, 1.0f, 0.0f) ==
        ML_STATUS_OK, "position PID initializes");
    check(pid_set_limits(&pid, -10.0f, 10.0f, -3.0f, 3.0f) ==
        ML_STATUS_OK, "PID limits initialize");
    pid.target = 20.0f;
    pid.now = 0.0f;
    check(pid_cal(&pid) == ML_STATUS_OK && pid.out == 10.0f,
        "PID output saturates at configured maximum");
    check(pid.iout == 0.0f,
        "position PID anti-windup rejects saturated integral growth");
    check(pid_reset(&pid) == ML_STATUS_OK && pid.out == 0.0f &&
        pid.error[0] == 0.0f, "PID reset clears dynamic state");
    check(pid_set_limits(&pid, 1.0f, -1.0f, -1.0f, 1.0f) ==
        ML_STATUS_INVALID_ARGUMENT, "PID rejects reversed limits");
}

static void test_velocity(void)
{
    const motor_velocity_config_t config = {
        {100.0f, 0.0f, 0.0f, 1000.0f, 5000.0f, 2500.0f},
        {80.0f, 0.0f, 0.0f, 900.0f, 4500.0f, 2000.0f},
        0.35f
    };
    motor_velocity_config_t invalid = config;
    motor_velocity_measurement_t measurement;

    invalid.motor_a.output_limit = 20001.0f;
    check(motor_velocity_init(&invalid) == ML_STATUS_INVALID_ARGUMENT,
        "velocity control rejects limits above the motor safety cap");
    check(motor_velocity_init(&config) == ML_STATUS_OK,
        "velocity control initializes");
    check(motor_velocity_update(10.0f, 10.0f, 0.0f, 0.0f, 1, -1) ==
        ML_STATUS_OK && g_duty[0] > 0 && g_duty[1] < 0 &&
        motor_velocity_get_measurement(&measurement) == ML_STATUS_OK &&
        measurement.duty_a_count == (uint16_t) g_duty[0] &&
        measurement.duty_b_count == (uint16_t) -g_duty[1],
        "velocity update reports absolute applied duty after polarity");
    check(motor_velocity_update(10.0f, 10.0f, 10.0f, -10.0f, 1, 1) ==
        ML_STATUS_OK &&
        motor_velocity_get_measurement(&measurement) == ML_STATUS_OK &&
        fabsf(measurement.filtered_a_ticks - 3.5f) < 0.001f &&
        fabsf(measurement.filtered_b_ticks + 3.5f) < 0.001f,
        "velocity measurement applies independent 0.35 low-pass filters");
    motorA.iout = 100.0f;
    check(motor_velocity_update(-10.0f, 10.0f, 0.0f, 0.0f, 1, 1) ==
        ML_STATUS_OK && motorA.iout <= 0.0f,
        "wheel direction reversal clears the corresponding PID state");
    check(motor_velocity_update(0.0f, 0.0f, 5.0f, -5.0f, 1, 1) ==
        ML_STATUS_OK && g_duty[0] == 0 && g_duty[1] == 0 &&
        motor_velocity_get_measurement(&measurement) == ML_STATUS_OK &&
        measurement.duty_a_count == 0U &&
        measurement.duty_b_count == 0U,
        "zero targets reset controllers and reported duty");
    check(motor_velocity_update(NAN, 0.0f, 0.0f, 0.0f, 1, 1) ==
        ML_STATUS_INVALID_ARGUMENT && g_duty[0] == 0 && g_duty[1] == 0,
        "invalid targets fail safe");
    check(motor_velocity_reset() == ML_STATUS_OK,
        "velocity controller reset succeeds");

    motor_target_set(4, -4);
    g_encoder_a = 1;
    g_encoder_b = -1;
    check(pid_control() == ML_STATUS_OK,
        "legacy PID control wrapper remains operational");
}

static void test_self_test_and_race_limits(void)
{
    const motor_velocity_config_t self_test = {
        {700.0f, 0.0f, 0.0f, 6000.0f, 11500.0f, 5750.0f},
        {700.0f, 0.0f, 0.0f, 6000.0f, 11500.0f, 5750.0f},
        1.0f
    };
    const motor_velocity_config_t race = {
        {700.0f, 0.0f, 0.0f, 6000.0f, 14000.0f, 7000.0f},
        {700.0f, 0.0f, 0.0f, 6000.0f, 14000.0f, 7000.0f},
        1.0f
    };

    (void) motor_velocity_init(&self_test);
    (void) motor_velocity_update(1000.0f, 1000.0f,
        0.0f, 0.0f, 1, 1);
    check(g_duty[0] == 17500 && g_duty[1] == 17500,
        "default self-test PID plus feedforward remains at 35 percent");
    (void) motor_velocity_init(&race);
    (void) motor_velocity_update(1000.0f, 1000.0f,
        0.0f, 0.0f, 1, 1);
    check(g_duty[0] == 20000 && g_duty[1] == 20000,
        "race PID plus feedforward reaches but cannot exceed 40 percent");
}

int main(void)
{
    test_pid();
    test_velocity();
    test_self_test_and_race_limits();
    if (g_failures == 0) {
        printf("PASS: pid and motor velocity tests\n");
    }
    return g_failures == 0 ? 0 : 1;
}
