#include "chassis.h"
#include "chassis_config.h"
#include "chassis_telemetry.h"
#include "ml_board.h"
#include "ml_encoder.h"
#include "ml_motor_driver.h"
#include "ml_tim.h"
#include "motor_velocity.h"

#include <stdio.h>
#include <string.h>

static int g_failures;
static tim_callback_t g_timer_callback;
static void *g_timer_context;
static uint32_t g_motor_update_calls;
static uint32_t g_motor_stop_calls;
static ml_encoder_diagnostics_t g_encoder_diagnostics;

GPTIMER_Regs g_test_timg0;
volatile int32_t Encoder_count1;
volatile int32_t Encoder_count2;
uint8_t motorA_dir;
uint8_t motorB_dir;

static void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

ml_status_t ml_motor_driver_init(void)
{
    return ML_STATUS_OK;
}

ml_status_t ml_motor_driver_stop_all(void)
{
    ++g_motor_stop_calls;
    return ML_STATUS_OK;
}

ml_status_t ml_encoder_init(void)
{
    memset(&g_encoder_diagnostics, 0, sizeof(g_encoder_diagnostics));
    return ML_STATUS_OK;
}

ml_status_t ml_encoder_read_and_clear(
    int32_t *count_a, int32_t *count_b)
{
    if ((count_a == 0) || (count_b == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *count_a = 0;
    *count_b = 0;
    return ML_STATUS_OK;
}

ml_status_t ml_encoder_get_diagnostics(
    ml_encoder_diagnostics_t *diagnostics)
{
    if (diagnostics == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    *diagnostics = g_encoder_diagnostics;
    return ML_STATUS_OK;
}

ml_status_t motor_velocity_init(const motor_velocity_config_t *config)
{
    return (config != 0) ? ML_STATUS_OK : ML_STATUS_INVALID_ARGUMENT;
}

ml_status_t motor_velocity_reset(void)
{
    return ML_STATUS_OK;
}

ml_status_t motor_velocity_update(
    float target_a_ticks, float target_b_ticks,
    float actual_a_ticks, float actual_b_ticks,
    int8_t motor_a_sign, int8_t motor_b_sign)
{
    (void) target_a_ticks;
    (void) target_b_ticks;
    (void) actual_a_ticks;
    (void) actual_b_ticks;
    (void) motor_a_sign;
    (void) motor_b_sign;
    ++g_motor_update_calls;
    return ML_STATUS_OK;
}

ml_status_t motor_velocity_get_measurement(
    motor_velocity_measurement_t *measurement)
{
    if (measurement == 0) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    measurement->filtered_a_ticks = 0.0f;
    measurement->filtered_b_ticks = 0.0f;
    measurement->duty_a_count = 0U;
    measurement->duty_b_count = 0U;
    return ML_STATUS_OK;
}

ml_status_t tim_interrupt_ms_init_ex(GPTIMER_Regs *timer,
    uint32_t time_ms, uint8_t priority,
    tim_callback_t callback, void *context)
{
    (void) priority;
    if ((timer != TIMG0) || (time_ms == 0U) || (callback == 0)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }
    g_timer_callback = callback;
    g_timer_context = context;
    return ML_STATUS_OK;
}

static void run_control_cycle(uint32_t timestamp_ms)
{
    chassis_set_imu_sample(-0.02f * 60.0f,
        -60.0f, timestamp_ms, true);
    g_timer_callback(g_timer_context);
    chassis_poll();
}

int main(void)
{
    chassis_telemetry_record_t record;
    uint32_t cycle;

    check(chassis_idle_capture_start() == ML_STATUS_NOT_INITIALIZED,
        "capture cannot start before chassis initialization");
    check(chassis_init(&g_chassis_default_config) == ML_STATUS_OK,
        "chassis initializes with hardware stubs");
    check(!chassis_idle_capture_active() &&
        chassis_telemetry_count() == 0U,
        "capture and RAM start empty");
    check(chassis_idle_capture_start() == ML_STATUS_OK &&
        chassis_idle_capture_active(),
        "stopped chassis starts idle fusion capture");

    for (cycle = 1U; cycle <= 4U; ++cycle) {
        run_control_cycle(cycle * 20U);
    }
    check(chassis_telemetry_count() == 0U,
        "idle capture does not sample faster than 10 Hz");
    run_control_cycle(100U);
    check(chassis_telemetry_count() == 1U &&
        chassis_telemetry_get(0U, &record) == ML_STATUS_OK &&
        record.timestamp_ms == 100U &&
        record.fusion_active == 1U &&
        record.fused_yaw_rate_cdps > 0,
        "fresh IMU data produces the first 100 ms fusion record");
    check(g_motor_update_calls == 0U && g_motor_stop_calls > 0U,
        "idle capture keeps the motor controller stopped");

    for (cycle = 6U; cycle <= 10U; ++cycle) {
        run_control_cycle(cycle * 20U);
    }
    check(chassis_telemetry_count() == 2U,
        "idle capture records exactly once per 100 ms");

    chassis_idle_capture_stop();
    run_control_cycle(220U);
    check(!chassis_idle_capture_active() &&
        chassis_telemetry_count() == 2U,
        "stopping idle capture prevents further stopped samples");

    check(chassis_idle_capture_start() == ML_STATUS_OK,
        "capture can restart without clearing RAM");
    check(chassis_set_wheel_speed(60.0f, 60.0f) == ML_STATUS_OK &&
        !chassis_idle_capture_active(),
        "a motion command defensively disables idle capture");
    check(chassis_idle_capture_start() == ML_STATUS_BUSY,
        "idle capture is rejected while motion is running");
    run_control_cycle(240U);
    check(g_motor_update_calls == 1U,
        "normal motor updates remain enabled during motion");

    chassis_stop();
    check(!chassis_idle_capture_active(),
        "normal stop leaves idle capture disabled");
    check(chassis_idle_capture_start() == ML_STATUS_OK,
        "capture can start again after normal stop");
    chassis_emergency_stop();
    check(!chassis_idle_capture_active(),
        "emergency stop always disables idle capture");

    if (g_failures == 0) {
        puts("chassis idle capture tests passed");
    }
    return g_failures == 0 ? 0 : 1;
}
