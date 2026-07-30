#include "chassis_track_app.h"

#include <stdio.h>
#include <string.h>

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_key.h"
#include "chassis_telemetry.h"
#include "chassis_telemetry_uart.h"
#include "chassis_track_line_control.h"
#include "chassis_track_line_test.h"
#include "chassis_track_mission.h"
#include "icm42688_service.h"
#include "line_sensor.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_oled.h"
#include "ml_uart.h"

#define TRACK_DISPLAY_PERIOD_MS (100U)
#define TRACK_UART_BAUD          (115200UL)
#define TRACK_DIAGNOSTIC_PAGE_MS (1000U)
#define TRACK_UART_BUSY_PERIOD_MS (1000U)

typedef enum {
    TRACK_APP_CALIBRATING = 0,
    TRACK_APP_READY,
    TRACK_APP_RUNNING,
    TRACK_APP_FINISHED,
    TRACK_APP_FAULT
} track_app_state_t;

typedef struct {
    icm42688_service_t imu;
    icm42688_service_output_t imu_output;
    chassis_track_mission_t mission;
    chassis_track_output_t output;
    chassis_track_line_control_t line_control;
    chassis_track_line_control_output_t line_control_output;
    chassis_track_line_test_t line_test;
    chassis_track_line_test_output_t line_test_output;
    chassis_key_t center_key;
    chassis_key_t up_key;
    chassis_key_t down_key;
    track_app_state_t state;
    uint32_t last_control_ms;
    uint32_t last_display_ms;
    ml_status_t fault_status;
    chassis_fault_t chassis_fault;
    line_sample_t last_line;
    line_sensor_white_guard_t white_guard;
    uint32_t last_white_sample_ms;
    uint32_t last_ready_sample_ms;
    uint32_t last_uart_busy_ms;
    bool imu_calibration_complete;
    bool velocity_started;
    bool braking_capture_started;
    bool line_gpio_fault;
    bool line_only_mode;
    bool uart_busy_sent;
    bool initialized;
} track_app_context_t;

static const icm42688_service_config_t g_track_imu_config = {
    {
        {
            IMU_ATTITUDE_AXIS_X,
            IMU_ATTITUDE_AXIS_Y,
            IMU_ATTITUDE_AXIS_Z
        },
        {-1, 1, -1}
    },
    TIMG8,
    1U
};

static track_app_context_t g_track_app;

static bool track_center_pressed(void)
{
    return gpio_get(ML_KEY_CENTER_PORT,
        ML_KEY_CENTER_PIN) == ML_KEY_ACTIVE_LEVEL;
}

static bool track_up_pressed(void)
{
    return gpio_get(ML_KEY_UP_PORT,
        ML_KEY_UP_PIN) == ML_KEY_ACTIVE_LEVEL;
}

static bool track_down_pressed(void)
{
    return gpio_get(ML_KEY_DOWN_PORT,
        ML_KEY_DOWN_PIN) == ML_KEY_ACTIVE_LEVEL;
}

static bool track_line_only_stopped_page(void)
{
    return g_track_app.line_only_mode &&
        ((g_track_app.state == TRACK_APP_READY) ||
         (g_track_app.state == TRACK_APP_FINISHED));
}

static void track_show_line(uint8_t line, const char *text)
{
    char padded[OLED_TEXT_COLUMN_COUNT + 1U];
    uint8_t index = 0U;

    while ((index < OLED_TEXT_COLUMN_COUNT) && (text[index] != '\0')) {
        padded[index] = text[index];
        ++index;
    }
    while (index < OLED_TEXT_COLUMN_COUNT) {
        padded[index++] = ' ';
    }
    padded[OLED_TEXT_COLUMN_COUNT] = '\0';
    (void) OLED_ShowLine(line, padded);
}

static void track_show(const chassis_status_t *status)
{
    char line[OLED_TEXT_COLUMN_COUNT + 1U];
    int32_t time_cs;
    int32_t error_mm;

    if (g_track_app.state == TRACK_APP_CALIBRATING) {
        if (g_track_app.imu_calibration_complete &&
            (g_track_app.white_guard.consecutive_samples <
             LINE_SENSOR_WHITE_STABLE_SAMPLES)) {
            track_show_line(1U, g_track_app.line_only_mode ?
                "LF ONLY WHITE" : "LF WHITE WAIT");
            (void) snprintf(line, sizeof(line), "NEED RF GOT R%X",
                (unsigned int) g_track_app.last_line.raw_bits);
            track_show_line(2U, line);
            (void) snprintf(line, sizeof(line), "STABLE %02u/%02u",
                (unsigned int) g_track_app.white_guard.consecutive_samples,
                (unsigned int) LINE_SENSOR_WHITE_STABLE_SAMPLES);
            track_show_line(3U, line);
            track_show_line(4U, "MOTORS LOCKED");
        } else {
            track_show_line(1U, g_track_app.line_only_mode ?
                "LF ONLY CAL" : "WHITE + IMU CAL");
            (void) snprintf(line, sizeof(line), "LF R%X N%02u/%02u",
                (unsigned int) g_track_app.last_line.raw_bits,
                (unsigned int) g_track_app.white_guard.consecutive_samples,
                (unsigned int) LINE_SENSOR_WHITE_STABLE_SAMPLES);
            track_show_line(2U, line);
            (void) snprintf(line, sizeof(line), "IMU N%04u",
                (unsigned int) g_track_app.imu_output.calibration_samples);
            track_show_line(3U, line);
            track_show_line(4U, "KEEP CAR STILL");
        }
    } else if (g_track_app.state == TRACK_APP_READY) {
        track_show_line(1U, g_track_app.line_only_mode ?
            "LF ONLY READY" : "READY PRESS C");
        if (g_track_app.line_only_mode) {
            (void) snprintf(line, sizeof(line), "S%03u NO LIMIT",
                (unsigned int) g_track_app.line_test_output.
                    selected_speed_mm_s);
            track_show_line(2U, line);
        } else {
            track_show_line(2U, "LF VIEW ONLY");
        }
        (void) snprintf(line, sizeof(line), "R%X W%X B%X",
            (unsigned int) g_track_app.last_line.raw_bits,
            (unsigned int) line_sensor_white_levels(),
            (unsigned int) g_track_app.last_line.black_bits);
        track_show_line(3U, line);
        if (g_track_app.line_only_mode) {
            track_show_line(4U, "C=GO U/D=SPD");
        } else {
            track_show_line(4U, "C=ZERO+START");
        }
    } else if (g_track_app.state == TRACK_APP_RUNNING) {
        if (g_track_app.line_only_mode) {
            track_show_line(1U,
                g_track_app.line_test_output.state ==
                CHASSIS_TRACK_LINE_TEST_BRAKING ?
                "LF ONLY BRAKING" : "LF ONLY RUN");
        } else {
            track_show_line(1U, chassis_track_state_text(
                g_track_app.output.state));
        }
        (void) snprintf(line, sizeof(line), "T%03lu.%01lu D%04lu",
            (unsigned long) ((g_track_app.line_only_mode ?
                g_track_app.line_test_output.elapsed_ms :
                (status->timestamp_ms -
                 g_track_app.mission.start_time_ms)) / 1000U),
            (unsigned long) (((g_track_app.line_only_mode ?
                g_track_app.line_test_output.elapsed_ms :
                (status->timestamp_ms -
                 g_track_app.mission.start_time_ms)) / 100U) % 10U),
            (unsigned long) (g_track_app.line_only_mode ?
                g_track_app.line_test_output.progress_mm :
                g_track_app.output.progress_mm));
        track_show_line(2U, line);
        if (g_track_app.line_only_mode) {
            (void) snprintf(line, sizeof(line), "S%03u V%03lu L%03u",
                (unsigned int) g_track_app.line_test_output.
                    selected_speed_mm_s,
                (unsigned long) g_track_app.line_test_output.
                    requested_speed_mm_s,
                    (unsigned int) g_track_app.line_control_output.lost_ms);
        } else {
            (void) snprintf(line, sizeof(line), "V%03lu H%+04ld I%u",
                (unsigned long) g_track_app.output.linear_mm_s,
                (long) g_track_app.output.heading_progress_deg,
                status->heading_fusion_active ? 1U : 0U);
        }
        track_show_line(3U, line);
        if (((status->timestamp_ms / TRACK_DIAGNOSTIC_PAGE_MS) & 1U) ==
            0U) {
            (void) snprintf(line, sizeof(line), "R%X W%X B%X",
                (unsigned int) g_track_app.last_line.raw_bits,
                (unsigned int) line_sensor_white_levels(),
                (unsigned int) g_track_app.last_line.black_bits);
        } else {
            (void) snprintf(line, sizeof(line), "LF%X L%u D%uH%u",
                (unsigned int) g_track_app.last_line.black_bits,
                g_track_app.line_control_output.recovering ? 1U : 0U,
                g_track_app.line_only_mode ? 0U :
                    g_track_app.output.distance_gate_met,
                g_track_app.line_only_mode ? 0U :
                    g_track_app.output.heading_gate_met);
        }
        track_show_line(4U, line);
    } else if (g_track_app.state == TRACK_APP_FINISHED) {
        if (g_track_app.line_only_mode) {
            time_cs = (int32_t)
                (g_track_app.line_test_output.elapsed_ms / 10U);
            track_show_line(1U, "LF TEST COMPLETE");
            (void) snprintf(line, sizeof(line), "TIME %02ld.%02ld S",
                (long) (time_cs / 100), (long) (time_cs % 100));
            track_show_line(2U, line);
            (void) snprintf(line, sizeof(line), "S%03u D%04lu",
                (unsigned int) g_track_app.line_test_output.
                    selected_speed_mm_s,
                (unsigned long) g_track_app.line_test_output.progress_mm);
            track_show_line(3U, line);
            track_show_line(4U, "C=GO D=CSV");
            return;
        }
        time_cs = (int32_t) (g_track_app.output.elapsed_s *
            100.0f + 0.5f);
        error_mm = (int32_t) (g_track_app.output.stop_error_mm +
            (g_track_app.output.stop_error_mm >= 0.0f ? 0.5f : -0.5f));
        track_show_line(1U, "LAP COMPLETE");
        (void) snprintf(line, sizeof(line), "TIME %02ld.%02ld S",
            (long) (time_cs / 100), (long) (time_cs % 100));
        track_show_line(2U, line);
        (void) snprintf(line, sizeof(line), "EST ERR %+04ldMM",
            (long) error_mm);
        track_show_line(3U, line);
        track_show_line(4U, g_track_app.output.passed ?
            "ENC+IMU PASS" : "ENC+IMU FAIL");
    } else {
        if (g_track_app.line_gpio_fault) {
            track_show_line(1U, "LF GPIO FAULT");
        } else if (g_track_app.chassis_fault == CHASSIS_FAULT_STALL) {
            track_show_line(1U, "TRACK STALL");
        } else if (g_track_app.chassis_fault ==
                   CHASSIS_FAULT_ENCODER) {
            track_show_line(1U, "ENCODER FAULT");
        } else if (g_track_app.chassis_fault ==
                   CHASSIS_FAULT_MOTOR_DRIVER) {
            track_show_line(1U, "MOTOR FAULT");
        } else if ((g_track_app.output.state ==
             CHASSIS_TRACK_FAULT_LAP_CHECK) ||
            (g_track_app.output.state == CHASSIS_TRACK_FAULT_EMERGENCY)) {
            track_show_line(1U, chassis_track_state_text(
                g_track_app.output.state));
        } else {
            track_show_line(1U, "TRACK HW FAULT");
        }
        if (g_track_app.output.state ==
            CHASSIS_TRACK_FAULT_LAP_CHECK) {
            error_mm = (int32_t) (g_track_app.output.progress_mm -
                chassis_track_route_length(&g_track_app.mission.config));
            (void) snprintf(line, sizeof(line), "D%+05ld H%+04ld",
                (long) error_mm,
                (long) g_track_app.output.heading_progress_deg);
            track_show_line(2U, line);
            (void) snprintf(line, sizeof(line), "ENC/IMU MISMATCH");
        } else {
            track_show_line(2U, "MOTORS LOCKED");
            (void) snprintf(line, sizeof(line), "STATUS %02u",
                (unsigned int) g_track_app.fault_status);
        }
        track_show_line(3U, line);
        track_show_line(4U, "POWER CYCLE ONLY");
    }
}

static void track_fail(ml_status_t status)
{
    chassis_emergency_stop();
    g_track_app.fault_status = status;
    g_track_app.chassis_fault = CHASSIS_FAULT_NONE;
    g_track_app.line_gpio_fault = false;
    g_track_app.state = TRACK_APP_FAULT;
}

static void track_fail_line_gpio(ml_status_t status)
{
    chassis_emergency_stop();
    g_track_app.fault_status = status;
    g_track_app.chassis_fault = CHASSIS_FAULT_NONE;
    g_track_app.line_gpio_fault = true;
    g_track_app.state = TRACK_APP_FAULT;
}

static void track_fail_chassis(chassis_fault_t fault)
{
    chassis_emergency_stop();
    g_track_app.fault_status = ML_STATUS_TIMEOUT;
    g_track_app.chassis_fault = fault;
    g_track_app.line_gpio_fault = false;
    g_track_app.state = TRACK_APP_FAULT;
}

ml_status_t chassis_track_app_init(void)
{
    chassis_track_line_control_config_t line_control_config;
    ml_status_t status;

    memset(&g_track_app, 0, sizeof(g_track_app));
    chassis_key_init(&g_track_app.center_key);
    chassis_key_init(&g_track_app.up_key);
    chassis_key_init(&g_track_app.down_key);
    status = OLED_Init();
    if (status == ML_STATUS_OK) {
        status = board_resource_claim(
            ML_BOARD_RESOURCE_PB24, ML_BOARD_OWNER_KEY);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_UP_PORT, ML_KEY_UP_PIN,
            (GPIOn_enum) ML_KEY_UP_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_DOWN_PORT, ML_KEY_DOWN_PIN,
            (GPIOn_enum) ML_KEY_DOWN_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
            (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        g_track_app.line_only_mode = track_up_pressed();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_init();
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(UART0, TRACK_UART_BAUD, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = chassis_init(&g_chassis_race_config);
    }
    if (status == ML_STATUS_OK) {
        status = chassis_track_mission_init(&g_track_app.mission,
            &g_chassis_track_default_config);
    }
    if (status == ML_STATUS_OK) {
        status = chassis_track_line_test_init(&g_track_app.line_test,
            &g_chassis_track_line_test_default_config);
    }
    if (status == ML_STATUS_OK) {
        line_control_config = g_track_app.line_only_mode ?
            g_chassis_track_line_control_line_only_config :
            g_chassis_track_line_control_default_config;
        line_control_config.effective_track_mm =
            g_chassis_race_config.effective_track_mm;
        line_control_config.maximum_wheel_speed_mm_s =
            g_chassis_race_config.maximum_wheel_speed_mm_s;
        line_control_config.control_period_ms =
            g_chassis_race_config.control_period_ms;
        status = chassis_track_line_control_init(
            &g_track_app.line_control, &line_control_config);
    }
    if (status == ML_STATUS_OK) {
        status = icm42688_service_init(&g_track_app.imu,
            &g_track_imu_config);
    }
    if (status != ML_STATUS_OK) {
        track_fail(status);
        return status;
    }
    g_track_app.state = TRACK_APP_CALIBRATING;
    line_sensor_white_guard_reset(&g_track_app.white_guard);
    g_track_app.initialized = true;
    chassis_key_require_release(&g_track_app.center_key);
    chassis_key_require_release(&g_track_app.up_key);
    chassis_key_require_release(&g_track_app.down_key);
    (void) OLED_Clear();
    return ML_STATUS_OK;
}

void chassis_track_app_poll(void)
{
    icm42688_service_event_t imu_event;
    chassis_status_t status;
    line_sample_t line;
    float center_distance_mm;
    ml_status_t command_status;
    bool key_press;
    bool up_press;
    bool down_press;
    uint8_t uart_byte;

    if (!g_track_app.initialized) {
        return;
    }
    imu_event = icm42688_service_poll(&g_track_app.imu,
        &g_track_app.imu_output);
    if ((imu_event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED) ||
        (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE)) {
        chassis_set_imu_sample(g_track_app.imu_output.angles.yaw_deg,
            g_track_app.imu_output.body_gyro_z_dps,
            g_track_app.imu_output.timestamp_ms, true);
    }
    if (imu_event == ICM42688_SERVICE_EVENT_CALIBRATION_COMPLETE) {
        g_track_app.imu_calibration_complete = true;
    }
    chassis_poll();
    status = chassis_get_status();
    if ((g_track_app.state == TRACK_APP_RUNNING) &&
        (status.result == CHASSIS_RESULT_FAULT)) {
        track_fail_chassis(status.fault);
    }
    chassis_key_update(&g_track_app.center_key,
        track_center_pressed(), status.timestamp_ms);
    chassis_key_update(&g_track_app.up_key,
        track_up_pressed(), status.timestamp_ms);
    chassis_key_update(&g_track_app.down_key,
        track_down_pressed(), status.timestamp_ms);
    key_press = chassis_key_take_press(&g_track_app.center_key);
    up_press = chassis_key_take_press(&g_track_app.up_key);
    down_press = chassis_key_take_press(&g_track_app.down_key);

    if (track_line_only_stopped_page()) {
        if (up_press) {
            chassis_track_line_test_speed_up(&g_track_app.line_test);
        }
        if (down_press) {
            chassis_track_line_test_speed_down(&g_track_app.line_test);
        }
    }

    if ((g_track_app.state == TRACK_APP_CALIBRATING) &&
        ((uint32_t) (status.timestamp_ms -
         g_track_app.last_white_sample_ms) >=
         CHASSIS_CONTROL_PERIOD_MS)) {
        g_track_app.last_white_sample_ms = status.timestamp_ms;
        g_track_app.last_line = line_sensor_read();
        if (g_track_app.last_line.io_fault) {
            track_fail_line_gpio(ML_STATUS_TIMEOUT);
        } else {
            (void) line_sensor_white_guard_update(
                &g_track_app.white_guard,
                g_track_app.last_line.raw_bits);
        }
    }

    if ((g_track_app.state == TRACK_APP_CALIBRATING) &&
        g_track_app.imu_calibration_complete &&
        (g_track_app.white_guard.consecutive_samples >=
         LINE_SENSOR_WHITE_STABLE_SAMPLES)) {
        g_track_app.state = TRACK_APP_READY;
        g_track_app.last_ready_sample_ms =
            status.timestamp_ms - CHASSIS_CONTROL_PERIOD_MS;
        chassis_key_require_release(&g_track_app.center_key);
    }

    if (((g_track_app.state == TRACK_APP_READY) ||
         (g_track_app.line_only_mode &&
          (g_track_app.state == TRACK_APP_FINISHED))) &&
        (status.timestamp_ms != g_track_app.last_ready_sample_ms)) {
        g_track_app.last_ready_sample_ms = status.timestamp_ms;
        g_track_app.last_line = line_sensor_read();
        if (g_track_app.last_line.io_fault) {
            track_fail_line_gpio(ML_STATUS_TIMEOUT);
        } else {
            chassis_telemetry_set_line_bits(
                g_track_app.last_line.black_bits);
            if (g_track_app.line_only_mode) {
                center_distance_mm = 0.5f *
                    (status.pose.left_distance_mm +
                     status.pose.right_distance_mm);
                (void) chassis_track_line_test_update(
                    &g_track_app.line_test, center_distance_mm,
                    status.measured_left_mm_s,
                    status.measured_right_mm_s,
                    status.timestamp_ms,
                    &g_track_app.line_test_output);
            }
        }
    }

    if (((g_track_app.state == TRACK_APP_READY) ||
         (g_track_app.line_only_mode &&
          (g_track_app.state == TRACK_APP_FINISHED))) && key_press) {
        command_status = line_sensor_reassert_inputs();
        if (command_status == ML_STATUS_OK) {
            g_track_app.last_line = line_sensor_read();
            if (g_track_app.last_line.io_fault) {
                command_status = ML_STATUS_TIMEOUT;
            }
        }
        if (command_status != ML_STATUS_OK) {
            if (command_status != ML_STATUS_BUSY) {
                track_fail_line_gpio(command_status);
            }
        } else {
            chassis_reset_pose(0.0f, 0.0f, 0.0f);
            status = chassis_get_status();
            center_distance_mm = 0.5f *
                (status.pose.left_distance_mm +
                 status.pose.right_distance_mm);
            if (g_track_app.line_only_mode) {
                command_status = chassis_track_line_test_start(
                    &g_track_app.line_test, center_distance_mm,
                    status.timestamp_ms);
            } else {
                command_status = chassis_track_mission_start(
                    &g_track_app.mission, center_distance_mm,
                    status.fused_heading_deg,
                    status.timestamp_ms);
            }
            if (command_status == ML_STATUS_OK) {
                chassis_track_line_control_reset(&g_track_app.line_control);
                chassis_telemetry_session_start(status.timestamp_ms);
                chassis_telemetry_set_line_bits(
                    g_track_app.last_line.black_bits);
                g_track_app.state = TRACK_APP_RUNNING;
                g_track_app.last_control_ms = status.timestamp_ms -
                    CHASSIS_CONTROL_PERIOD_MS;
                g_track_app.velocity_started = false;
                g_track_app.braking_capture_started = false;
                g_track_app.uart_busy_sent = false;
            } else {
                track_fail(command_status);
            }
        }
    } else if ((g_track_app.state == TRACK_APP_RUNNING) && key_press) {
        chassis_emergency_stop();
        g_track_app.velocity_started = false;
        if (g_track_app.line_only_mode) {
            chassis_telemetry_session_finish(status.timestamp_ms);
            chassis_telemetry_set_line_correction(0.0f);
            (void) chassis_capture_telemetry_now();
        }
        g_track_app.output.state = CHASSIS_TRACK_FAULT_EMERGENCY;
        g_track_app.fault_status = ML_STATUS_TIMEOUT;
        g_track_app.state = TRACK_APP_FAULT;
    }

    if ((g_track_app.state == TRACK_APP_RUNNING) &&
        (status.timestamp_ms != g_track_app.last_control_ms)) {
        g_track_app.last_control_ms = status.timestamp_ms;
        line = line_sensor_read();
        g_track_app.last_line = line;
        if (line.io_fault) {
            track_fail_line_gpio(ML_STATUS_TIMEOUT);
        } else {
            center_distance_mm = 0.5f *
                (status.pose.left_distance_mm +
                 status.pose.right_distance_mm);
            if (g_track_app.line_only_mode) {
                command_status = chassis_track_line_test_update(
                    &g_track_app.line_test, center_distance_mm,
                    status.measured_left_mm_s,
                    status.measured_right_mm_s,
                    status.timestamp_ms,
                    &g_track_app.line_test_output);
            } else {
                command_status = chassis_track_mission_update(
                    &g_track_app.mission, center_distance_mm,
                    status.measured_left_mm_s,
                    status.measured_right_mm_s,
                    status.fused_heading_deg, status.timestamp_ms, false,
                    &g_track_app.output);
            }
        }
        if ((g_track_app.state == TRACK_APP_RUNNING) &&
            (command_status != ML_STATUS_OK)) {
            track_fail(command_status);
        } else if ((g_track_app.state == TRACK_APP_RUNNING) &&
                   (g_track_app.line_only_mode ?
                    g_track_app.line_test_output.command_stop :
                    g_track_app.output.command_stop)) {
            if (!g_track_app.braking_capture_started) {
                chassis_stop();
                g_track_app.velocity_started = false;
                chassis_telemetry_set_line_correction(0.0f);
                if ((g_track_app.line_only_mode &&
                     (g_track_app.line_test_output.state ==
                      CHASSIS_TRACK_LINE_TEST_BRAKING)) ||
                    (!g_track_app.line_only_mode &&
                     (g_track_app.output.state ==
                      CHASSIS_TRACK_BRAKING))) {
                    (void) chassis_capture_telemetry_now();
                    command_status = chassis_idle_capture_start();
                    if (command_status == ML_STATUS_OK) {
                        g_track_app.braking_capture_started = true;
                    }
                }
            }
            if (g_track_app.line_only_mode ?
                g_track_app.line_test_output.finished :
                g_track_app.output.finished) {
                chassis_telemetry_session_finish(
                    g_track_app.line_only_mode ?
                    g_track_app.line_test.stop_time_ms :
                    g_track_app.mission.stop_time_ms);
                (void) chassis_capture_telemetry_now();
                chassis_idle_capture_stop();
                g_track_app.state = TRACK_APP_FINISHED;
            } else if (!g_track_app.line_only_mode &&
                       ((g_track_app.output.state ==
                        CHASSIS_TRACK_FAULT_LAP_CHECK) ||
                       (g_track_app.output.state ==
                        CHASSIS_TRACK_FAULT_EMERGENCY))) {
                g_track_app.fault_status = ML_STATUS_BUFFER_EMPTY;
                g_track_app.state = TRACK_APP_FAULT;
            }
        } else if (g_track_app.state == TRACK_APP_RUNNING) {
            command_status = chassis_track_line_control_update(
                &g_track_app.line_control, &line,
                g_track_app.line_only_mode ?
                    g_track_app.line_test_output.requested_speed_mm_s :
                    g_track_app.output.linear_mm_s,
                g_track_app.line_only_mode ? 0.0f :
                    g_track_app.output.angular_rad_s,
                &g_track_app.line_control_output);
            if (command_status == ML_STATUS_OK) {
                chassis_telemetry_set_line_state(line.black_bits,
                    g_track_app.line_control_output.line_valid,
                    g_track_app.line_control_output.recovering,
                    false);
                chassis_telemetry_set_line_correction(
                    g_track_app.line_control_output.correction_mm_s);
            }
            if ((command_status == ML_STATUS_OK) &&
                (g_track_app.state == TRACK_APP_RUNNING)) {
                if (g_track_app.line_only_mode) {
                    if (!g_track_app.velocity_started) {
                        command_status = chassis_set_wheel_speed(
                            g_track_app.line_control_output.left_mm_s,
                            g_track_app.line_control_output.right_mm_s);
                        g_track_app.velocity_started =
                            command_status == ML_STATUS_OK;
                    } else {
                        command_status = chassis_update_wheel_speed(
                            g_track_app.line_control_output.left_mm_s,
                            g_track_app.line_control_output.right_mm_s);
                    }
                } else {
                    g_track_app.output.linear_mm_s =
                        g_track_app.line_control_output.linear_mm_s;
                    g_track_app.output.angular_rad_s =
                        g_track_app.line_control_output.angular_rad_s;
                    if (!g_track_app.velocity_started) {
                        command_status = chassis_set_velocity(
                            g_track_app.output.linear_mm_s,
                            g_track_app.output.angular_rad_s);
                        g_track_app.velocity_started =
                            command_status == ML_STATUS_OK;
                    } else {
                        command_status = chassis_update_velocity(
                            g_track_app.output.linear_mm_s,
                            g_track_app.output.angular_rad_s);
                    }
                }
            }
        }
        if ((g_track_app.state == TRACK_APP_RUNNING) &&
            (command_status != ML_STATUS_OK)) {
            track_fail(command_status);
        }
    }
    while (uart_try_read(UART0, &uart_byte) == ML_STATUS_OK) {
        if ((g_track_app.state == TRACK_APP_RUNNING) &&
            ((uart_byte == (uint8_t) 'D') ||
             (uart_byte == (uint8_t) 'd') ||
             (uart_byte == (uint8_t) 'C') ||
             (uart_byte == (uint8_t) 'c'))) {
            if (!g_track_app.uart_busy_sent ||
                ((uint32_t) (status.timestamp_ms -
                 g_track_app.last_uart_busy_ms) >=
                 TRACK_UART_BUSY_PERIOD_MS)) {
                (void) chassis_uart0_send_busy();
                g_track_app.last_uart_busy_ms = status.timestamp_ms;
                g_track_app.uart_busy_sent = true;
            }
        } else {
            (void) chassis_telemetry_uart0_handle_byte(uart_byte, true);
        }
    }
    if ((uint32_t) (status.timestamp_ms -
        g_track_app.last_display_ms) >= TRACK_DISPLAY_PERIOD_MS) {
        track_show(&status);
        g_track_app.last_display_ms = status.timestamp_ms;
    }
}
