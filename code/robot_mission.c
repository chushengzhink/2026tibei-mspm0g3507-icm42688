#include "robot_mission.h"

#include <stdbool.h>

#include "hmi_control.h"
#include "line_sensor.h"
#include "ml_board.h"
#include "ml_gpio.h"
#include "ml_motor.h"
#include "ml_oled.h"
#include "ml_uart.h"
#include "robot_config.h"
#include "vision.h"

#define MISSION_KEY_DEBOUNCE_TICKS    (3U)
#define MISSION_DISPLAY_PERIOD_TICKS  (50U)

typedef struct {
    mission_state_t state;
    mission_fault_t fault;
    motion_fault_t motion_fault;
    uint8_t radius_cm;
    float park_to_a_mm;
    uint32_t state_start_tick;
    uint32_t mission_start_tick;
    uint32_t last_processed_tick;
    uint32_t last_display_tick;
    uint8_t key_pressed_ticks;
    bool key_stable_pressed;
    bool oled_ready;
} mission_context_t;

static mission_context_t g_mission;

static bool mission_is_running(void)
{
    return (g_mission.state != MISSION_STATE_WAIT_START) &&
        (g_mission.state != MISSION_STATE_DONE) &&
        (g_mission.state != MISSION_STATE_FAULT);
}

static void mission_write_line(uint8_t line, const char *text)
{
    char padded[17];
    uint8_t index = 0U;

    if (!g_mission.oled_ready) {
        return;
    }
    while ((text[index] != '\0') && (index < 16U)) {
        padded[index] = text[index];
        ++index;
    }
    while (index < 16U) {
        padded[index++] = ' ';
    }
    padded[16] = '\0';
    (void) OLED_ShowString(line, 1U, padded);
}

static const char *mission_state_text(mission_state_t state)
{
    switch (state) {
    case MISSION_STATE_WAIT_START:
        return "WAIT START";
    case MISSION_STATE_SEEK_A:
        return "SEEK A";
    case MISSION_STATE_WAIT_VISION:
        return "WAIT VISION";
    case MISSION_STATE_TURN_OUTBOUND:
        return "TURN TO LINE";
    case MISSION_STATE_FOLLOW_OUTBOUND:
        return "LINE OUT";
    case MISSION_STATE_TURN_TANGENT:
        return "TURN TANGENT";
    case MISSION_STATE_DRAW_CIRCLE:
        return "DRAW CIRCLE";
    case MISSION_STATE_TURN_RETURN:
        return "TURN RETURN";
    case MISSION_STATE_FOLLOW_RETURN:
        return "LINE RETURN";
    case MISSION_STATE_TURN_PARK:
        return "TURN PARK";
    case MISSION_STATE_ENTER_PARK:
        return "ENTER PARK";
    case MISSION_STATE_DONE:
        return "DONE";
    case MISSION_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static void mission_display_state(void)
{
    motion_status_t motion = motion_get_status();

    mission_write_line(1U, mission_state_text(g_mission.state));
    mission_write_line(2U, "RADIUS:-- CM");
    if (g_mission.radius_cm != 0U) {
        (void) OLED_ShowNum(2U, 8U, g_mission.radius_cm, 2U);
    }
    mission_write_line(3U, "LINE W:0 B:0");
    (void) OLED_ShowHexNum(3U, 8U, line_sensor_white_levels(), 1U);
    (void) OLED_ShowHexNum(3U, 12U, motion.line.black_bits, 1U);
    mission_write_line(4U, "ERR:00 M:00");
    (void) OLED_ShowNum(4U, 5U, (uint32_t) g_mission.fault, 2U);
    (void) OLED_ShowNum(4U, 10U, (uint32_t) g_mission.motion_fault, 2U);
}

static void mission_transition(mission_state_t state)
{
    g_mission.state = state;
    g_mission.state_start_tick = motion_get_uptime_ticks();
    mission_display_state();
}

static void mission_set_fault(
    mission_fault_t fault, motion_fault_t motion_fault)
{
    motion_emergency_stop();
    g_mission.fault = fault;
    g_mission.motion_fault = motion_fault;
    board_led_on();
    mission_transition(MISSION_STATE_FAULT);
}

static bool mission_start_command(ml_status_t status)
{
    if (status == ML_STATUS_OK) {
        return true;
    }
    mission_set_fault(MISSION_FAULT_COMMAND, MOTION_FAULT_NONE);
    return false;
}

static void mission_begin(void)
{
    vision_reset();
    g_mission.radius_cm = 0U;
    g_mission.park_to_a_mm = 0.0f;
    g_mission.fault = MISSION_FAULT_NONE;
    g_mission.motion_fault = MOTION_FAULT_NONE;
    g_mission.mission_start_tick = motion_get_uptime_ticks();
    board_led_off();
    if (mission_start_command(
        motion_start_find_a(ROBOT_APPROACH_SPEED_MM_S))) {
        mission_transition(MISSION_STATE_SEEK_A);
    }
}

static void mission_handle_key_tick(void)
{
    bool pressed = gpio_get(ML_KEY_CENTER_PORT,
        ML_KEY_CENTER_PIN) == ML_KEY_ACTIVE_LEVEL;
    bool stable;

    if (pressed) {
        if (g_mission.key_pressed_ticks < UINT8_MAX) {
            ++g_mission.key_pressed_ticks;
        }
    } else {
        g_mission.key_pressed_ticks = 0U;
    }
    stable = g_mission.key_pressed_ticks >= MISSION_KEY_DEBOUNCE_TICKS;
    if (stable && !g_mission.key_stable_pressed) {
        if (g_mission.state == MISSION_STATE_WAIT_START) {
            mission_begin();
        } else if (mission_is_running()) {
            mission_set_fault(
                MISSION_FAULT_EMERGENCY_STOP, MOTION_FAULT_NONE);
        } else {
            g_mission.fault = MISSION_FAULT_NONE;
            g_mission.motion_fault = MOTION_FAULT_NONE;
            g_mission.radius_cm = 0U;
            vision_reset();
            mission_transition(MISSION_STATE_WAIT_START);
        }
    }
    g_mission.key_stable_pressed = stable;
}

static void mission_handle_time(void)
{
    uint32_t now = motion_get_uptime_ticks();

    while (g_mission.last_processed_tick != now) {
        ++g_mission.last_processed_tick;
        vision_tick_10ms();
        mission_handle_key_tick();
    }
    if (mission_is_running() &&
        ((uint32_t) (now - g_mission.mission_start_tick) >=
         ROBOT_MISSION_TIMEOUT_TICKS)) {
        mission_set_fault(
            MISSION_FAULT_MISSION_TIMEOUT, MOTION_FAULT_NONE);
    }
}

static void mission_drain_vision_uart(void)
{
    uint8_t byte;

    while (uart_try_read(ROBOT_VISION_UART, &byte) == ML_STATUS_OK) {
        vision_feed_byte(byte);
    }
}

static void mission_drain_hmi_uart(void)
{
    uint8_t byte;

    while (uart_try_read(ROBOT_HMI_UART, &byte) == ML_STATUS_OK) {
        hmi_control_feed_byte(byte);
    }
}

static void mission_handle_hmi_start(void)
{
    if (!hmi_control_take_start_request()) {
        return;
    }
    if (g_mission.state == MISSION_STATE_WAIT_START) {
        mission_begin();
    }
}

static void mission_show_periodic(void)
{
    uint32_t now = motion_get_uptime_ticks();
    motion_status_t motion;

    if ((uint32_t) (now - g_mission.last_display_tick) <
        MISSION_DISPLAY_PERIOD_TICKS) {
        return;
    }
    g_mission.last_display_tick = now;
    motion = motion_get_status();
    (void) OLED_ShowHexNum(3U, 8U, line_sensor_white_levels(), 1U);
    (void) OLED_ShowHexNum(3U, 12U, motion.line.black_bits, 1U);
}

static void mission_advance(void)
{
    motion_status_t motion = motion_get_status();
    uint32_t now = motion.uptime_ticks;

    if (motion.result == MOTION_RESULT_FAULT) {
        mission_set_fault(MISSION_FAULT_MOTION, motion.fault);
        return;
    }

    switch (g_mission.state) {
    case MISSION_STATE_SEEK_A:
        if (motion.result == MOTION_RESULT_COMPLETE) {
            g_mission.park_to_a_mm = motion.mean_distance_mm;
            if (vision_try_get_radius(&g_mission.radius_cm)) {
                if (mission_start_command(motion_start_turn90(true, true))) {
                    mission_transition(MISSION_STATE_TURN_OUTBOUND);
                }
            } else {
                mission_transition(MISSION_STATE_WAIT_VISION);
            }
        }
        break;

    case MISSION_STATE_WAIT_VISION:
        if (vision_try_get_radius(&g_mission.radius_cm)) {
            if (mission_start_command(motion_start_turn90(true, true))) {
                mission_transition(MISSION_STATE_TURN_OUTBOUND);
            }
        } else if ((uint32_t) (now - g_mission.state_start_tick) >=
            ROBOT_VISION_TIMEOUT_TICKS) {
            mission_set_fault(
                MISSION_FAULT_VISION_TIMEOUT, MOTION_FAULT_NONE);
        }
        break;

    case MISSION_STATE_TURN_OUTBOUND:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_line(
                (float) g_mission.radius_cm * 10.0f,
                ROBOT_LINE_SPEED_MM_S))) {
            mission_transition(MISSION_STATE_FOLLOW_OUTBOUND);
        }
        break;

    case MISSION_STATE_FOLLOW_OUTBOUND:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_turn90(true, false))) {
            mission_transition(MISSION_STATE_TURN_TANGENT);
        }
        break;

    case MISSION_STATE_TURN_TANGENT:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_circle(
                (uint16_t) g_mission.radius_cm * 10U))) {
            mission_transition(MISSION_STATE_DRAW_CIRCLE);
        }
        break;

    case MISSION_STATE_DRAW_CIRCLE:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_turn90(true, true))) {
            mission_transition(MISSION_STATE_TURN_RETURN);
        }
        break;

    case MISSION_STATE_TURN_RETURN:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_line_to_end(
                (float) g_mission.radius_cm * 10.0f,
                ROBOT_LINE_SPEED_MM_S))) {
            mission_transition(MISSION_STATE_FOLLOW_RETURN);
        }
        break;

    case MISSION_STATE_FOLLOW_RETURN:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_turn90(false, false))) {
            mission_transition(MISSION_STATE_TURN_PARK);
        }
        break;

    case MISSION_STATE_TURN_PARK:
        if ((motion.result == MOTION_RESULT_COMPLETE) &&
            mission_start_command(motion_start_straight(
                g_mission.park_to_a_mm, ROBOT_PARK_SPEED_MM_S))) {
            mission_transition(MISSION_STATE_ENTER_PARK);
        }
        break;

    case MISSION_STATE_ENTER_PARK:
        if (motion.result == MOTION_RESULT_COMPLETE) {
            board_led_on();
            mission_transition(MISSION_STATE_DONE);
        }
        break;

    default:
        break;
    }
}

ml_status_t robot_mission_init(void)
{
    ml_status_t status;

    g_mission.state = MISSION_STATE_WAIT_START;
    g_mission.fault = MISSION_FAULT_NONE;
    g_mission.motion_fault = MOTION_FAULT_NONE;
    g_mission.radius_cm = 0U;
    g_mission.park_to_a_mm = 0.0f;
    g_mission.key_pressed_ticks = 0U;
    g_mission.key_stable_pressed = false;
    g_mission.oled_ready = false;

    status = board_led_init();
    if (status == ML_STATUS_OK) {
        status = gpio_init(ML_KEY_CENTER_PORT, ML_KEY_CENTER_PIN,
            (GPIOn_enum) ML_KEY_CENTER_IOMUX, IN_UP);
    }
    if (status == ML_STATUS_OK) {
        status = motor_init();
    }
    if (status == ML_STATUS_OK) {
        status = encoder_init();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_init();
    }
    if (status == ML_STATUS_OK) {
        status = line_sensor_calibrate_white(64U, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(ROBOT_HMI_UART, ROBOT_HMI_BAUD, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = uart_init(ROBOT_VISION_UART, ROBOT_VISION_BAUD, 2U);
    }
    if (status == ML_STATUS_OK) {
        status = OLED_Init();
        if (status == ML_STATUS_OK) {
            g_mission.oled_ready = true;
        }
    }
    hmi_control_init();
    vision_init();
    if (status == ML_STATUS_OK) {
        status = motion_init(&g_robot_calibration);
    }
    if (status != ML_STATUS_OK) {
        g_mission.fault = MISSION_FAULT_INIT;
        return status;
    }

    g_mission.last_processed_tick = motion_get_uptime_ticks();
    g_mission.last_display_tick = g_mission.last_processed_tick;
    g_mission.state_start_tick = g_mission.last_processed_tick;
    g_mission.mission_start_tick = g_mission.last_processed_tick;
    (void) OLED_Clear();
    mission_display_state();
    return ML_STATUS_OK;
}

void robot_mission_poll(void)
{
    mission_drain_hmi_uart();
    mission_drain_vision_uart();
    mission_handle_time();
    mission_handle_hmi_start();
    mission_advance();
    mission_show_periodic();
}

mission_state_t robot_mission_get_state(void)
{
    return g_mission.state;
}

mission_fault_t robot_mission_get_fault(void)
{
    return g_mission.fault;
}
