#include "mission_view.h"

#include "ml_oled.h"

static void mission_view_write_line(uint8_t line, const char *text)
{
    char padded[17];
    uint8_t index = 0U;

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

static const char *mission_view_state_text(mission_state_t state)
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

ml_status_t mission_view_init(void)
{
    ml_status_t status = OLED_Init();

    if (status == ML_STATUS_OK) {
        status = OLED_Clear();
    }
    return status;
}

void mission_view_render(mission_state_t state,
    mission_fault_t fault, motion_fault_t motion_fault,
    uint8_t radius_cm, uint8_t white_levels,
    const motion_status_t *motion)
{
    if (motion == 0) {
        return;
    }
    mission_view_write_line(1U, mission_view_state_text(state));
    mission_view_write_line(2U, "RADIUS:-- CM");
    if (radius_cm != 0U) {
        (void) OLED_ShowNum(2U, 8U, radius_cm, 2U);
    }
    mission_view_write_line(3U, "LINE W:0 B:0");
    (void) OLED_ShowHexNum(3U, 8U, white_levels, 1U);
    (void) OLED_ShowHexNum(3U, 12U, motion->line.black_bits, 1U);
    mission_view_write_line(4U, "ERR:00 M:00");
    (void) OLED_ShowNum(4U, 5U, (uint32_t) fault, 2U);
    (void) OLED_ShowNum(4U, 10U, (uint32_t) motion_fault, 2U);
}

void mission_view_update_line(
    uint8_t white_levels, const motion_status_t *motion)
{
    if (motion == 0) {
        return;
    }
    (void) OLED_ShowHexNum(3U, 8U, white_levels, 1U);
    (void) OLED_ShowHexNum(3U, 12U, motion->line.black_bits, 1U);
}
