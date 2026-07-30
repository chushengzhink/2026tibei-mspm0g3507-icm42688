#include "maix_ball_protocol.h"

#include <float.h>
#include <string.h>

#define MAIX_FLAG_RESPONSE       (0x80U)
#define MAIX_FLAG_REPORT         (0x20U)
#define MAIX_FLAG_VERSION_MASK   (0x03U)
#define MAIX_PROTOCOL_VERSION    (0x01U)
#define MAIX_BALL_REPORT_COMMAND (0x02U)

static const uint8_t g_maix_header[4] = {0xAAU, 0xCAU, 0xACU, 0xBBU};

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t) ((uint16_t) data[0] |
        ((uint16_t) data[1] << 8U));
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t) data[0] |
        ((uint32_t) data[1] << 8U) |
        ((uint32_t) data[2] << 16U) |
        ((uint32_t) data[3] << 24U);
}

static bool float_is_valid_score(float value)
{
    return (value == value) && (value >= 0.0f) &&
        (value <= 1.0f) && (value <= FLT_MAX);
}

uint16_t maix_crc16_ibm(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0U;
    uint32_t i;
    uint32_t bit;

    if (data == 0) {
        return 0U;
    }
    for (i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t) ((crc >> 1U) ^ 0xA001U);
            } else {
                crc = (uint16_t) (crc >> 1U);
            }
        }
    }
    return crc;
}

void maix_ball_parser_init(maix_ball_parser_t *parser)
{
    if (parser != 0) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void parser_reset(maix_ball_parser_t *parser, uint8_t last_byte)
{
    parser->frame_size = 0U;
    if (last_byte == g_maix_header[0]) {
        parser->frame[0] = last_byte;
        parser->frame_size = 1U;
    }
}

static bool parser_decode(maix_ball_parser_t *parser,
    maix_ball_measurement_t *measurement)
{
    uint8_t flags = parser->frame[8];
    uint8_t valid = parser->frame[22];
    uint16_t expected_crc = maix_crc16_ibm(parser->frame, 26U);
    uint16_t received_crc = read_le16(&parser->frame[26]);
    float score;

    if (expected_crc != received_crc) {
        ++parser->crc_errors;
        return false;
    }
    if (((flags & (MAIX_FLAG_RESPONSE | MAIX_FLAG_REPORT)) !=
         (MAIX_FLAG_RESPONSE | MAIX_FLAG_REPORT)) ||
        ((flags & MAIX_FLAG_VERSION_MASK) != MAIX_PROTOCOL_VERSION) ||
        (parser->frame[9] != MAIX_BALL_REPORT_COMMAND) ||
        (valid > 1U)) {
        ++parser->format_errors;
        return false;
    }

    memcpy(&score, &parser->frame[18], sizeof(score));
    if ((valid != 0U) && !float_is_valid_score(score)) {
        ++parser->format_errors;
        return false;
    }

    measurement->capture_ms = read_le32(&parser->frame[10]);
    measurement->center_x_px = (int16_t) read_le16(&parser->frame[14]);
    measurement->center_y_px = (int16_t) read_le16(&parser->frame[16]);
    measurement->score = (valid == 0U) ? 0.0f : score;
    measurement->valid = (valid != 0U);
    ++parser->frames_ok;
    return true;
}

bool maix_ball_parser_push(maix_ball_parser_t *parser, uint8_t byte,
    maix_ball_measurement_t *measurement)
{
    bool decoded;

    if ((parser == 0) || (measurement == 0)) {
        return false;
    }

    if (parser->frame_size < 4U) {
        if (byte == g_maix_header[parser->frame_size]) {
            parser->frame[parser->frame_size++] = byte;
        } else {
            parser_reset(parser, byte);
        }
        return false;
    }

    parser->frame[parser->frame_size++] = byte;
    if (parser->frame_size == 8U) {
        if (read_le32(&parser->frame[4]) != MAIX_BALL_DATA_SIZE) {
            ++parser->length_errors;
            parser_reset(parser, byte);
        }
        return false;
    }
    if (parser->frame_size < MAIX_BALL_FRAME_SIZE) {
        return false;
    }

    decoded = parser_decode(parser, measurement);
    parser_reset(parser, byte);
    return decoded;
}
