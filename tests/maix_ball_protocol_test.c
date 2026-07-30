#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "maix_ball_protocol.h"

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
    data[2] = (uint8_t) (value >> 16U);
    data[3] = (uint8_t) (value >> 24U);
}

static void make_frame(uint8_t *frame, uint32_t capture_ms,
    int16_t x, int16_t y, float score, uint8_t valid)
{
    uint16_t crc;

    memset(frame, 0, MAIX_BALL_FRAME_SIZE);
    frame[0] = 0xAAU;
    frame[1] = 0xCAU;
    frame[2] = 0xACU;
    frame[3] = 0xBBU;
    write_le32(&frame[4], MAIX_BALL_DATA_SIZE);
    frame[8] = 0xA1U;
    frame[9] = 0x02U;
    write_le32(&frame[10], capture_ms);
    write_le16(&frame[14], (uint16_t) x);
    write_le16(&frame[16], (uint16_t) y);
    memcpy(&frame[18], &score, sizeof(score));
    frame[22] = valid;
    crc = maix_crc16_ibm(frame, 26U);
    write_le16(&frame[26], crc);
}

static bool feed(maix_ball_parser_t *parser, const uint8_t *bytes,
    uint32_t length, maix_ball_measurement_t *measurement)
{
    uint32_t i;
    bool decoded = false;

    for (i = 0U; i < length; ++i) {
        if (maix_ball_parser_push(parser, bytes[i], measurement)) {
            decoded = true;
        }
    }
    return decoded;
}

int main(void)
{
    maix_ball_parser_t parser;
    maix_ball_measurement_t measurement;
    uint8_t frame[MAIX_BALL_FRAME_SIZE];
    uint8_t stream[MAIX_BALL_FRAME_SIZE + 5U];

    assert(maix_crc16_ibm(0, 10U) == 0U);
    maix_ball_parser_init(&parser);
    make_frame(frame, 0x12345678UL, 160, 112, 0.75f, 1U);
    assert(!feed(&parser, frame, 9U, &measurement));
    assert(feed(&parser, &frame[9], MAIX_BALL_FRAME_SIZE - 9U,
        &measurement));
    assert(measurement.capture_ms == 0x12345678UL);
    assert(measurement.center_x_px == 160);
    assert(measurement.center_y_px == 112);
    assert(fabsf(measurement.score - 0.75f) < 0.0001f);
    assert(measurement.valid);

    make_frame(frame, 20U, 0, 0, 0.0f, 0U);
    assert(feed(&parser, frame, sizeof(frame), &measurement));
    assert(!measurement.valid);
    assert(measurement.score == 0.0f);

    make_frame(frame, 30U, 100, 100, 0.5f, 1U);
    frame[26] ^= 0x01U;
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    assert(parser.crc_errors == 1U);

    make_frame(frame, 40U, 100, 100, 0.5f, 1U);
    frame[4] = 19U;
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    assert(parser.length_errors >= 1U);

    make_frame(frame, 50U, 100, 100, 0.5f, 1U);
    frame[8] = 0xA2U;
    write_le16(&frame[26], maix_crc16_ibm(frame, 26U));
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    assert(parser.format_errors == 1U);

    make_frame(frame, 51U, 100, 100, 0.5f, 1U);
    frame[8] = 0x81U;
    write_le16(&frame[26], maix_crc16_ibm(frame, 26U));
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    make_frame(frame, 52U, 100, 100, 0.5f, 1U);
    frame[9] = 0x03U;
    write_le16(&frame[26], maix_crc16_ibm(frame, 26U));
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    make_frame(frame, 53U, 100, 100, 0.5f, 2U);
    assert(!feed(&parser, frame, sizeof(frame), &measurement));
    assert(parser.format_errors == 4U);

    make_frame(frame, 60U, 120, 110, 0.8f, 1U);
    memset(stream, 0x55, 5U);
    memcpy(&stream[5], frame, sizeof(frame));
    assert(feed(&parser, stream, sizeof(stream), &measurement));
    assert(measurement.capture_ms == 60U);
    assert(parser.frames_ok == 3U);

    printf("maix ball protocol tests: PASS\n");
    return 0;
}
