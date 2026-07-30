#ifndef MAIX_BALL_PROTOCOL_H
#define MAIX_BALL_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define MAIX_BALL_BODY_SIZE  (20U)
#define MAIX_BALL_DATA_SIZE  (24U)
#define MAIX_BALL_FRAME_SIZE (32U)

typedef struct {
    uint32_t capture_ms;
    int16_t center_x_px;
    int16_t center_y_px;
    float position_cm;
    float score;
    bool valid;
} maix_ball_measurement_t;

typedef struct {
    uint8_t frame[MAIX_BALL_FRAME_SIZE];
    uint8_t frame_size;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t format_errors;
} maix_ball_parser_t;

void maix_ball_parser_init(maix_ball_parser_t *parser);
bool maix_ball_parser_push(maix_ball_parser_t *parser, uint8_t byte,
    maix_ball_measurement_t *measurement);
uint16_t maix_crc16_ibm(const uint8_t *data, uint32_t length);

#endif
