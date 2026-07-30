#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stdint.h>

void vision_init(void);
void vision_reset(void);
void vision_feed_byte(uint8_t byte);
void vision_tick_10ms(void);
bool vision_try_get_radius(uint8_t *radius_cm);
uint16_t vision_valid_packet_count(void);

#endif
