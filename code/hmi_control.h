#ifndef HMI_CONTROL_H
#define HMI_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void hmi_control_init(void);
void hmi_control_feed_byte(uint8_t byte);
bool hmi_control_take_start_request(void);

#endif
