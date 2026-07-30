#ifndef ROBOT_INPUT_H
#define ROBOT_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "ml_common.h"

ml_status_t robot_input_init(void);
void robot_input_poll(uint32_t elapsed_ticks);
bool robot_input_take_hmi_start(void);
bool robot_input_take_center_press(void);
void robot_input_reset_vision(void);
bool robot_input_try_get_radius(uint8_t *radius_cm);

#endif
