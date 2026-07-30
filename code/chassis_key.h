#ifndef CHASSIS_KEY_H
#define CHASSIS_KEY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t pressed_since_ms;
    bool was_pressed;
    bool release_seen;
    bool press_pending;
} chassis_key_t;

void chassis_key_init(chassis_key_t *key);
void chassis_key_require_release(chassis_key_t *key);
void chassis_key_update(
    chassis_key_t *key, bool pressed, uint32_t now_ms);
bool chassis_key_take_press(chassis_key_t *key);

#endif
