#include "chassis_key.h"

#define CHASSIS_KEY_DEBOUNCE_MS (60U)

void chassis_key_init(chassis_key_t *key)
{
    if (key == 0) {
        return;
    }
    key->pressed_since_ms = 0U;
    key->was_pressed = false;
    key->release_seen = false;
    key->press_pending = false;
}

void chassis_key_require_release(chassis_key_t *key)
{
    if (key == 0) {
        return;
    }
    key->release_seen = false;
    key->press_pending = false;
}

void chassis_key_update(
    chassis_key_t *key, bool pressed, uint32_t now_ms)
{
    if (key == 0) {
        return;
    }
    if (!pressed) {
        key->was_pressed = false;
        key->release_seen = true;
        key->press_pending = false;
        return;
    }
    if (!key->was_pressed) {
        key->was_pressed = true;
        key->pressed_since_ms = now_ms;
        return;
    }
    if (key->release_seen &&
        ((uint32_t) (now_ms - key->pressed_since_ms) >=
         CHASSIS_KEY_DEBOUNCE_MS)) {
        key->release_seen = false;
        key->press_pending = true;
    }
}

bool chassis_key_take_press(chassis_key_t *key)
{
    bool pending;

    if (key == 0) {
        return false;
    }
    pending = key->press_pending;
    key->press_pending = false;
    return pending;
}
