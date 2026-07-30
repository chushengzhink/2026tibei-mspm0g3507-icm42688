#include "chassis_key.h"
#include "chassis_config.h"

#include <stdio.h>

static int g_failures;

static void expect(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

int main(void)
{
    chassis_key_t key;

    expect(CHASSIS_SW6_MAPPING_DIAGNOSTIC == 0U,
        "verified center-key control is enabled");

    chassis_key_init(&key);
    chassis_key_update(&key, false, 0U);
    chassis_key_update(&key, true, 20U);
    chassis_key_update(&key, true, 79U);
    expect(!chassis_key_take_press(&key),
        "press shorter than 60 ms is filtered");
    chassis_key_update(&key, true, 80U);
    expect(chassis_key_take_press(&key),
        "press at 60 ms produces one event");
    chassis_key_update(&key, true, 200U);
    expect(!chassis_key_take_press(&key),
        "a held key does not repeat");

    chassis_key_update(&key, false, 220U);
    chassis_key_update(&key, true, 240U);
    chassis_key_update(&key, true, 340U);
    expect(chassis_key_take_press(&key),
        "release rearms the next press despite a long poll gap");

    chassis_key_require_release(&key);
    chassis_key_update(&key, true, 500U);
    expect(!chassis_key_take_press(&key),
        "entering a wait state while held cannot trigger");
    chassis_key_update(&key, false, 520U);
    chassis_key_update(&key, true, 540U);
    chassis_key_update(&key, true, 600U);
    expect(chassis_key_take_press(&key),
        "a release after entering wait state rearms the key");

    if (g_failures == 0) {
        printf("PASS: chassis key time debounce\n");
    }
    return g_failures == 0 ? 0 : 1;
}
