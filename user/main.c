#include "attitude_app.h"
#include "ml_system.h"

int main(void)
{
    if (system_init() != ML_STATUS_OK) {
        while (1) {
        }
    }

    __enable_irq();
    (void) attitude_app_init();
    while (1) {
        attitude_app_poll();
    }
}
