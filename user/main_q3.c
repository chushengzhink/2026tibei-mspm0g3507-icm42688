#include "ml_system.h"
#include "q3_ball_app.h"

int main(void)
{
    if (system_init() != ML_STATUS_OK) {
        while (1) {
        }
    }
    __enable_irq();
    if (q3_ball_app_init() != ML_STATUS_OK) {
        while (1) {
        }
    }
    while (1) {
        q3_ball_app_poll();
    }
}
