#ifndef H5_BUILD
#define H5_BUILD (0)
#endif

#include "h5_app.h"
#include "ml_system.h"

int main(void)
{
    if (system_init() != ML_STATUS_OK) {
        while (1) {
        }
    }

    __enable_irq();
    (void) h5_app_init();
    while (1) {
        h5_app_poll();
    }
}
