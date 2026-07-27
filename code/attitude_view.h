#ifndef ATTITUDE_VIEW_H
#define ATTITUDE_VIEW_H

#include <stdbool.h>
#include <stdint.h>

#include "icm42688_service.h"
#include "ml_common.h"

ml_status_t attitude_view_init(void);
void attitude_view_show_error(
    icm42688_service_state_t state, ml_status_t status);
void attitude_view_show_calibration(void);
void attitude_view_show_calibration_progress(
    uint16_t calibration_samples, bool restarted);
void attitude_view_show_angles_layout(void);
void attitude_view_show_angles(const imu_attitude_angles_t *angles);

#endif
