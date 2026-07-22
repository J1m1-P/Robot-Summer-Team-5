/* Implements world-frame pose integration from body-frame motion deltas. */
#include "control/drivetrain/odometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// Integrates one valid body delta or records an invalid-cycle fault.
esp_err_t drivetrain_odometry_update(
    DrivetrainOdometry *odometry,
    const DrivetrainOdometryDelta *delta,
    bool valid
) {
    if (odometry == NULL || delta == NULL ||
        !isfinite(delta->forward_mm) || !isfinite(delta->lateral_mm) ||
        !isfinite(delta->heading_delta_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!valid) {
        odometry->fault_latched = true;
        odometry->last_fault = DRIVETRAIN_ODOMETRY_FAULT_INVALID_CYCLE;
        return ESP_OK;
    }

    /* Right-handed frame: +x forward, +y left, +heading CCW. */
    const float cosine = cosf(odometry->pose.heading_rad);
    const float sine = sinf(odometry->pose.heading_rad);
    odometry->pose.x_mm += cosine * delta->forward_mm - sine * delta->lateral_mm;
    odometry->pose.y_mm += sine * delta->forward_mm + cosine * delta->lateral_mm;
    odometry->pose.heading_rad += delta->heading_delta_rad;
    return ESP_OK;
}

// Restores pose and fault fields to their zero defaults.
void drivetrain_odometry_reset(DrivetrainOdometry *odometry) {
    if (odometry != NULL) memset(odometry, 0, sizeof(*odometry));
}
