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

    /* Body "right" (positive lateral, matching DrivetrainBodyVelocity.vy's
     * convention) is 90 degrees CLOCKWISE from "forward" in a CCW-positive
     * heading frame (matching DrivetrainBodyVelocity.omega's convention) --
     * i.e. world-frame right = (sin(heading), -cos(heading)). Previously
     * this used +cosine on the lateral term for y_mm, which is the sign a
     * CW-positive heading (or a left-positive lateral) would require --
     * inconsistent with the CCW/right convention documented everywhere else
     * and used by callers like move_s.c's progress projection. Concretely
     * wrong for any move starting from a nonzero heading (e.g. after a
     * RotS turn): a robot that had turned 90 degrees CCW and then moved
     * forward ended up reported at negative y instead of positive. */
    const float cosine = cosf(odometry->pose.heading_rad);
    const float sine = sinf(odometry->pose.heading_rad);
    odometry->pose.x_mm += cosine * delta->forward_mm + sine * delta->lateral_mm;
    odometry->pose.y_mm += sine * delta->forward_mm - cosine * delta->lateral_mm;
    odometry->pose.heading_rad += delta->heading_delta_rad;
    return ESP_OK;
}

// Restores pose and fault fields to their zero defaults.
void drivetrain_odometry_reset(DrivetrainOdometry *odometry) {
    if (odometry != NULL) memset(odometry, 0, sizeof(*odometry));
}
