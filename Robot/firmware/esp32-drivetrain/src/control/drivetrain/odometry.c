/* Implements world-frame pose integration from body-frame motion deltas. */
#include "control/drivetrain/odometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Wraps an angle difference into (-pi, pi] so a heading crossing +-pi does
// not appear as a near-2*pi jump.
static float wrap_angle_rad(float angle_rad) {
    while (angle_rad > (float)M_PI) angle_rad -= 2.0f * (float)M_PI;
    while (angle_rad <= -(float)M_PI) angle_rad += 2.0f * (float)M_PI;
    return angle_rad;
}

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

// Re-anchors pose to an arbitrary value and clears fault state.
esp_err_t drivetrain_odometry_set_pose(
    DrivetrainOdometry *odometry,
    const DrivetrainPose *pose
) {
    if (odometry == NULL || pose == NULL ||
        !isfinite(pose->x_mm) || !isfinite(pose->y_mm) ||
        !isfinite(pose->heading_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    odometry->pose = *pose;
    odometry->fault_latched = false;
    odometry->last_fault = DRIVETRAIN_ODOMETRY_FAULT_NONE;
    return ESP_OK;
}

// Inverts the same rotation drivetrain_odometry_update() applies, recovering
// the body-frame delta that would have produced this world-frame pose change.
DrivetrainOdometryDelta drivetrain_odometry_delta_between(
    const DrivetrainPose *from,
    const DrivetrainPose *to
) {
    const float cosine = cosf(from->heading_rad);
    const float sine = sinf(from->heading_rad);
    const float world_dx_mm = to->x_mm - from->x_mm;
    const float world_dy_mm = to->y_mm - from->y_mm;

    return (DrivetrainOdometryDelta){
        .forward_mm = cosine * world_dx_mm + sine * world_dy_mm,
        .lateral_mm = -sine * world_dx_mm + cosine * world_dy_mm,
        .heading_delta_rad =
            wrap_angle_rad(to->heading_rad - from->heading_rad),
    };
}
