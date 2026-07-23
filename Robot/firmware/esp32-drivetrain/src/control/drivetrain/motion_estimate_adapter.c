#include "control/drivetrain/motion_estimate_adapter.h"

#include <math.h>
#include <stddef.h>

esp_err_t motion_estimate_from_drivetrain_pose(
    const DrivetrainPose *pose,
    bool estimate_valid,
    MotionEstimate *estimate_out)
{
    if (pose == NULL || estimate_out == NULL || !isfinite(pose->x_mm) ||
        !isfinite(pose->y_mm) || !isfinite(pose->heading_rad)) {
        if (estimate_out != NULL) *estimate_out = (MotionEstimate){0};
        return ESP_ERR_INVALID_ARG;
    }

    estimate_out->x_m = pose->x_mm / 1000.0f;
    estimate_out->y_m = pose->y_mm / 1000.0f;
    estimate_out->heading_rad = pose->heading_rad;
    estimate_out->valid = estimate_valid;
    return estimate_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}
