/* Implements source-agnostic straight-path geometry helpers. */
#include "control/drivetrain/path_planner.h"

#include <math.h>
#include <stddef.h>

static const float kPi = 3.14159265358979323846f;

bool motion_estimate_is_valid(const MotionEstimate *estimate)
{
    return estimate != NULL && estimate->valid &&
           isfinite(estimate->x_m) && isfinite(estimate->y_m) &&
           isfinite(estimate->heading_rad);
}

esp_err_t path_planner_line_start(
    PathPlannerLine *line,
    const MotionEstimate *start,
    float target_x_m,
    float target_y_m)
{
    if (line == NULL || !motion_estimate_is_valid(start) ||
        !isfinite(target_x_m) || !isfinite(target_y_m)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float dx = target_x_m - start->x_m;
    const float dy = target_y_m - start->y_m;
    const float length_m = hypotf(dx, dy);
    if (length_m <= 1.0e-6f) return ESP_ERR_INVALID_ARG;

    line->start_x_m = start->x_m;
    line->start_y_m = start->y_m;
    line->direction_x = dx / length_m;
    line->direction_y = dy / length_m;
    line->length_m = length_m;
    return ESP_OK;
}

esp_err_t path_planner_line_feedback(
    const PathPlannerLine *line,
    const MotionEstimate *estimate,
    PathPlannerLineFeedback *feedback_out)
{
    if (line == NULL || feedback_out == NULL || !motion_estimate_is_valid(estimate) ||
        !isfinite(line->start_x_m) || !isfinite(line->start_y_m) ||
        !isfinite(line->direction_x) || !isfinite(line->direction_y) ||
        !isfinite(line->length_m) || line->length_m <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float dx = estimate->x_m - line->start_x_m;
    const float dy = estimate->y_m - line->start_y_m;
    const float progress_m = dx * line->direction_x + dy * line->direction_y;

    /* World-frame path-left is (-uy, ux), matching positive body vy. */
    const float displacement_left_m =
        -dx * line->direction_y + dy * line->direction_x;
    feedback_out->along_track_progress_m = progress_m;
    feedback_out->correction_error_m = -displacement_left_m;

    const float remaining_along_m = line->length_m - progress_m;
    feedback_out->distance_to_goal_m = hypotf(
        remaining_along_m,
        displacement_left_m);
    return ESP_OK;
}

float path_planner_wrap_angle_rad(float angle_rad)
{
    if (!isfinite(angle_rad)) return NAN;
    const float wrapped = remainderf(angle_rad, 2.0f * kPi);
    return wrapped == -kPi ? kPi : wrapped;
}
