/* Declares sensor-agnostic geometry helpers for advanced movement paths. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A world-frame pose supplied by the active estimator.  Encoder odometry and
 * a future PMW3610 estimator both adapt into this unit-normalized contract. */
typedef struct {
    float x_m;
    float y_m;
    float heading_rad;
    bool valid;
} MotionEstimate;

/* Immutable geometry for a straight world-frame path. */
typedef struct {
    float start_x_m;
    float start_y_m;
    float direction_x;
    float direction_y;
    float length_m;
} PathPlannerLine;

/* Feedback derived from an estimate, expressed in the path frame.  Positive
 * correction_error_m means command positive path-left velocity to return to
 * the line. */
typedef struct {
    float along_track_progress_m;
    float correction_error_m;
    float distance_to_goal_m;
} PathPlannerLineFeedback;

bool motion_estimate_is_valid(const MotionEstimate *estimate);

/* Builds the shortest straight path from start to the target point. */
esp_err_t path_planner_line_start(
    PathPlannerLine *line,
    const MotionEstimate *start,
    float target_x_m,
    float target_y_m);

/* Converts a source-agnostic pose estimate into line-following feedback. */
esp_err_t path_planner_line_feedback(
    const PathPlannerLine *line,
    const MotionEstimate *estimate,
    PathPlannerLineFeedback *feedback_out);

/* Wraps an angle to [-pi, pi], for geometric heading error only. */
float path_planner_wrap_angle_rad(float angle_rad);

#ifdef __cplusplus
}
#endif
