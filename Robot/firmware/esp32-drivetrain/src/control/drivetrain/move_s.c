/* Implements MoveS: an open-loop, dead-reckoned straight-line move. */
#include "control/drivetrain/move_s.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Below this commanded speed, the jerk-bounded ramp is considered to have
 * actually reached zero rather than just asymptotically approaching it. */
static const float kStoppedSpeedMps = 0.005f;

static bool pose_is_finite(const DrivetrainPose *pose)
{
    return pose != NULL && isfinite(pose->x_mm) && isfinite(pose->y_mm) &&
           isfinite(pose->heading_rad);
}

bool move_s_config_is_valid(const MoveSConfig *config)
{
    return config != NULL &&
           speed_profile_config_is_valid(&config->speed_profile) &&
           isfinite(config->distance_tolerance_m) &&
           config->distance_tolerance_m >= 0.0f;
}

esp_err_t move_s_start(
    MoveS *move,
    const MoveSConfig *config,
    const DrivetrainPose *start_pose,
    float distance_m,
    float heading_rad,
    float max_speed_mps,
    float max_accel_mps2)
{
    if (move == NULL || !move_s_config_is_valid(config) ||
        !pose_is_finite(start_pose) ||
        !isfinite(distance_m) || distance_m <= 0.0f ||
        !isfinite(heading_rad) ||
        !isfinite(max_speed_mps) || max_speed_mps <= 0.0f ||
        !isfinite(max_accel_mps2) || max_accel_mps2 <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(move, 0, sizeof(*move));
    move->config = config;
    move->start_pose = *start_pose;
    move->distance_m = distance_m;
    move->body_direction_x = cosf(heading_rad);
    move->body_direction_y = sinf(heading_rad);
    const float world_heading_rad = start_pose->heading_rad + heading_rad;
    move->world_direction_x = cosf(world_heading_rad);
    move->world_direction_y = sinf(world_heading_rad);
    move->max_speed_mps = max_speed_mps;
    move->max_accel_mps2 = max_accel_mps2;
    speed_profile_reset(&move->profile, 0.0f);
    move->status = MOVE_S_RUNNING;
    return ESP_OK;
}

esp_err_t move_s_update(
    MoveS *move,
    const DrivetrainPose *current_pose,
    float dt_s,
    MoveSOutput *output)
{
    if (move == NULL || output == NULL || !pose_is_finite(current_pose) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (move->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));

    /* Progress along the world-frame direction captured at start -- fixed
     * for the whole move, not re-derived from the current heading, since
     * MoveS never commands rotation and re-deriving it would silently mask
     * the heading drift §3's F_ang calibration is meant to measure. */
    const float progress_m =
        ((current_pose->x_mm - move->start_pose.x_mm) * move->world_direction_x +
         (current_pose->y_mm - move->start_pose.y_mm) * move->world_direction_y) /
        1000.0f;
    const float remaining_m = move->distance_m - progress_m;
    output->remaining_distance_m = remaining_m;

    const bool within_tolerance = remaining_m <= move->config->distance_tolerance_m;
    const float target_speed_mps = within_tolerance
        ? 0.0f
        : fminf(move->max_speed_mps, sqrtf(fmaxf(0.0f, 2.0f * move->max_accel_mps2 *
                 (remaining_m - move->config->distance_tolerance_m))));

    float commanded_speed_mps = 0.0f;
    const esp_err_t error = speed_profile_update(
        &move->profile, &move->config->speed_profile, target_speed_mps,
        move->max_accel_mps2, dt_s, &commanded_speed_mps);
    if (error != ESP_OK) {
        return error;
    }

    output->requested_velocity.vx = commanded_speed_mps * move->body_direction_x;
    output->requested_velocity.vy = commanded_speed_mps * move->body_direction_y;
    output->requested_velocity.omega = 0.0f;

    if (within_tolerance && fabsf(commanded_speed_mps) <= kStoppedSpeedMps) {
        move->status = MOVE_S_COMPLETE;
    }
    output->status = move->status;
    output->motion_valid = true;
    return ESP_OK;
}
