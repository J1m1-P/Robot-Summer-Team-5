/* Implements MoveP's synchronized line-and-heading controller. */
#include "control/drivetrain/move_p.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

static const float kStoppedSpeed = 0.005f;
static const float kHeadingBrakeMargin = 1.25f;

bool move_p_config_is_valid(const MovePConfig *c) {
    return c != NULL && off_tape_motion_config_is_valid(&c->off_tape_motion) &&
           speed_profile_config_is_valid(&c->speed_profile) &&
           bounded_pid_config_is_valid(&c->heading_controller) &&
           isfinite(c->distance_tolerance_m) && c->distance_tolerance_m >= 0.0f &&
           isfinite(c->heading_tolerance_rad) && c->heading_tolerance_rad >= 0.0f &&
           isfinite(c->max_accel_mps2) && c->max_accel_mps2 > 0.0f &&
           isfinite(c->max_alpha_rad_s2) && c->max_alpha_rad_s2 > 0.0f &&
           isfinite(c->max_omega_rad_s) && c->max_omega_rad_s > 0.0f;
}

esp_err_t move_p_start(MoveP *m, const MovePConfig *c, const MotionEstimate *start,
                       float tx, float ty, float final_heading, float max_speed) {
    if (m == NULL || !move_p_config_is_valid(c) || !motion_estimate_is_valid(start) ||
        !isfinite(tx) || !isfinite(ty) || !isfinite(final_heading) ||
        !isfinite(max_speed) || max_speed <= 0.0f) return ESP_ERR_INVALID_ARG;
    memset(m, 0, sizeof(*m));
    m->config = c;
    const esp_err_t line_error = path_planner_line_start(&m->line, start, tx, ty);
    if (line_error != ESP_OK) return line_error;
    const esp_err_t pid_error = off_tape_motion_init(&m->off_tape_motion, &c->off_tape_motion);
    if (pid_error != ESP_OK) return pid_error;
    speed_profile_reset(&m->translation_profile, 0.0f);
    speed_profile_reset(&m->omega_profile, 0.0f);
    m->start_heading_rad = start->heading_rad;
    m->final_heading_rad = final_heading;
    m->heading_delta_rad = path_planner_wrap_angle_rad(final_heading - start->heading_rad);
    m->max_speed_mps = max_speed;
    m->status = MOVE_P_TRANSLATE_AND_TURN;
    return ESP_OK;
}

esp_err_t move_p_update(MoveP *m, const MotionEstimate *estimate, float dt, MovePOutput *out) {
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (m == NULL || out == NULL || !motion_estimate_is_valid(estimate) || !isfinite(dt) || dt <= 0.0f) {
        if (m != NULL && m->config != NULL) m->status = MOVE_P_FAULT;
        if (out != NULL) out->status = MOVE_P_FAULT;
        return ESP_ERR_INVALID_ARG;
    }
    if (m->config == NULL) return ESP_ERR_INVALID_STATE;
    if (m->status == MOVE_P_COMPLETE || m->status == MOVE_P_FAULT) {
        out->status = m->status;
        return m->status == MOVE_P_COMPLETE ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (dt > m->config->off_tape_motion.controller_dt_max_s) {
        m->status = MOVE_P_FAULT;
        out->status = MOVE_P_FAULT;
        return ESP_ERR_INVALID_STATE;
    }
    PathPlannerLineFeedback f = {};
    const esp_err_t feedback_error = path_planner_line_feedback(&m->line, estimate, &f);
    if (feedback_error != ESP_OK) return feedback_error;
    out->distance_to_goal_m = f.distance_to_goal_m;

    float speed = 0.0f;
    float omega_target = 0.0f;
    float world_x = 0.0f, world_y = 0.0f;
    if (m->status == MOVE_P_TRANSLATE_AND_TURN) {
        const float remaining = m->line.length_m - f.along_track_progress_m;
        if (!m->braking_translation) {
            float stop_distance = 0.0f;
            const esp_err_t e = speed_profile_predict_stopping_distance(&m->translation_profile,
                &m->config->speed_profile, m->config->max_accel_mps2, dt, kStoppedSpeed, &stop_distance);
            if (e != ESP_OK) return e;
            if (remaining <= stop_distance + m->config->distance_tolerance_m) m->braking_translation = true;
        }
        const esp_err_t e = speed_profile_update(&m->translation_profile, &m->config->speed_profile,
            m->braking_translation ? 0.0f : m->max_speed_mps, m->config->max_accel_mps2, dt, &speed);
        if (e != ESP_OK) return e;
        speed = clamp(speed, 0.0f, m->max_speed_mps);
        m->translation_profile.commanded_speed_mps = speed;
        OffTapeMotionOutput correction = {};
        const OffTapeMotionInput input = {.error = f.correction_error_m, .travel_velocity_mps = speed};
        const esp_err_t correction_error = off_tape_motion_update(&m->off_tape_motion, &input, dt, &correction);
        if (correction_error != ESP_OK) return correction_error;
        /* Path left is (-uy, ux), matching positive body vy = left. */
        world_x = correction.requested_velocity.vx * m->line.direction_x +
                  correction.requested_velocity.vy * -m->line.direction_y;
        world_y = correction.requested_velocity.vx * m->line.direction_y +
                  correction.requested_velocity.vy * m->line.direction_x;
        const float fraction = clamp(f.along_track_progress_m / m->line.length_m, 0.0f, 1.0f);
        const float desired_heading = m->start_heading_rad + m->heading_delta_rad * fraction;
        out->heading_error_rad = path_planner_wrap_angle_rad(desired_heading - estimate->heading_rad);
        const float heading_feedback = bounded_pid_update(&m->heading_controller,
            &m->config->heading_controller, out->heading_error_rad, dt);
        omega_target = speed * m->heading_delta_rad / m->line.length_m + heading_feedback;
        float heading_stop_distance = 0.0f;
        const esp_err_t heading_stop_error = speed_profile_predict_stopping_distance(
            &m->omega_profile, &m->config->speed_profile,
            m->config->max_alpha_rad_s2, dt, kStoppedSpeed,
            &heading_stop_distance);
        if (heading_stop_error != ESP_OK) return heading_stop_error;
        const float planned_heading_remaining = fabsf(
            m->final_heading_rad - desired_heading);
        if (planned_heading_remaining <=
            kHeadingBrakeMargin * heading_stop_distance +
            m->config->heading_tolerance_rad) {
            /* Brake before the final heading. A small undershoot is safer
             * than carrying angular momentum through the endpoint; the
             * settle phase performs the final correction. */
            omega_target = 0.0f;
        }
        if (m->braking_translation && speed <= kStoppedSpeed) {
            /* The one-way profile is deliberately allowed to undershoot.
             * Enter the bounded endpoint-adjustment/heading-settle phase even
             * when the along-track residual is larger than tolerance; the
             * position-hold correction below closes that final gap. */
            bounded_pid_reset(&m->heading_controller);
            endpoint_settle_reset(&m->settle);
            m->status = MOVE_P_SETTLE_HEADING;
        }
    }
    if (m->status == MOVE_P_SETTLE_HEADING) {
        out->heading_error_rad = path_planner_wrap_angle_rad(m->final_heading_rad - estimate->heading_rad);
        float heading_stop_distance = 0.0f;
        const esp_err_t stop_error = speed_profile_predict_stopping_distance(
            &m->omega_profile, &m->config->speed_profile,
            m->config->max_alpha_rad_s2, dt, kStoppedSpeed,
            &heading_stop_distance);
        if (stop_error != ESP_OK) return stop_error;
        if (fabsf(out->heading_error_rad) > kHeadingBrakeMargin * heading_stop_distance +
            m->config->heading_tolerance_rad) {
            omega_target = bounded_pid_update(
                &m->heading_controller, &m->config->heading_controller,
                out->heading_error_rad, dt);
        } else {
            /* Brake to zero before the target heading; do not let the
             * jerk-limited profile carry the robot through the endpoint. */
            omega_target = 0.0f;
        }

        /* Rotation is not perfectly holonomic in practice: wheel mismatch
         * can translate the chassis while heading settles. Hold the endpoint
         * with a small bounded world-frame correction so that this phase
         * cannot turn a valid arrival into a lateral miss. */
        const float target_x = m->line.start_x_m +
                               m->line.direction_x * m->line.length_m;
        const float target_y = m->line.start_y_m +
                               m->line.direction_y * m->line.length_m;
        const float hold_x = target_x - estimate->x_m;
        const float hold_y = target_y - estimate->y_m;
        const float hold_distance = hypotf(hold_x, hold_y);
        const float hold_speed = endpoint_settle_update(
            &m->settle, hold_distance, m->config->distance_tolerance_m, dt);
        if (hold_speed > 0.0f) {
            world_x = hold_x / hold_distance * hold_speed;
            world_y = hold_y / hold_distance * hold_speed;
        }
    }
    omega_target = clamp(omega_target, -m->config->max_omega_rad_s, m->config->max_omega_rad_s);
    float omega = 0.0f;
    const esp_err_t omega_error = speed_profile_update(&m->omega_profile, &m->config->speed_profile,
        omega_target, m->config->max_alpha_rad_s2, dt, &omega);
    if (omega_error != ESP_OK) return omega_error;
    omega = clamp(omega, -m->config->max_omega_rad_s, m->config->max_omega_rad_s);
    m->omega_profile.commanded_speed_mps = omega;
    const float c = cosf(estimate->heading_rad), s = sinf(estimate->heading_rad);
    out->requested_velocity.vx = c * world_x + s * world_y;
    out->requested_velocity.vy = -s * world_x + c * world_y;
    out->requested_velocity.omega = omega;
    if (m->status == MOVE_P_SETTLE_HEADING &&
        out->distance_to_goal_m <= m->config->distance_tolerance_m &&
        fabsf(out->heading_error_rad) <= m->config->heading_tolerance_rad &&
        fabsf(omega) <= kStoppedSpeed)
        m->status = MOVE_P_COMPLETE;
    if (m->status == MOVE_P_COMPLETE || m->status == MOVE_P_FAULT) {
        out->requested_velocity.vx = 0.0f;
        out->requested_velocity.vy = 0.0f;
        out->requested_velocity.omega = 0.0f;
    }
    out->status = m->status;
    out->motion_valid = m->status != MOVE_P_FAULT;
    return ESP_OK;
}
