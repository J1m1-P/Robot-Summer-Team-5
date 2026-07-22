/* Implements MoveC's closed-loop circular-arc controller. */
#include "control/drivetrain/move_c.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

static const float kPi = 3.14159265358979323846f;
static const float kStoppedSpeed = 0.005f;

bool move_c_config_is_valid(const MoveCConfig *c) {
    return c != NULL && off_tape_motion_config_is_valid(&c->off_tape_motion) &&
           speed_profile_config_is_valid(&c->speed_profile) &&
           tape_following_controller_config_is_valid(&c->heading_controller) &&
           isfinite(c->arc_length_tolerance_m) && c->arc_length_tolerance_m >= 0.0f &&
           isfinite(c->radial_tolerance_m) && c->radial_tolerance_m >= 0.0f &&
           isfinite(c->heading_tolerance_rad) && c->heading_tolerance_rad >= 0.0f &&
           isfinite(c->max_accel_mps2) && c->max_accel_mps2 > 0.0f &&
           isfinite(c->max_alpha_rad_s2) && c->max_alpha_rad_s2 > 0.0f &&
           isfinite(c->max_omega_rad_s) && c->max_omega_rad_s > 0.0f;
}

static float phase_for_pose(const MoveC *m, const MotionEstimate *e) {
    return atan2f(e->y_m - m->center_y_m, e->x_m - m->center_x_m);
}

static float signed_phase_delta(float from, float to) {
    return path_planner_wrap_angle_rad(to - from);
}

esp_err_t move_c_start(MoveC *m, const MoveCConfig *c, const MotionEstimate *start,
                       float radius, float arc_angle, float max_speed) {
    if (m == NULL || !move_c_config_is_valid(c) || !motion_estimate_is_valid(start) ||
        !isfinite(radius) || radius <= 0.0f || !isfinite(arc_angle) ||
        fabsf(arc_angle) <= 1.0e-6f || fabsf(arc_angle) > 2.0f * kPi ||
        !isfinite(max_speed) || max_speed <= 0.0f) return ESP_ERR_INVALID_ARG;

    memset(m, 0, sizeof(*m));
    m->config = c;
    m->radius_m = radius;
    m->signed_arc_angle_rad = arc_angle;
    m->arc_length_m = radius * fabsf(arc_angle);
    m->start_heading_rad = start->heading_rad;
    m->final_heading_rad = start->heading_rad + arc_angle;
    const float turn_sign = copysignf(1.0f, arc_angle);
    const float center_x_offset = -sinf(start->heading_rad) * turn_sign * radius;
    const float center_y_offset = cosf(start->heading_rad) * turn_sign * radius;
    m->center_x_m = start->x_m + center_x_offset;
    m->center_y_m = start->y_m + center_y_offset;
    m->start_phase_rad = phase_for_pose(m, start);
    m->last_phase_rad = m->start_phase_rad;
    m->last_heading_rad = start->heading_rad;
    m->initialized_phase = true;
    m->max_speed_mps = max_speed;
    if (off_tape_motion_init(&m->off_tape_motion, &c->off_tape_motion) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    speed_profile_reset(&m->translation_profile, 0.0f);
    speed_profile_reset(&m->omega_profile, 0.0f);
    tape_following_controller_reset(&m->heading_controller);
    m->status = MOVE_C_RUNNING;
    return ESP_OK;
}

esp_err_t move_c_update(MoveC *m, const MotionEstimate *estimate, float dt,
                        MoveCOutput *out) {
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (m == NULL || out == NULL || !motion_estimate_is_valid(estimate) ||
        !isfinite(dt) || dt <= 0.0f) {
        if (m != NULL && m->config != NULL) m->status = MOVE_C_FAULT;
        if (out != NULL) { out->status = MOVE_C_FAULT; out->motion_valid = false; }
        return ESP_ERR_INVALID_ARG;
    }
    if (m->config == NULL || m->status == MOVE_C_IDLE) return ESP_ERR_INVALID_STATE;
    if (m->status == MOVE_C_COMPLETE || m->status == MOVE_C_FAULT) {
        out->status = m->status;
        out->motion_valid = m->status != MOVE_C_FAULT;
        return m->status == MOVE_C_COMPLETE ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (dt > m->config->off_tape_motion.controller_dt_max_s) {
        m->status = MOVE_C_FAULT;
        out->status = MOVE_C_FAULT;
        return ESP_ERR_INVALID_STATE;
    }

    const float phase = phase_for_pose(m, estimate);
    if (!m->initialized_phase) {
        m->last_phase_rad = phase;
        m->initialized_phase = true;
    } else {
        const float turn_sign = copysignf(1.0f, m->signed_arc_angle_rad);
        const float delta = signed_phase_delta(m->last_phase_rad, phase);
        m->progress_m += turn_sign * delta * m->radius_m;
        m->last_phase_rad = phase;
    }
    const float turn_sign = copysignf(1.0f, m->signed_arc_angle_rad);
    const float radial_distance = hypotf(estimate->x_m - m->center_x_m,
                                         estimate->y_m - m->center_y_m);
    const float radial_error = radial_distance - m->radius_m;
    const float remaining = m->arc_length_m - m->progress_m;
    out->remaining_arc_m = remaining;
    out->radial_error_m = radial_error;

    float stop_distance = 0.0f;
    if (speed_profile_predict_stopping_distance(&m->translation_profile,
            &m->config->speed_profile, m->config->max_accel_mps2, dt,
            kStoppedSpeed, &stop_distance) != ESP_OK) {
        m->status = MOVE_C_FAULT;
        out->status = MOVE_C_FAULT;
        return ESP_ERR_INVALID_STATE;
    }
    const bool braking = remaining <= stop_distance + m->config->arc_length_tolerance_m;
    float speed = 0.0f;
    if (speed_profile_update(&m->translation_profile, &m->config->speed_profile,
            braking ? 0.0f : m->max_speed_mps, m->config->max_accel_mps2,
            dt, &speed) != ESP_OK) {
        m->status = MOVE_C_FAULT;
        out->status = MOVE_C_FAULT;
        return ESP_ERR_INVALID_STATE;
    }
    speed = clamp(speed, 0.0f, m->max_speed_mps);
    /* Positive path-left is outward for a CCW arc and inward for a CW arc.
     * Negate the signed radial error so the PID correction always points
     * back toward the commanded circle. */
    const OffTapeMotionInput correction_input = {
        .error = -turn_sign * radial_error,
        .travel_velocity_mps = speed,
    };
    OffTapeMotionOutput correction = {};
    if (off_tape_motion_update(&m->off_tape_motion, &correction_input, dt,
                               &correction) != ESP_OK || !correction.motion_valid) {
        m->status = MOVE_C_FAULT;
        out->status = MOVE_C_FAULT;
        return ESP_ERR_INVALID_STATE;
    }

    const float progress_fraction = clamp(m->progress_m / m->arc_length_m, 0.0f, 1.0f);
    const float desired_heading = m->start_heading_rad +
        m->signed_arc_angle_rad * progress_fraction;
    out->heading_error_rad = path_planner_wrap_angle_rad(
        desired_heading - estimate->heading_rad);
    const float heading_feedback = tape_following_controller_update(
        &m->heading_controller, &m->config->heading_controller,
        out->heading_error_rad, dt);
    float omega_target = turn_sign * speed / m->radius_m + heading_feedback;
    omega_target = clamp(omega_target, -m->config->max_omega_rad_s,
                         m->config->max_omega_rad_s);
    float omega = 0.0f;
    if (speed_profile_update(&m->omega_profile, &m->config->speed_profile,
            omega_target, m->config->max_alpha_rad_s2, dt, &omega) != ESP_OK) {
        m->status = MOVE_C_FAULT;
        out->status = MOVE_C_FAULT;
        return ESP_ERR_INVALID_STATE;
    }
    omega = clamp(omega, -m->config->max_omega_rad_s, m->config->max_omega_rad_s);

    const float tangent_heading = desired_heading;
    const float tangent_c = cosf(tangent_heading), tangent_s = sinf(tangent_heading);
    const float world_x = correction.requested_velocity.vx * tangent_c -
                          correction.requested_velocity.vy * tangent_s;
    const float world_y = correction.requested_velocity.vx * tangent_s +
                          correction.requested_velocity.vy * tangent_c;
    const float c_heading = cosf(estimate->heading_rad), s_heading = sinf(estimate->heading_rad);
    out->requested_velocity.vx = c_heading * world_x + s_heading * world_y;
    out->requested_velocity.vy = -s_heading * world_x + c_heading * world_y;
    out->requested_velocity.omega = omega;

    if (braking && speed <= kStoppedSpeed &&
        fabsf(remaining) <= m->config->arc_length_tolerance_m &&
        fabsf(radial_error) <= m->config->radial_tolerance_m &&
        fabsf(out->heading_error_rad) <= m->config->heading_tolerance_rad &&
        fabsf(omega) <= kStoppedSpeed) {
        m->status = MOVE_C_COMPLETE;
    }
    if (m->status == MOVE_C_COMPLETE || m->status == MOVE_C_FAULT) {
        out->requested_velocity.vx = 0.0f;
        out->requested_velocity.vy = 0.0f;
        out->requested_velocity.omega = 0.0f;
    }
    out->status = m->status;
    out->motion_valid = m->status != MOVE_C_FAULT;
    return ESP_OK;
}
