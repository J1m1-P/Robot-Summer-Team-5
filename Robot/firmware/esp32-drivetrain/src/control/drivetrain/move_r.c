/* Implements closed-loop in-place heading control. */
#include "control/drivetrain/move_r.h"

#include <math.h>
#include <string.h>

#include <robot_common/math_utils.h>

static const float kStoppedOmegaRadS = 0.005f;
static const float kPi = 3.14159265358979323846f;

bool move_r_config_is_valid(const MoveRConfig *config)
{
    return config != NULL &&
           bounded_pid_config_is_valid(&config->heading_controller) &&
           speed_profile_config_is_valid(&config->speed_profile) &&
           isfinite(config->heading_tolerance_rad) &&
           config->heading_tolerance_rad >= 0.0f &&
           isfinite(config->max_alpha_rad_s2) && config->max_alpha_rad_s2 > 0.0f &&
           isfinite(config->max_omega_rad_s) && config->max_omega_rad_s > 0.0f &&
           isfinite(config->controller_dt_max_s) && config->controller_dt_max_s > 0.0f;
}

static float wrap_heading_error(float error)
{
    return remainderf(error, 2.0f * kPi);
}

esp_err_t move_r_start(
    MoveR *move,
    const MoveRConfig *config,
    const MotionEstimate *start,
    float target_heading_rad)
{
    if (move == NULL || !move_r_config_is_valid(config) ||
        !motion_estimate_is_valid(start) || !isfinite(target_heading_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(move, 0, sizeof(*move));
    move->config = config;
    move->target_heading_rad = target_heading_rad;
    move->status = MOVE_R_RUNNING;
    speed_profile_reset(&move->profile, 0.0f);
    return bounded_pid_reset(&move->heading_state);
}

esp_err_t move_r_update(
    MoveR *move,
    const MotionEstimate *estimate,
    float dt_s,
    MoveROutput *output)
{
    if (output != NULL) memset(output, 0, sizeof(*output));
    if (move == NULL || output == NULL || move->config == NULL ||
        !motion_estimate_is_valid(estimate) || !isfinite(dt_s) || dt_s <= 0.0f) {
        if (move != NULL) move->status = MOVE_R_FAULT;
        if (output != NULL) output->status = MOVE_R_FAULT;
        return ESP_ERR_INVALID_ARG;
    }
    if (move->status == MOVE_R_COMPLETE || move->status == MOVE_R_FAULT) {
        output->status = move->status;
        return move->status == MOVE_R_COMPLETE ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (dt_s > move->config->controller_dt_max_s) {
        move->status = MOVE_R_FAULT;
        output->status = MOVE_R_FAULT;
        return ESP_ERR_INVALID_STATE;
    }

    const float error = wrap_heading_error(
        move->target_heading_rad - estimate->heading_rad);
    output->heading_error_rad = error;
    const float feedback = bounded_pid_update(
        &move->heading_state, &move->config->heading_controller, error, dt_s);
    const float target_omega = fabsf(error) <= move->config->heading_tolerance_rad
        ? 0.0f
        : clamp(feedback, -move->config->max_omega_rad_s,
                move->config->max_omega_rad_s);

    float omega = 0.0f;
    const esp_err_t profile_error = speed_profile_update(
        &move->profile, &move->config->speed_profile, target_omega,
        move->config->max_alpha_rad_s2, dt_s, &omega);
    if (profile_error != ESP_OK) return profile_error;
    omega = clamp(omega, -move->config->max_omega_rad_s,
                  move->config->max_omega_rad_s);
    move->profile.commanded_speed_mps = omega;

    output->requested_velocity.vx = 0.0f;
    output->requested_velocity.vy = 0.0f;
    output->requested_velocity.omega = omega;

    if (fabsf(error) <= move->config->heading_tolerance_rad &&
        fabsf(omega) <= kStoppedOmegaRadS) {
        move->status = MOVE_R_COMPLETE;
        output->requested_velocity.omega = 0.0f;
    } else if (fabsf(error) > move->config->heading_tolerance_rad &&
               fabsf(target_omega) < rotational_settle_deadband_rad_s() &&
               fabsf(omega) < rotational_settle_deadband_rad_s()) {
        /* Both the PID's own target and the profile-tracked actual have
         * dropped below the angular deadband (see rotational_settle.h)
         * while heading error is still outside tolerance -- unlike a
         * linear move's along-track profile, which intentionally brakes to
         * exactly zero as a real maneuver end-state, this PID can converge
         * to a steady sub-deadband value on its own with no separate
         * "am I braking" signal to gate on, so continuing to feed it would
         * be physically inert and the primitive would sit in RUNNING
         * forever. Checking target_omega too (not just omega) excludes the
         * initial jerk-limited ramp-up from a cold start, where target_omega
         * is already near max_omega_rad_s (correctly not yet "small") even
         * though the profile-tracked omega briefly passes through small
         * values on its way up. Pulse above the deadband instead, the same
         * pattern MoveL/MoveC use for their own linear residual correction. */
        if (!move->settling) {
            move->settling = true;
            rotational_settle_reset(&move->settle);
        }
        const float hold_omega = rotational_settle_update(
            &move->settle, fabsf(error),
            move->config->heading_tolerance_rad, dt_s);
        output->requested_velocity.omega = copysignf(hold_omega, error);
    }
    output->status = move->status;
    output->motion_valid = move->status != MOVE_R_FAULT;
    return ESP_OK;
}
