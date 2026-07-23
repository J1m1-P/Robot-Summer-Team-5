/* Declares the shared endpoint-settle pulse/pause hold controller for
 * angular (heading) residual error -- the rotational analog of
 * endpoint_settle.h. Kept as a separate module rather than generalizing
 * endpoint_settle itself so MoveL/MoveC's already-verified linear settle
 * behavior is untouched. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Retains the pulse/pause timers between cycles. */
typedef struct {
    float pulse_remaining_s;
    float pause_remaining_s;
} RotationalSettleState;

/* Clears the pulse/pause timers. Call once when entering the settle phase. */
void rotational_settle_reset(RotationalSettleState *state);

/* Returns the derived angular deadband (see rotational_settle_update()'s
 * doc comment for the full derivation) so callers can detect "the
 * commanded omega has dropped below what the wheels can actually achieve"
 * themselves -- e.g. to decide when to stop driving continuously and start
 * calling rotational_settle_update() instead. Unlike a linear move's
 * along-track profile, which intentionally brakes its own axis to exactly
 * zero as a real maneuver end-state, MoveR's PID output can converge to a
 * steady sub-deadband (but not near-zero) value on its own -- there is no
 * separate "am I braking" signal to gate on, so the deadband itself is the
 * only reliable trigger. */
float rotational_settle_deadband_rad_s(void);

/* Once a rotation's own speed profile has decayed to a stop, a small
 * residual heading error can produce a continuous PID correction below the
 * angular equivalent of the characterized ~0.05 m/s linear wheel-velocity
 * floor -- physically inert (the wheels can't reliably break static
 * friction at that commanded rate), so the error would never shrink and
 * the primitive would sit in RUNNING forever. Worse, a controller that
 * hard-switches its target to exactly zero at the tolerance boundary (as
 * move_r_update() does) combined with this deadband produces chatter right
 * at the edge rather than a clean stop. This closes the gap the same way
 * endpoint_settle.h does for linear residual error: a short burst clearly
 * above the deadband, then a pause for a fresh estimate, repeated only
 * while still outside tolerance.
 *
 * The deadband floor this module's constants are built around is derived
 * (not guessed) from the characterized linear wheel floor via the X-drive
 * geometry: for a pure in-place rotation every wheel's linear surface
 * speed equals `arm * |omega|`, where
 * `arm = chassis_half_length_m * sin(wheel_angle_rad) +
 *        chassis_half_width_m * cos(wheel_angle_rad)`
 * (see x_drive_kinematics.c's Jacobian). Using
 * DRIVETRAIN_CONFIG.x_drive_kinematics's values (0.100, 0.1365, 30 deg):
 * arm = 0.168212... m, so omega_floor = 0.05 / arm = 0.297243 rad/s
 * (~17.03 deg/s). See rotational_settle.c for where that number is used.
 *
 * `error_magnitude_rad` is the caller's nonnegative remaining heading error
 * (e.g. MoveR's |heading_error_rad|); `tolerance_rad` is the caller's own
 * tolerance below which no correction is issued. Returns a nonnegative
 * hold-omega magnitude to apply this cycle; the caller supplies direction.
 * Zero while paused between pulses or once within tolerance. */
float rotational_settle_update(RotationalSettleState *state,
                               float error_magnitude_rad,
                               float tolerance_rad,
                               float dt_s);

#ifdef __cplusplus
}
#endif
