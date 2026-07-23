/* Declares the shared endpoint-settle pulse/pause hold controller. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Retains the pulse/pause timers between cycles. */
typedef struct {
    float pulse_remaining_s;
    float pause_remaining_s;
} EndpointSettleState;

/* Clears the pulse/pause timers. Call once when entering the settle phase. */
void endpoint_settle_reset(EndpointSettleState *state);

/* Once a move's one-way speed profile has stopped, a small residual error
 * produces a continuous correction below the characterized ~0.05 m/s
 * wheel-velocity floor -- physically inert, so the error would never shrink
 * and the primitive would sit in RUNNING forever. This closes that gap with
 * a short burst clearly above the deadband, then a pause for a fresh
 * estimate, repeated only while still outside tolerance.
 *
 * `error_magnitude_m` is the caller's nonnegative remaining error (e.g.
 * MoveC's |radial_error|, MoveP's hold_distance); `tolerance_m` is the
 * caller's own tolerance below which no correction is issued. Returns a
 * nonnegative hold-speed magnitude to apply this cycle; the caller supplies
 * direction. Zero while paused between pulses or once within tolerance. */
float endpoint_settle_update(EndpointSettleState *state,
                             float error_magnitude_m,
                             float tolerance_m,
                             float dt_s);

#ifdef __cplusplus
}
#endif
