#ifndef TAPE_PID_H
#define TAPE_PID_H

#include <stdbool.h>
#include "sensors/tape_following.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PID steering framework built on top of sensors/tape_following.h.
 *
 * Each module (TAPE_SENSOR_FRONT/BACK/LEFT) is its own named TapeSensor
 * so tape_pid_compute_error() just takes
 * a pointer to whichever TapeSensor is relevant at that point in the course
 * (front ~90% of the time, back/left only at specific points). Call
 * tape_pid_state_reset() whenever you switch which TapeSensor is driving
 * steering, so stale integral/derivative history from the previous one
 * doesn't leak into the new segment.
 */

#define TAPE_PID_CHANNEL_COUNT 4

/* Position weight assigned to each of a sensor's 4 channels, in
 * sensor_0..sensor_3 order, e.g. {-3, -1, 1, 3} for left_outer..right_outer.
 * Order/sign must match the physical left-to-right layout on your board. */
typedef struct {
    float weights[TAPE_PID_CHANNEL_COUNT];
} TapePidSensorConfig;

/* Tunable PID gains + output/anti-windup limits. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit; /* clamp |integral| to this, to prevent windup */
    float output_min;     /* clamp final correction to [output_min, output_max] */
    float output_max;
} TapePidGains;

/* Running state for one PID "instance". Keep a separate instance per
 * steering source if you ever run more than one concurrently; otherwise a
 * single static instance is fine. */
typedef struct {
    float integral;
    float prev_error;
    bool has_prev_error;   /* false until the first update() call after init/reset */
    bool line_was_present; /* false when the last compute_error() lost the line */
    float last_known_error;
} TapePidState;

/**
 * Zero out integral/derivative history. Call once at startup, and again any
 * time you switch which TapeSensor is driving steering (front -> left, etc.)
 * so the D term doesn't see a spike and the I term doesn't carry over
 * error accumulated against a different sensor.
 */
void tape_pid_state_reset(TapePidState *state);

/**
 * Compute a weighted line-position error from one TapeSensor's 4 channel
 * readings (sensor->sensor_0..sensor_3). Call tape_sensor_read_all_channels()
 * first so those fields are current.
 *
 * Returns true and sets *out_error to the weighted centroid when at least
 * one of sensor_0..sensor_3 is true. Returns false when the line is lost
 * (no channel active) - *out_error is still set, to a fallback value derived
 * from state->last_known_error, so the caller can keep steering sensibly
 * instead of suddenly reading "error = 0" (which would mean "go straight").
 */
bool tape_pid_compute_error(const TapeSensor *sensor,
                             const TapePidSensorConfig *sensor_cfg,
                             TapePidState *state,
                             float *out_error);

/**
 * Core PID step. Given the current error (from tape_pid_compute_error, or
 * any other source) and the time in seconds since the last call, returns a
 * correction value clamped to [gains->output_min, gains->output_max].
 *
 * This function does NOT touch motors - it's pure PID math. Wiring the
 * returned correction into left/right motor speeds is up to you.
 */
float tape_pid_update(TapePidState *state,
                       const TapePidGains *gains,
                       float error,
                       float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* TAPE_PID_H */