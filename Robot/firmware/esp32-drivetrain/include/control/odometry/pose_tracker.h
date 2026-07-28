/* Declares a fused wheel-encoder + optical world-frame pose tracker. */
#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/drivetrain_odometry_source.h"
#include "control/drivetrain/odometry.h"
#include "control/drivetrain/pmw3610_odometry_source.h"

#ifdef __cplusplus
extern "C" {
#endif

// World-frame robot pose in caller-friendly units (meters, radians).
typedef struct {
    float x_m;
    float y_m;
    float heading_rad;
} Pose;

/* Fuses wheel-encoder and PMW3610-optical odometry into one cumulative
 * pose: prefers a fresh optical sample each cycle, falling back to encoder
 * dead reckoning when none is available. Never touches hardware directly --
 * the caller reads wheel counts and the latest optical packet and passes
 * them in each cycle, matching every other pure control module here.
 *
 * Each optical sample is ground truth, not just an increment: optical_source
 * only reports the body-frame delta since the *previous* optical sample, so
 * composing it onto whatever the running pose currently is would leave any
 * encoder-fallback drift accrued in between permanently baked in. Instead
 * optical_anchor_pose tracks the pose as of the last optical fix; a fresh
 * optical delta composes from that anchor and overwrites (not adds to) the
 * running pose, discarding any intervening encoder drift outright. */
typedef struct {
    DrivetrainOdometry odometry;
    DrivetrainOdometrySource encoder_source;
    Pmw3610OdometrySource optical_source;
    DrivetrainOdometrySourceConfig encoder_config;
    DrivetrainPose optical_anchor_pose;
    // Diagnostic only: counts which branch pose_tracker_update() actually
    // took each cycle, so a caller can tell "optical is arriving but not
    // being used" apart from "optical genuinely isn't arriving often".
    uint32_t optical_update_count;
    uint32_t encoder_update_count;
} PoseTracker;

// Zero-initialize `tracker` before calling (PoseTracker tracker = {0};).
esp_err_t pose_tracker_init(PoseTracker *tracker,
                            const DrivetrainOdometrySourceConfig *encoder_config);

// Clears cumulative pose, both sources' tracked baselines, and counters.
void pose_tracker_reset(PoseTracker *tracker);

/* Runs one fusion cycle. `wheel_counts` must be this cycle's accumulated
 * encoder counts. `optical_packet` is the latest decoded PMW3610 sample --
 * pass NULL if the optical link isn't wired up in this build, or if nothing
 * new has arrived; the tracker falls back to encoder dead reckoning
 * whenever the optical sample is missing, stale, or invalid. Call once per
 * control cycle. */
esp_err_t pose_tracker_update(PoseTracker *tracker,
                              const DrivetrainWheelCounts *wheel_counts,
                              const OdometryPacket *optical_packet);

// Returns the current cumulative world-frame pose.
Pose pose_tracker_get_pose(const PoseTracker *tracker);

// Re-anchors the fused pose without resetting encoder or optical baselines.
// The next delta is applied from this corrected world-frame pose.
esp_err_t pose_tracker_set_pose(PoseTracker *tracker, const Pose *pose);

#ifdef __cplusplus
}
#endif
