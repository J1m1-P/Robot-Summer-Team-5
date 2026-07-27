/* Declares a fused wheel-encoder + optical world-frame pose tracker. */
#pragma once

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
 * them in each cycle, matching every other pure control module here. */
typedef struct {
    DrivetrainOdometry odometry;
    DrivetrainOdometrySource encoder_source;
    Pmw3610OdometrySource optical_source;
    DrivetrainOdometrySourceConfig encoder_config;
} PoseTracker;

// Zero-initialize `tracker` before calling (PoseTracker tracker = {0};).
esp_err_t pose_tracker_init(PoseTracker *tracker,
                            const DrivetrainOdometrySourceConfig *encoder_config);

// Clears cumulative pose and both sources' tracked baselines.
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

#ifdef __cplusplus
}
#endif
