/* Implements fused encoder and optical world-frame pose tracking. */
#include "control/odometry/pose_tracker.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Optical-only coordinate shear from the tape calibration.  Encoder deltas
 * are intentionally left untouched because their axes are already usable.
 * The +x and +y calibration runs define the correction in the body frame
 * before the delta is rotated into the fused world frame. */
static const float kOpticalShearXFromY = -0.09f;
static const float kOpticalShearYFromX = -0.05f;
static const float kOpticalHeadingWeight = 0.25f;
static const float kEncoderHeadingWeight = 1.0f - kOpticalHeadingWeight;

static void apply_optical_shear(DrivetrainOdometryDelta *delta) {
    const float forward = delta->forward_mm;
    const float lateral = delta->lateral_mm;
    delta->forward_mm = forward + kOpticalShearXFromY * lateral;
    delta->lateral_mm = lateral + kOpticalShearYFromX * forward;
}

esp_err_t pose_tracker_init(PoseTracker *tracker,
                            const DrivetrainOdometrySourceConfig *encoder_config)
{
    if (tracker == NULL ||
        !drivetrain_odometry_source_config_is_valid(encoder_config)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(tracker, 0, sizeof(*tracker));
    tracker->encoder_config = *encoder_config;
    return ESP_OK;
}

void pose_tracker_reset(PoseTracker *tracker)
{
    if (tracker == NULL) {
        return;
    }
    drivetrain_odometry_reset(&tracker->odometry);
    drivetrain_odometry_source_reset(&tracker->encoder_source);
    pmw3610_odometry_source_reset(&tracker->optical_source);
    tracker->optical_anchor_pose = (DrivetrainPose){0};
    tracker->encoder_heading_since_optical_rad = 0.0f;
    tracker->optical_update_count = 0;
    tracker->encoder_update_count = 0;
}

esp_err_t pose_tracker_update(PoseTracker *tracker,
                              const DrivetrainWheelCounts *wheel_counts,
                              const OdometryPacket *optical_packet)
{
    if (tracker == NULL || wheel_counts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /**
     * Compute encoder delta to supply new data between optical sensor refreshes over UART.
     */
    DrivetrainOdometryDelta encoder_delta = {0};
    bool has_encoder_delta = false;
    bool encoder_valid = false;
    esp_err_t error = drivetrain_odometry_source_compute_delta(
        &tracker->encoder_source, &tracker->encoder_config, wheel_counts,
        &encoder_delta, &has_encoder_delta, &encoder_valid);
    if (error != ESP_OK) {
        return error;
    }

    if (has_encoder_delta && encoder_valid) {
        tracker->encoder_heading_since_optical_rad +=
            encoder_delta.heading_delta_rad;
    }

    DrivetrainOdometryDelta optical_delta = {0};
    const bool optical_was_primed = tracker->optical_source.has_previous;
    if (pmw3610_odometry_source_update(&tracker->optical_source, optical_packet,
                                       &optical_delta)) {
        apply_optical_shear(&optical_delta);
        optical_delta.heading_delta_rad =
            kOpticalHeadingWeight * optical_delta.heading_delta_rad +
            kEncoderHeadingWeight *
                tracker->encoder_heading_since_optical_rad;
        tracker->encoder_heading_since_optical_rad = 0.0f;
        // Compose from the last-known-good optical anchor, not from
        // tracker->odometry.pose -- that may have drifted via encoder
        // fallback since the last optical fix, and this reading overrides
        // that drift rather than adding on top of it.
        DrivetrainOdometry anchored = {.pose = tracker->optical_anchor_pose};
        const esp_err_t compose_error =
            drivetrain_odometry_update(&anchored, &optical_delta, true);
        if (compose_error != ESP_OK) return compose_error;
        tracker->optical_anchor_pose = anchored.pose;
        ++tracker->optical_update_count;
        return drivetrain_odometry_set_pose(&tracker->odometry, &anchored.pose);
    }

    // The first optical packet only establishes the optical baseline. The
    // encoder heading accumulated before that packet does not cover the same
    // interval as the next optical delta, so discard it after preserving the
    // current encoder fallback update.
    if (!optical_was_primed && tracker->optical_source.has_previous) {
        tracker->encoder_heading_since_optical_rad = 0.0f;
    }

    if (!has_encoder_delta) {
        return ESP_OK;
    }
    ++tracker->encoder_update_count;
    return drivetrain_odometry_update(&tracker->odometry, &encoder_delta, encoder_valid);
}

Pose pose_tracker_get_pose(const PoseTracker *tracker)
{
    if (tracker == NULL) {
        Pose empty = {0};
        return empty;
    }
    Pose pose = {
        .x_m = tracker->odometry.pose.x_mm / 1000.0f,
        .y_m = tracker->odometry.pose.y_mm / 1000.0f,
        .heading_rad = tracker->odometry.pose.heading_rad,
    };
    return pose;
}

esp_err_t pose_tracker_set_pose(PoseTracker *tracker, const Pose *pose)
{
    if (tracker == NULL || pose == NULL ||
        !isfinite(pose->x_m) || !isfinite(pose->y_m) ||
        !isfinite(pose->heading_rad)) {
        return ESP_ERR_INVALID_ARG;
    }

    const DrivetrainPose corrected = {
        .x_mm = pose->x_m * 1000.0f,
        .y_mm = pose->y_m * 1000.0f,
        .heading_rad = pose->heading_rad,
    };
    const esp_err_t error = drivetrain_odometry_set_pose(
        &tracker->odometry, &corrected);
    if (error != ESP_OK) return error;
    tracker->optical_anchor_pose = corrected;
    return ESP_OK;
}
