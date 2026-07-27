/* Implements fused encoder and optical world-frame pose tracking. */
#include "control/odometry/pose_tracker.h"

#include <stddef.h>
#include <string.h>

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

    DrivetrainOdometryDelta optical_delta = {0};
    if (pmw3610_odometry_source_update(&tracker->optical_source, optical_packet,
                                       &optical_delta)) {
        return drivetrain_odometry_update(&tracker->odometry, &optical_delta, true);
    }

    if (!has_encoder_delta) {
        return ESP_OK;
    }
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
