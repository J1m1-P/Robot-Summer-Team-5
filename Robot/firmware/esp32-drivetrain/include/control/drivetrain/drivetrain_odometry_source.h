/* Declares the encoder-tick to world-frame odometry bridge. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/odometry.h"
#include "control/drivetrain/x_drive_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raw accumulated encoder counts, one per wheel. The caller reads these from
 * drivetrain_get_encoder_accumulated_count() each cycle -- this module never
 * touches hardware directly, matching every other pure control module. */
typedef struct {
    int32_t fl;
    int32_t fr;
    int32_t bl;
    int32_t br;
} DrivetrainWheelCounts;

/* Combines wheel kinematics and per-wheel encoder resolution needed to
 * convert raw tick deltas into a body-frame motion delta each cycle. */
typedef struct {
    XDriveKinematicsConfig x_drive_kinematics;
    uint32_t counts_per_revolution_fl;
    uint32_t counts_per_revolution_fr;
    uint32_t counts_per_revolution_bl;
    uint32_t counts_per_revolution_br;
} DrivetrainOdometrySourceConfig;

// Rejects configurations that could produce undefined or unsafe behavior.
bool drivetrain_odometry_source_config_is_valid(
    const DrivetrainOdometrySourceConfig *config);

/* Tracks the wheel counts observed on the previous update, so each call can
 * diff against them to get this cycle's tick delta. */
typedef struct {
    DrivetrainWheelCounts previous_counts;
    bool has_previous_counts;
} DrivetrainOdometrySource;

// Clears the tracked previous-count baseline (not the odometry pose itself).
void drivetrain_odometry_source_reset(DrivetrainOdometrySource *source);

/* Converts this cycle's raw wheel counts into a body-frame delta and
 * integrates it into `odometry` via drivetrain_odometry_update(). The first
 * call after reset only captures a baseline -- there is no previous count to
 * diff against yet, so it integrates nothing and still returns ESP_OK, per
 * the "nothing to do yet is not an error" convention every drivetrain update
 * function must follow. */
esp_err_t drivetrain_odometry_source_update(
    DrivetrainOdometrySource *source,
    const DrivetrainOdometrySourceConfig *config,
    const DrivetrainWheelCounts *counts,
    DrivetrainOdometry *odometry);

#ifdef __cplusplus
}
#endif
