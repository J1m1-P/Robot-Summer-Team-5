/* Declares the stateful tape-following behavior used by the robot manager. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/velocity_kinematics.h"
#include "control/tape_following/tape_following_controller.h"
#include "control/tape_following/tape_following_kinematics.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include "sensing/tape_following/tape_line_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Describes the current operating state of the tape-following behavior.
 * Hardware and software failures are returned separately as esp_err_t values. */
typedef enum {
    TAPE_FOLLOWER_IDLE = 0,
    TAPE_FOLLOWER_TRACKING,
    TAPE_FOLLOWER_SEARCHING,
    TAPE_FOLLOWER_LOST,
} TapeFollowerStatus;

/* Indexes direction-specific sensors, estimators, and runtime history. */
typedef enum {
    TAPE_FOLLOWER_FRONT = 0,
    TAPE_FOLLOWER_BACK,
    TAPE_FOLLOWER_SENSOR_COUNT,
} TapeFollowerSensor;

/* Behavior used after a previously acquired line disappears. */
typedef struct {
    float velocity_mps;
    float timeout_s;
} TapeFollowerSearchConfig;

/* Combines estimator geometry, feedback gains, and lost-tape behavior.
 *
 * The front and back estimators should use weights whose signs follow the
 * drivetrain body convention: negative is left and positive is right.
 */
typedef struct {
    const TapeLineEstimatorConfig *estimators[TAPE_FOLLOWER_SENSOR_COUNT];
    TapeFollowingControllerConfig controller;
    TapeFollowingKinematicsConfig heading;
    TapeFollowerSearchConfig search;

    /* Upper bound used for PID integration and differentiation after loop stalls. */
    float controller_dt_max_s;
} TapeFollowerConfig;

/* Supplies both guidance modules and the requested signed travel velocity.
 * Positive travel uses the front sensor; negative travel uses the back sensor.
 * Sensor sampling remains outside this module so it can be tested without GPIO. */
typedef struct {
    const TapeSensor *sensors[TAPE_FOLLOWER_SENSOR_COUNT];
    float travel_velocity_mps;
} TapeFollowerInput;

/* Returns a command compatible with drivetrain_set_body_velocity plus useful
 * diagnostic values. Apply requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    TapeFollowerStatus status;
    float line_error;
    bool motion_valid;
} TapeFollowerOutput;

/* Retains estimator, controller, and tape-loss history for one behavior instance. */
typedef struct {
    const TapeFollowerConfig *config;
    TapeLineEstimatorState estimator_states[TAPE_FOLLOWER_SENSOR_COUNT];
    TapeFollowingControllerState controller_state;
    float lost_elapsed_s;
    float requested_omega_rad_s;
    int8_t active_direction;
    bool ever_tracked[TAPE_FOLLOWER_SENSOR_COUNT];
} TapeFollower;

/* Validates the configuration and prepares an idle tape follower.
 * Zero-initialize the runtime object before its first init call:
 * TapeFollower follower = {0}; */
esp_err_t tape_follower_init(TapeFollower *follower,
                             const TapeFollowerConfig *config);

/* Clears all estimator, PID, and tape-loss history without changing config. */
esp_err_t tape_follower_reset(TapeFollower *follower);

/* Calculates one motion request using the sensor leading the requested travel.
 * This function never commands the drivetrain directly. */
esp_err_t tape_follower_update(TapeFollower *follower,
                               const TapeFollowerInput *input,
                               float dt_s,
                               TapeFollowerOutput *output);

#ifdef __cplusplus
}
#endif
