/* Declares the stateful tape-following behavior used by the robot manager. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/velocity_kinematics.h"
#include "control/tape_following/tape_following_controller.h"
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

/* Combines estimator geometry, feedback gains, and lost-tape behavior.
 *
 * The front and back estimators should use weights whose signs follow the
 * drivetrain body convention: negative is left and positive is right.
 */
typedef struct {
    const TapeLineEstimatorConfig *front_estimator;
    const TapeLineEstimatorConfig *back_estimator;
    TapeFollowingControllerConfig controller;

    /* Converts the lateral/forward velocity angle into angular velocity so
     * the leading edge of the robot turns toward its direction of travel. */
    float heading_gain_s_inv;

    /* Bounds the commanded rotation and its rate of change for smooth turns. */
    float max_omega_rad_s;
    float max_angular_acceleration_rad_s2;

    /* Lateral body velocity used to search toward the last tape position. */
    float search_velocity_mps;

    /* Maximum continuous search duration before reporting TAPE_FOLLOWER_LOST. */
    float lost_timeout_s;

    /* Upper bound used for PID integration and differentiation after loop stalls. */
    float controller_dt_max_s;

} TapeFollowerConfig;

/* Supplies both guidance modules and the requested signed travel velocity.
 * Positive travel uses the front sensor; negative travel uses the back sensor.
 * Sensor sampling remains outside this module so it can be tested without GPIO. */
typedef struct {
    const TapeSensor *front_sensor;
    const TapeSensor *back_sensor;
    float travel_velocity_mps;
} TapeFollowerInput;

/* Returns a command compatible with drivetrain_set_body_velocity plus useful
 * diagnostic values. Apply requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    TapeFollowerStatus status;

    float line_error;
    bool line_present;
    bool using_front_sensor;
    bool motion_valid;
} TapeFollowerOutput;

/* Retains estimator, controller, and tape-loss history for one behavior instance. */
typedef struct {
    const TapeFollowerConfig *config;

    TapeLineEstimatorState front_estimator_state;
    TapeLineEstimatorState back_estimator_state;
    TapeFollowingControllerState controller_state;
    float lost_elapsed_s;
    float requested_omega_rad_s;

    TapeFollowerStatus status;
    int8_t active_direction;
    bool front_ever_tracked;
    bool back_ever_tracked;
    bool initialized;
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
