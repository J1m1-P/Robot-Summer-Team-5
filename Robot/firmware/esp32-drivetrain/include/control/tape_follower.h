/* Declares the stateful tape-following behavior used by the robot manager. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain_kinematics.h"
#include "control/tape_following_controller.h"
#include "drivers/tape_sensor_driver.h"
#include "sensing/tape_line_estimator.h"

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

    /* Lateral body duty used to search toward the last observed tape position. */
    float search_duty;

    /* Maximum continuous search duration before reporting TAPE_FOLLOWER_LOST. */
    float lost_timeout_s;

    /* Upper bound used for PID integration and differentiation after loop stalls. */
    float controller_dt_max_s;

} TapeFollowerConfig;

/* Supplies both guidance modules and the requested signed travel duty.
 * Positive travel uses the front sensor; negative travel uses the back sensor.
 * Sensor sampling remains outside this module so it can be tested without GPIO. */
typedef struct {
    const TapeSensor *front_sensor;
    const TapeSensor *back_sensor;
    float travel_duty;
} TapeFollowerInput;

/* Returns the motion requested by the behavior and useful diagnostic values.
 * The robot manager should apply requested_motion only when motion_valid is true. */
typedef struct {
    DrivetrainBodyDuty requested_motion;
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
