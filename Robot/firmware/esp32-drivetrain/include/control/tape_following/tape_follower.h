/* Declares the stateful tape-following behavior used by the robot manager. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/off_tape_motion.h"
#include "control/drivetrain/x_drive_kinematics.h"
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
    TAPE_FOLLOWER_SIDE,
    TAPE_FOLLOWER_SENSOR_COUNT,
} TapeFollowerSensor;

/* Robot-relative tape travel directions. The side sensor is +y and is the
 * py leading sensor; its channel polarity matches px after a 90-degree CCW
 * rotation of the sensor frame. */
typedef enum {
    TAPE_FOLLOWER_PX = 0,
    TAPE_FOLLOWER_MX,
    TAPE_FOLLOWER_PY,
    TAPE_FOLLOWER_DIRECTION_COUNT,
} TapeFollowerDirection;

/* Behavior used after a previously acquired line disappears. */
typedef struct {
    float angular_velocity_rad_s;
    float timeout_s;
} TapeFollowerSearchConfig;

/* Combines estimator geometry, feedback gains, and lost-tape behavior.
 *
 * The front and back estimators should use weights whose signs follow the
 * drivetrain body convention: negative is right and positive is left.
 */
typedef struct {
    const TapeLineEstimatorConfig *estimators[TAPE_FOLLOWER_SENSOR_COUNT];
    /* Source-agnostic lateral error controller. Tape line error is only one
     * possible error source; the controller itself is shared with motion. */
    OffTapeMotionConfig lateral_motion;
    TapeFollowingKinematicsConfig heading;
    TapeFollowerSearchConfig search;

    /* Upper bound used for PID integration and differentiation after loop stalls. */

    /* Rate-limits how fast the commanded lateral velocity itself may change,
     * mirroring heading.max_acceleration_rad_s2's existing slew limit on
     * omega. line_error is a coarse, quantized signal (each tape module is 4
     * boolean channels, not a continuous sensor -- see
     * tape_line_estimator.c), so it steps rather than varies smoothly; a
     * derivative term on it would react to those steps as sharp spikes
     * instead of damping real oscillation. This limits the *output's* rate
     * of change directly instead, which damps the same jumpiness without
     * differentiating the noisy input. */
    float max_lateral_accel_mps2;
} TapeFollowerConfig;

/* Supplies all guidance modules and an explicit px/mx/py travel direction.
 * travel_velocity_mps is a positive speed magnitude. Sensor sampling remains
 * outside this module so it can be tested without GPIO. */
typedef struct {
    const TapeSensor *sensors[TAPE_FOLLOWER_SENSOR_COUNT];
    TapeFollowerDirection direction;
    float travel_velocity_mps;
    /* Net odometry delta projected onto the selected tape axis since the
     * previous update, in m. Positive means progress in the selected
     * direction. The caller may use encoders, optical flow, or another source. */
    float along_tape_delta_m;
} TapeFollowerInput;

/* Returns a command compatible with drivetrain_set_body_velocity plus useful
 * diagnostic values. Apply requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    TapeFollowerStatus status;
    float line_error;
    /* Net progress along the tape during this follower session. It is signed
     * in the requested travel direction and therefore does not accumulate
     * lateral/steering oscillations as distance. */
    float along_tape_distance_m;
    bool motion_valid;
} TapeFollowerOutput;

/* Retains estimator, controller, and tape-loss history for one behavior instance. */
typedef struct {
    const TapeFollowerConfig *config;
    TapeLineEstimatorState estimator_states[TAPE_FOLLOWER_SENSOR_COUNT];
    OffTapeMotion lateral_motion;
    float along_tape_distance_m;
    float lost_elapsed_s;
    float requested_omega_rad_s;
    /* Last slew-limited lateral velocity commanded -- see
     * TapeFollowerConfig.max_lateral_accel_mps2. */
    float requested_lateral_velocity_mps;
    TapeFollowerDirection active_direction;
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
