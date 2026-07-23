/* Declares MoveL: a closed-loop straight-line movement primitive. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/off_tape_motion.h"
#include "control/drivetrain/speed_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MoveL follows a straight path using the shared off-tape PID for lateral
 * correction.  It deliberately receives no encoder, PMW3610, or pose type:
 * the caller's active estimator/error module supplies normalized path
 * feedback through MoveLInput, making the primitive sensor-source agnostic. */
typedef struct {
    OffTapeMotionConfig off_tape_motion;
    SpeedProfileConfig speed_profile;
    float distance_tolerance_m;
    float max_accel_mps2;
} MoveLConfig;

bool move_l_config_is_valid(const MoveLConfig *config);

typedef enum {
    MOVE_L_IDLE = 0,
    MOVE_L_RUNNING,
    MOVE_L_COMPLETE,
    MOVE_L_FAULT,
} MoveLStatus;

/* Both values are measured in the commanded path frame, not a sensor's raw
 * coordinate frame.  The error provider may derive them from encoder
 * odometry today or PMW3610-based estimation later. */
typedef struct {
    float along_track_progress_m;
    float cross_track_error_m;
    bool valid;
} MoveLInput;

typedef struct {
    const MoveLConfig *config;
    OffTapeMotion off_tape_motion;
    SpeedProfile profile;
    float distance_m;
    float body_direction_x;
    float body_direction_y;
    float max_speed_mps;
    bool braking;
    MoveLStatus status;
} MoveL;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    MoveLStatus status;
    float remaining_distance_m;
    bool motion_valid;
} MoveLOutput;

/* `heading_rad` is a body-frame travel direction: zero is body +x/front and
 * positive rotates the path toward body +y/left. The caller converts this
 * body-frame direction into world coordinates using the current estimate. */
esp_err_t move_l_start(
    MoveL *move,
    const MoveLConfig *config,
    float distance_m,
    float heading_rad,
    float max_speed_mps);

/* Advances one closed-loop cycle from source-agnostic path feedback.  The
 * caller applies requested_velocity through the drivetrain facade. */
esp_err_t move_l_update(
    MoveL *move,
    const MoveLInput *input,
    float dt_s,
    MoveLOutput *output);

#ifdef __cplusplus
}
#endif
