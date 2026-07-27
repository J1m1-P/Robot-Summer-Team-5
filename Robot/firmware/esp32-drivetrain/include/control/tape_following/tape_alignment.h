/* Declares bounded, non-oscillating tape alignment primitives. */
#pragma once

#include <stdbool.h>

#include "control/drivetrain/x_drive_kinematics.h"
#include "control/tape_following/tape_follower.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TAPE_ALIGNMENT_I_ALIGN_LONGITUDINAL = 0,
    TAPE_ALIGNMENT_L_ALIGN_PY_MX,
    TAPE_ALIGNMENT_MODE_COUNT,
} TapeAlignmentMode;

typedef enum {
    TAPE_ALIGNMENT_IDLE = 0,
    TAPE_ALIGNMENT_RUNNING,
    TAPE_ALIGNMENT_COMPLETE,
    TAPE_ALIGNMENT_TIMEOUT,
    TAPE_ALIGNMENT_FAULT,
} TapeAlignmentStatus;

typedef struct {
    TapeAlignmentMode mode;
    const TapeLineEstimatorConfig *estimators[TAPE_FOLLOWER_SENSOR_COUNT];
    float correction_speed_mps;
    float error_tolerance;
    float timeout_s;
    unsigned settle_samples;
} TapeAlignmentConfig;

typedef struct {
    const TapeSensor *sensors[TAPE_FOLLOWER_SENSOR_COUNT];
    float dt_s;
} TapeAlignmentInput;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    TapeAlignmentStatus status;
    float front_error;
    float back_error;
    float side_error;
    bool motion_valid;
    bool aligned;
} TapeAlignmentOutput;

typedef struct {
    const TapeAlignmentConfig *config;
    TapeLineEstimatorState estimator_states[TAPE_FOLLOWER_SENSOR_COUNT];
    TapeAlignmentStatus status;
    float elapsed_s;
    unsigned settle_count;
    bool started;
} TapeAlignment;

bool tape_alignment_config_is_valid(const TapeAlignmentConfig *config);

esp_err_t tape_alignment_init(TapeAlignment *alignment,
                              const TapeAlignmentConfig *config);
esp_err_t tape_alignment_start(TapeAlignment *alignment);
esp_err_t tape_alignment_reset(TapeAlignment *alignment);
esp_err_t tape_alignment_update(TapeAlignment *alignment,
                                const TapeAlignmentInput *input,
                                TapeAlignmentOutput *output);

#ifdef __cplusplus
}
#endif
