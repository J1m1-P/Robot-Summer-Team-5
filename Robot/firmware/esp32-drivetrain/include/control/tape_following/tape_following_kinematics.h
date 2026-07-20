/* Declares the pure tape-following velocity-to-turn-rate mapping. */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gain_s_inv;
    float max_omega_rad_s;
    float max_acceleration_rad_s2;
} TapeFollowingKinematicsConfig;

/* Turns the leading edge toward the requested translation direction while
 * limiting angular velocity and its change from the previous request. */
esp_err_t tape_following_kinematics_velocity_to_angular_velocity(
    const TapeFollowingKinematicsConfig *config,
    float travel_velocity_mps,
    float lateral_velocity_mps,
    float previous_omega_rad_s,
    float dt_s,
    float *omega_out);

#ifdef __cplusplus
}
#endif
