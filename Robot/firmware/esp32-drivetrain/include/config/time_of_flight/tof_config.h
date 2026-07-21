/* Exposes the drivetrain time-of-flight topology and sensor indices. */
#pragma once

#include "control/time_of_flight/tof_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRIVETRAIN_TOF_LEFT = 0,
    DRIVETRAIN_TOF_RIGHT,
    DRIVETRAIN_VL53L0X_COUNT
} DrivetrainVL53L0XId;

typedef enum {
    DRIVETRAIN_TOF_CENTER = 0,
    DRIVETRAIN_VL53L5CX_COUNT
} DrivetrainVL53L5CXId;

extern const TofManagerConfig DRIVETRAIN_TOF_CONFIG;

#ifdef __cplusplus
}
#endif
