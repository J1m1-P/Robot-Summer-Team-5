/* Exposes the claw time-of-flight topology and sensor IDs. */
#pragma once

#include "control/time_of_flight/tof_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ARM_TOF_LEFT = 0,
    ARM_TOF_MID,
    ARM_TOF_RIGHT,
    ARM_TOF_COUNT
} ArmTofId;

extern const TofManagerConfig ARM_TOF_CONFIG;

#ifdef __cplusplus
}
#endif
