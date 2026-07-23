/* Declares the PMW3610 cumulative-pose-packet to body-frame delta bridge. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/odometry_packet.h>

#include "control/drivetrain/odometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Tracks the last packet consumed, so each call can de-integrate the
 * cumulative sample into a body-frame delta relative to the arm's own
 * previously-reported heading. */
typedef struct {
    DrivetrainPose previous_pose;
    bool has_previous;
    uint32_t last_sequence;
    bool has_last_sequence;
} Pmw3610OdometrySource;

// Clears the tracked previous-sample baseline.
void pmw3610_odometry_source_reset(Pmw3610OdometrySource *source);

/* Converts the latest received OdometryPacket into a body-frame delta since
 * the previous call. `packet` must be the caller's latest decoded sample --
 * pass NULL if none has ever been received. Returns false (and leaves
 * *delta_out untouched) when: packet is NULL, its sequence number has not
 * advanced since the last call (nothing new to de-integrate), there is no
 * prior sample yet to diff against (this call only captures a baseline), or
 * packet->valid is false -- in every such case the caller should fall back
 * to another odometry source. */
bool pmw3610_odometry_source_update(Pmw3610OdometrySource *source,
                                    const OdometryPacket *packet,
                                    DrivetrainOdometryDelta *delta_out);

#ifdef __cplusplus
}
#endif
