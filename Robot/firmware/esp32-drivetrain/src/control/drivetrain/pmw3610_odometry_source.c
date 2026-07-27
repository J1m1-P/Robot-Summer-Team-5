/* Implements the PMW3610 cumulative-pose-packet to body-frame delta bridge. */
#include "control/drivetrain/pmw3610_odometry_source.h"

#include <stddef.h>
#include <string.h>

void pmw3610_odometry_source_reset(Pmw3610OdometrySource *source) {
    if (source != NULL) memset(source, 0, sizeof(*source));
}

bool pmw3610_odometry_source_update(Pmw3610OdometrySource *source,
                                    const OdometryPacket *packet,
                                    DrivetrainOdometryDelta *delta_out) {
    if (source == NULL || packet == NULL || delta_out == NULL ||
        !packet->valid) {
        return false;
    }

    if (source->has_last_sequence && packet->sequence == source->last_sequence) {
        return false;
    }
    source->last_sequence = packet->sequence;
    source->has_last_sequence = true;

    const DrivetrainPose current_pose = {
        .x_mm = packet->x_mm,
        .y_mm = packet->y_mm,
        .heading_rad = packet->theta_rad,
    };

    if (!source->has_previous) {
        source->previous_pose = current_pose;
        source->has_previous = true;
        return false;
    }

    /* De-integrate the cumulative world-frame sample back into a body-frame
     * delta, rotating by the arm's own previously-reported heading -- the
     * same frame its own integration used, independent of the drivetrain's
     * world frame. */
    *delta_out = drivetrain_odometry_delta_between(&source->previous_pose,
                                                   &current_pose);
    source->previous_pose = current_pose;
    return true;
}
