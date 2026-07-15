#include "sensing/pmw3610_pose.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void pmw3610_pose_init(Pmw3610PoseManager *pm) {
    memset(pm, 0, sizeof(*pm));
}

void pmw3610_pose_update(Pmw3610PoseManager *pm, const DeltaPose *delta, bool l_valid, bool r_valid) {
    if (!l_valid || !r_valid) {
        pm->output_killed = true;
        pm->last_fault = (!l_valid && !r_valid)   ? FAULT_BOTH_SENSORS_INVALID
                          : !l_valid              ? FAULT_LEFT_SENSOR_INVALID
                                                   : FAULT_RIGHT_SENSOR_INVALID;
        return;
    }
    pm->output_killed = false;

    // The fused delta is measured over the same cycle as dtheta. Rotate it
    // through the cycle's midpoint heading rather than the final heading,
    // avoiding a systematic lateral error while the rig is turning.
    float midpoint_heading = pm->theta_rad + delta->dtheta_rad * 0.5f;
    pm->x_mm += delta->dx_mm * cosf(midpoint_heading) - delta->dy_mm * sinf(midpoint_heading);
    pm->y_mm += delta->dx_mm * sinf(midpoint_heading) + delta->dy_mm * cosf(midpoint_heading);
    pm->theta_rad += delta->dtheta_rad;
}

Pose pmw3610_pose_get(const Pmw3610PoseManager *pm) {
    Pose p;
    p.x_mm = pm->x_mm;
    p.y_mm = pm->y_mm;
    p.heading_deg = pm->theta_rad * 180.0f / (float)M_PI;
    return p;
}

PoseStatus pmw3610_pose_get_status(const Pmw3610PoseManager *pm) {
    PoseStatus s;
    s.output_killed = pm->output_killed;
    s.last_fault = pm->last_fault;
    return s;
}

void pmw3610_pose_zero(Pmw3610PoseManager *pm) {
    pm->theta_rad = 0.0f;
    pm->x_mm = 0.0f;
    pm->y_mm = 0.0f;
}

void pmw3610_pose_clear_fault(Pmw3610PoseManager *pm) {
    pm->output_killed = false;
    pm->last_fault = FAULT_NONE;
}

int pmw3610_pose_to_csv(const Pmw3610PoseManager *pm, char *buf, size_t buf_len) {
    Pose p = pmw3610_pose_get(pm);
    int written = snprintf(buf, buf_len, "%.2f,%.2f,%.2f", p.x_mm, p.y_mm, p.heading_deg);
    if (written < 0 || (size_t)written >= buf_len) return -1;
    return written;
}
