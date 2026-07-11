#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "sensing/pmw3610_fusion.h"

// Pose Manager: owns the canonical cumulative pose for this board's
// bench-testing output only. The main-control board (esp32-drivetrain)
// owns real fault policy from the per-cycle `valid` flag on the
// DELTA,... UART line; a fault here just resets pose to the origin (see
// pmw3610_pose_zero()) and resumes integrating on the next valid cycle --
// no latch.

#ifdef __cplusplus
extern "C" {
#endif

// Cumulative pose, as returned by pmw3610_pose_get().
typedef struct {
    float x_mm;
    float y_mm;
    float heading_deg;
} Pose;

// Coarse fault classification for pmw3610_pose_get_status() -- which
// side(s) reported a hardware-invalid cycle (see pmw3610_status_valid()
// for what "invalid" means: OVF, low SQUAL, or laser fault).
typedef enum {
    FAULT_NONE = 0,
    FAULT_LEFT_SENSOR_INVALID,
    FAULT_RIGHT_SENSOR_INVALID,
    FAULT_BOTH_SENSORS_INVALID,
} FaultReason;

typedef struct {
    bool output_killed;
    FaultReason last_fault;
} PoseStatus;

typedef struct {
    float theta_rad;  // kept in radians internally; see pmw3610_pose_get()
    float x_mm;
    float y_mm;

    bool output_killed;
    FaultReason last_fault;
} Pmw3610PoseManager;

void pmw3610_pose_init(Pmw3610PoseManager *pm);

// Feed one cycle's fused delta plus that cycle's raw hardware validity
// (from dual_pmw3610_poll()). If either sensor was invalid this cycle,
// records which side(s) failed and resets pose to the origin; the next
// valid cycle resumes integrating normally from there.
void pmw3610_pose_update(Pmw3610PoseManager *pm, const DeltaPose *delta, bool l_valid, bool r_valid);

Pose pmw3610_pose_get(const Pmw3610PoseManager *pm);
PoseStatus pmw3610_pose_get_status(const Pmw3610PoseManager *pm);

// Resets cumulative pose to the origin. Called automatically by
// pmw3610_pose_update() on a faulted cycle; also callable manually.
void pmw3610_pose_zero(Pmw3610PoseManager *pm);

// Clears the displayed fault status (output_killed/last_fault) without
// waiting for the next update() cycle. Not required for pose to resume --
// update() already auto-recovers on the next valid cycle -- this just
// lets a bench operator clear the STATUS line's display early.
void pmw3610_pose_clear_fault(Pmw3610PoseManager *pm);

// Compact text form for sending over any transport (UART/etc.) --
// "x_mm,y_mm,heading_deg". Returns the number of bytes written (excluding
// the null terminator), or -1 if buf_len was too small.
int pmw3610_pose_to_csv(const Pmw3610PoseManager *pm, char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
