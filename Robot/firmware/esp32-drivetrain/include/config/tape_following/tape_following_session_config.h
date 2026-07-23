/* Exposes the tape-following session's production tuning defaults. */
#pragma once

#include "control/tape_following/tape_following_session.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compile-time defaults for everything the session needs beyond one task's
 * direction/speed/distance (set per-invocation by tape_session_action.c):
 * controlled-stop/end-correction tuning and locating-detector geometry.
 * Values carried over from src/harnesses/drivetrain_test_main.cpp's
 * start_tape_session(), the only proven-working reference in the repo.
 * `stop_at_locating_event` and `home_before_following` are left false --
 * those capabilities are separate future task actions, not this one. */
extern const TapeFollowingSessionConfig TAPE_FOLLOWING_SESSION_CONFIG;

#ifdef __cplusplus
}
#endif
