#pragma once

#include <cstdint>

/* Gates work in a busy/blocking loop to a fixed period, without dictating
 * how the caller waits between checks. Any board/loop that must run a step
 * at a steady rate (tape following, an odometry link send, future movement
 * actions) can reuse this instead of re-deriving the same timestamp math. */
struct FixedRateGate {
    int64_t period_us;
    int64_t last_us;

    // Returns true (and rearms) once at least `period_us` has elapsed since
    // the last true return, writing the actual elapsed time to
    // `elapsed_us_out` so the caller's control math uses real dt, not the
    // nominal period. Returns false otherwise, leaving state untouched.
    bool Ready(int64_t now_us, int64_t *elapsed_us_out) {
        const int64_t elapsed_us = now_us - last_us;
        if (elapsed_us < period_us) {
            return false;
        }
        last_us = now_us;
        *elapsed_us_out = elapsed_us;
        return true;
    }
};
