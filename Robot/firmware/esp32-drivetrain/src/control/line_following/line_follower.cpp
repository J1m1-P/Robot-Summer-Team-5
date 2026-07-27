// Line following: own weighted-channel line error, EMA filter, PD steering.
#include "control/line_following/line_follower.hpp"

#include <algorithm>
#include <cmath>

#include "esp_timer.h"

#include <robot_common/fixed_rate_gate.h>

namespace {

constexpr int64_t kControlPeriodUs = 5000;  // 200 Hz
constexpr int kSideSensorIndex = 2;

constexpr float kFrontWeights[4] = {3.0f, 1.0f, -1.0f, -3.0f};
constexpr float kBackWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};  // back is mounted mirrored

constexpr float kP = 20.0f;
constexpr float kD = 12.0f;
constexpr float kEmaAlpha = 0.4f;
constexpr float kMaxOmegaRadS = 1.2f;

constexpr float kSearchOmegaRadS = 0.8f;  // spin rate while hunting for lost tape
constexpr float kSearchTimeoutS = 2.0f;

constexpr float kTapeWidthM = 0.019f;
constexpr float kSensorPitchM = 0.019f;
constexpr float kStripSpacingM = 0.045f;
// extra distance past the leading channel's first detection to land the
// array's center on the tape (ONE) or the gap between two strips (TWO)
constexpr float kAlignTapeCenterM = 1.5f * kSensorPitchM + kTapeWidthM / 2.0f;
constexpr float kAlignGapCenterM = kAlignTapeCenterM + kStripSpacingM / 2.0f;

// Arms a distance target on first detection; reports done once reached.
struct LateralAligner {
    bool prev_active = false;
    bool triggered = false;
    float target_m = 0.0f;

    bool Update(bool active, float distance_m, float offset_m) {
        if (!triggered) {
            if (active && !prev_active) {
                triggered = true;
                target_m = distance_m + offset_m;
            }
            prev_active = active;
            return false;
        }
        return distance_m >= target_m;
    }
};

struct DirectionInfo {
    int sensor_index;
    const float *weights;
};

DirectionInfo GetDirectionInfo(Direction dir) {
    switch (dir) {
        case Direction::PX: return {0, kFrontWeights};
        case Direction::MX: return {1, kBackWeights};
        case Direction::PY: return {kSideSensorIndex, kFrontWeights};
    }
    return {0, kFrontWeights};
}

// weighted centroid of active channels, in [-3, 3]; false if line is lost
bool ComputeLineError(const TapeSensor *s, const float w[4], float *error_out) {
    float sum = 0.0f;
    int active = 0;
    if (s->channel_0) { sum += w[0]; active++; }
    if (s->channel_1) { sum += w[1]; active++; }
    if (s->channel_2) { sum += w[2]; active++; }
    if (s->channel_3) { sum += w[3]; active++; }
    if (active == 0) return false;
    *error_out = sum / static_cast<float>(active);
    return true;
}

}  // namespace

bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                  StopCondition stop_type, float stop_value, float timeout_s) {
    if (ctx == nullptr || ctx->drivetrain == nullptr) return false;

    auto Abort = [ctx](bool result) {
        drivetrain_stop(ctx->drivetrain);
        return result;
    };

    const DirectionInfo steer = GetDirectionInfo(dir);
    const int64_t start_us = esp_timer_get_time();
    FixedRateGate gate = {kControlPeriodUs, start_us};
    LateralAligner lateral_aligner;
    float cumulative_distance_m = 0.0f;
    float filtered_error = 0.0f;
    float prev_error = 0.0f;
    float last_error_sign = 1.0f;
    float lost_elapsed_s = 0.0f;
    Pose previous_pose = pose_tracker_get_pose(ctx->pose_tracker);

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const float elapsed_s = static_cast<float>(now_us - start_us) / 1e6f;
        if (elapsed_s >= timeout_s) return Abort(false);

        int64_t dt_us = 0;
        if (!gate.Ready(now_us, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt_s = static_cast<float>(dt_us) / 1e6f;

        if (tape_sensor_driver_read_all(ctx->sensors) != ESP_OK) return Abort(false);

        const DrivetrainWheelCounts wheel_counts = drivetrain_get_wheel_counts(ctx->drivetrain);
        if (pose_tracker_update(ctx->pose_tracker, &wheel_counts, nullptr) != ESP_OK) {
            return Abort(false);
        }
        const Pose current_pose = pose_tracker_get_pose(ctx->pose_tracker);
        cumulative_distance_m += std::hypot(current_pose.x_m - previous_pose.x_m,
                                            current_pose.y_m - previous_pose.y_m);
        previous_pose = current_pose;

        const bool leading_active = ctx->sensors[kSideSensorIndex]->channel_0;
        const bool lateral_done = lateral_aligner.Update(
            leading_active, cumulative_distance_m,
            stop_type == StopCondition::LATERAL_TWO ? kAlignGapCenterM : kAlignTapeCenterM);

        float raw_error = 0.0f;
        const bool on_tape =
            ComputeLineError(ctx->sensors[steer.sensor_index], steer.weights, &raw_error);

        float omega;
        if (on_tape) {
            last_error_sign = raw_error >= 0.0f ? 1.0f : -1.0f;
            lost_elapsed_s = 0.0f;
            filtered_error = kEmaAlpha * raw_error + (1.0f - kEmaAlpha) * filtered_error;
            const float d_term = (filtered_error - prev_error) / dt_s;
            prev_error = filtered_error;
            omega = kP * filtered_error + kD * d_term;
        } else {
            lost_elapsed_s += dt_s;
            if (lost_elapsed_s >= kSearchTimeoutS) return Abort(false);
            omega = last_error_sign * kSearchOmegaRadS;  // spin toward last known side
        }
        omega = std::clamp(omega, -kMaxOmegaRadS, kMaxOmegaRadS);

        float vx = 0.0f, vy = 0.0f;
        switch (dir) {
            case Direction::PX: vx = speed_mps; break;
            case Direction::MX: vx = -speed_mps; break;
            case Direction::PY: vy = speed_mps; break;
        }
        drivetrain_set_body_velocity(ctx->drivetrain, vx, vy, omega);
        drivetrain_update(ctx->drivetrain, now_us);

        bool stop_reached = false;
        switch (stop_type) {
            case StopCondition::TIME_ONLY: stop_reached = elapsed_s >= stop_value; break;
            case StopCondition::DISTANCE: stop_reached = cumulative_distance_m >= stop_value; break;
            case StopCondition::LATERAL_ONE:
            case StopCondition::LATERAL_TWO: stop_reached = lateral_done; break;
        }
        if (stop_reached) return Abort(true);
    }
}
