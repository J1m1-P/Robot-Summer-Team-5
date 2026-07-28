// Line following: own weighted-channel line error, EMA filter, PD steering.
#include "control/line_following/line_follower.hpp"

#include <Arduino.h>
#include <cmath>

#include "esp_timer.h"
#include "control/pid/bounded_pid.h"
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/math_utils.h>

namespace {

constexpr int64_t kControlPeriodUs = 5000;  // 200 Hz
constexpr int kSideSensorIndex = 2;

constexpr float kFrontWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
constexpr float kBackWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};  // back is mounted mirrored

constexpr float kP = 0.5f;
constexpr float kD = 0.05f;
constexpr float kEmaAlpha = 0.4f;
constexpr float kMaxOmegaRadS = 1.4;

// Deadband due to tape width and sensor pitch
constexpr float kErrorDeadband = 1.5f;

// EMA for omega adjustment
constexpr float kOmegaEmaAlpha = 0.2f;

// No integral term
constexpr BoundedPidConfig kSteeringPidConfig = {
    .proportional_gain = kP,
    .integral_gain = 0.0f,
    .derivative_gain = kD,
    .integral_limit = 0.0f,
    .correction_min = -kMaxOmegaRadS,
    .correction_max = kMaxOmegaRadS,
};

constexpr float kSearchOmegaRadS = 0.4f;  // spin rate while hunting for lost tape
constexpr float kSearchTimeoutS = 2.0f;

// Telemetry rate (throttled)
constexpr int64_t kTelemetryPeriodUs = 200000;  // 5 Hz

// Ramps vx down over the last stretch before a known DISTANCE/LATERAL stop
// point instead of driving at full speed right up to the abrupt stop.
constexpr float kApproachRampDistanceM = 0.03f;

constexpr float kMinRampSpeedMps = 0.1f;

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
    BoundedPidState steering_pid_state = {};
    float last_error_sign = 1.0f;
    float lost_elapsed_s = 0.0f;
    float smoothed_omega = 0.0f;
    int64_t next_telemetry_us = start_us;
    Pose previous_pose = pose_tracker_get_pose(ctx->pose_service->pose_tracker);

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
        if (pose_service_update(ctx->pose_service, static_cast<uint32_t>(now_us / 1000)) != ESP_OK) {
            return Abort(false);
        }
        const Pose current_pose = pose_tracker_get_pose(ctx->pose_service->pose_tracker);
        cumulative_distance_m += std::hypot(current_pose.x_m - previous_pose.x_m,
                                            current_pose.y_m - previous_pose.y_m);
        previous_pose = current_pose;

        const bool leading_active = ctx->sensors[kSideSensorIndex]->channel_0;
        const bool lateral_done = lateral_aligner.Update(
            leading_active, cumulative_distance_m,
            stop_type == StopCondition::LATERAL_TWO ? kAlignGapCenterM : kAlignTapeCenterM);

        // Ramp speed down over the last kApproachRampDistanceM before stop point
        float ramp_target_speed_mps = speed_mps;
        const float ramp_floor_mps = fminf(kMinRampSpeedMps, speed_mps);
        const bool has_distance_target = stop_type == StopCondition::DISTANCE ||
            ((stop_type == StopCondition::LATERAL_ONE || stop_type == StopCondition::LATERAL_TWO) &&
             lateral_aligner.triggered);
        if (has_distance_target) {
            const float target_m =
                stop_type == StopCondition::DISTANCE ? stop_value : lateral_aligner.target_m;
            const float remaining_m = target_m - cumulative_distance_m;
            if (remaining_m < kApproachRampDistanceM) {
                const float t = clamp(remaining_m / kApproachRampDistanceM, 0.0f, 1.0f);
                ramp_target_speed_mps = ramp_floor_mps + t * (speed_mps - ramp_floor_mps);
            }
        }

        float raw_error = 0.0f;
        const bool on_tape = ComputeLineError(ctx->sensors[steer.sensor_index], steer.weights, &raw_error);

        float omega;
        float vx = 0.0f, vy = 0.0f;
        if (on_tape) {
            last_error_sign = raw_error >= 0.0f ? 1.0f : -1.0f;
            lost_elapsed_s = 0.0f;
            filtered_error = kEmaAlpha * raw_error + (1.0f - kEmaAlpha) * filtered_error;
            // Soft threshold with deadband
            const float error_magnitude = std::fabs(filtered_error);
            const float deadbanded_error = error_magnitude < kErrorDeadband
                ? 0.0f
                : std::copysign(error_magnitude - kErrorDeadband, filtered_error);
            omega = bounded_pid_update(&steering_pid_state, &kSteeringPidConfig, deadbanded_error, dt_s);
            switch (dir) {
                case Direction::PX: vx = ramp_target_speed_mps; break;
                case Direction::MX: vx = -ramp_target_speed_mps; break;
                case Direction::PY: vy = ramp_target_speed_mps; break;
            }
        } else {
            lost_elapsed_s += dt_s;
            if (lost_elapsed_s >= kSearchTimeoutS) return Abort(false);
            omega = last_error_sign * kSearchOmegaRadS;  // spin toward last known side
        }
        smoothed_omega = kOmegaEmaAlpha * omega + (1.0f - kOmegaEmaAlpha) * smoothed_omega;
        omega = smoothed_omega;

        if (now_us >= next_telemetry_us) {
            next_telemetry_us = now_us + kTelemetryPeriodUs;
            Serial.printf(
                "# tape t=%.2fs dist=%.3fm raw=%.2f filt=%.2f on_tape=%d omega=%.2f vx=%.2f\n",
                elapsed_s, cumulative_distance_m, raw_error, filtered_error,
                on_tape ? 1 : 0, omega, vx);
            const PoseTracker *pose_tracker = ctx->pose_service->pose_tracker;
            Serial.printf(
                "# pose x=%.3fm y=%.3fm heading=%.2frad optical_updates=%u encoder_updates=%u\n",
                current_pose.x_m, current_pose.y_m, current_pose.heading_rad,
                static_cast<unsigned>(pose_tracker->optical_update_count),
                static_cast<unsigned>(pose_tracker->encoder_update_count));
            const Pmw3610OdometryLink *odometry_link = ctx->pose_service->odometry_link;
            if (odometry_link != nullptr && odometry_link->has_packet) {
                const OdometryPacket &optical = odometry_link->latest;
                Serial.printf(
                    "# optical seq=%u valid=%d x=%.1fmm y=%.1fmm theta=%.2frad\n",
                    static_cast<unsigned>(optical.sequence), optical.valid ? 1 : 0,
                    optical.x_mm, optical.y_mm, optical.theta_rad);
            }
        }

        // use corrected velocity with calibration
        drivetrain_set_advanced_body_velocity(ctx->drivetrain, vx, vy, omega);
        // Fresh timestamp
        drivetrain_update(ctx->drivetrain, esp_timer_get_time());

        bool stop_reached = false;
        switch (stop_type) {
            case StopCondition::TIME_ONLY: stop_reached = elapsed_s >= stop_value; break;
            case StopCondition::DISTANCE: stop_reached = cumulative_distance_m >= stop_value; break;
            case StopCondition::LATERAL_ONE:
            case StopCondition::LATERAL_TWO: stop_reached = lateral_done; break;
        }
        if (!stop_reached) continue;

        const bool result = Abort(true);
        float overshoot_m = 0.0f;
        switch (stop_type) {
            case StopCondition::DISTANCE: overshoot_m = cumulative_distance_m - stop_value; break;
            case StopCondition::LATERAL_ONE:
            case StopCondition::LATERAL_TWO: overshoot_m = cumulative_distance_m - lateral_aligner.target_m; break;
            case StopCondition::TIME_ONLY: break;
        }
        Serial.printf(
            "# tape stop dir=%d dist=%.3fm target=%.3fm overshoot=%.3fm\n",
            static_cast<int>(dir), cumulative_distance_m, stop_value, overshoot_m);
        return result;
    }
}
