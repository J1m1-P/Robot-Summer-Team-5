#include "control/motion/translator.hpp"

#include <cmath>
#include <algorithm>

#include <Arduino.h>
#include "esp_timer.h"
#include "control/pid/bounded_pid.h"
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/math_utils.h>

namespace {

constexpr float kMinDisp = 1.0e-4f;       // m: ignore tiny translations
constexpr int64_t kCtrlPeriodUs = 5000;   // 200 Hz
constexpr float kPosTol = 0.01f;          // m: final position tolerance
constexpr float kHeadTol = 0.035f;        // rad: final heading tolerance (~2°)
constexpr float kVelTol = 0.05f;           // m/s: practical stopped threshold
constexpr float kStartHeadTol = 0.05f;    // rad: begin translation threshold
constexpr float kApproach = 0.03f;         // m: start slowing near target
constexpr float kMaxVy = 0.25f;            // m/s: correction limit
constexpr float kMaxOmega = 1.0f;          // rad/s: correction limit
constexpr float kMaxDt = 0.02f;            // s: reject delayed cycles

float wrap(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

BoundedPidConfig pid(float p, float i, float d, float limit) {
    return {p, i, d, limit, -limit, limit};
}

}  // namespace

esp_err_t precision_move(
    const PrecisionMoveContext *context,
    const PrecisionMoveTarget *target,
    float timeout_s) {
    if (context == nullptr || context->drivetrain == nullptr ||
        context->pose_service == nullptr || context->pose_service->pose_tracker == nullptr ||
        target == nullptr || !std::isfinite(target->dx_body_m) ||
        !std::isfinite(target->dy_body_m) || !std::isfinite(target->delta_heading_rad) ||
        !std::isfinite(target->body_velocity.vx) ||
        !std::isfinite(target->body_velocity.vy) ||
        !std::isfinite(target->body_velocity.omega) ||
        !std::isfinite(timeout_s) || timeout_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const Pose start = pose_tracker_get_pose(context->pose_service->pose_tracker);
    if (!std::isfinite(start.x_m) || !std::isfinite(start.y_m) ||
        !std::isfinite(start.heading_rad)) {
        return ESP_ERR_INVALID_STATE;
    }

    const float c0 = std::cos(start.heading_rad), s0 = std::sin(start.heading_rad);
    const float dx = c0 * target->dx_body_m - s0 * target->dy_body_m;
    const float dy = s0 * target->dx_body_m + c0 * target->dy_body_m;
    const Pose goal = {start.x_m + dx, start.y_m + dy,
                         wrap(start.heading_rad + target->delta_heading_rad)};

    const float disp = std::hypot(dx, dy);       // world-frame distance
    const float path_head = disp > kMinDisp
        ? std::atan2(dy, dx)
        : goal.heading_rad;

    const BoundedPidConfig x_pid = pid(1.0f, 0.1f, 0.3f, kMaxVy);
    const BoundedPidConfig y_pid = pid(1.0f, 0.1f, 0.3f, kMaxVy);
    const BoundedPidConfig h_pid = pid(1.0f, 0.1f, 0.3f, kMaxOmega);
    const BoundedPidConfig r_pid = pid(2.0f, 0.2f, 0.6f, kMaxOmega);
    BoundedPidState x_st = {}, y_st = {}, h_st = {}, r_st = {};  // PID history

    enum class State { RotateStart, TranslateAndBlend, FinalSettle };
    State state = disp > kMinDisp
        ? State::RotateStart : State::FinalSettle;
    const int64_t t0 = esp_timer_get_time();
    FixedRateGate gate = {kCtrlPeriodUs, t0};
    Pose prev = start;                           // previous estimate
    float vx = 0.0f, vy = 0.0f;                  // world-frame velocity

    auto stop = [&]() {
        drivetrain_stop(context->drivetrain);
    };

    while (true) {
        const int64_t now = esp_timer_get_time();
        if (static_cast<float>(now - t0) / 1.0e6f >= timeout_s) {
            stop();
            return ESP_ERR_TIMEOUT;
        }

        int64_t dt_us = 0;
        if (!gate.Ready(now, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt = static_cast<float>(dt_us) / 1.0e6f;
        if (!std::isfinite(dt) || dt <= 0.0f || dt > kMaxDt) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }
        const esp_err_t pose_err = pose_service_update(
            context->pose_service, static_cast<uint32_t>(now / 1000));
        if (pose_err != ESP_OK) {
            stop();
            return pose_err;
        }

        const Pose cur = pose_tracker_get_pose(context->pose_service->pose_tracker);
        if (!std::isfinite(cur.x_m) || !std::isfinite(cur.y_m) ||
            !std::isfinite(cur.heading_rad)) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }
        vx = (cur.x_m - prev.x_m) / dt;
        vy = (cur.y_m - prev.y_m) / dt;
        prev = cur;

    const float exw = goal.x_m - cur.x_m, eyw = goal.y_m - cur.y_m;
        const float dist = std::hypot(exw, eyw);  // remaining world distance
        const float c = std::cos(cur.heading_rad), s = std::sin(cur.heading_rad);
        const float ex = c * exw + s * eyw, ey = -s * exw + c * eyw;  // body error
        const float speed = std::hypot(c * vx + s * vy, -s * vx + c * vy);
        DrivetrainBodyVelocity cmd = {};

        switch (state) {
        case State::RotateStart: {
            const float eh = wrap(path_head - cur.heading_rad);
            if (std::fabs(eh) <= kStartHeadTol) {
                bounded_pid_reset(&r_st);
                state = State::TranslateAndBlend;
            } else {
                cmd.omega = bounded_pid_update(&r_st, &r_pid, eh, dt);
            }
            break;
        }
        case State::TranslateAndBlend: {
            const float turn = std::fabs(wrap(goal.heading_rad - path_head));
            const float blend = std::max(0.05f, turn * 0.05f);
            const float ratio = clamp(1.0f - dist / blend, 0.0f, 1.0f);
            const float head = wrap(path_head + ratio * wrap(goal.heading_rad - path_head));
            const float approach = clamp(dist / kApproach, 0.0f, 1.0f);
            cmd.vx = target->body_velocity.vx * approach +
                bounded_pid_update(&x_st, &x_pid, ex, dt);
            cmd.vy = target->body_velocity.vy * approach +
                bounded_pid_update(&y_st, &y_pid, ey, dt);
            cmd.omega = target->body_velocity.omega * approach +
                bounded_pid_update(&h_st, &h_pid, wrap(head - cur.heading_rad), dt);
            if (dist <= kPosTol) {
                bounded_pid_reset(&h_st);
                state = State::FinalSettle;
            }
            break;
        }
        case State::FinalSettle: {
            const float eh = wrap(goal.heading_rad - cur.heading_rad);
            if (dist > kPosTol) {
                state = State::TranslateAndBlend;
                break;
            }
            if (std::fabs(eh) <= kHeadTol && speed <= kVelTol) {
                stop();
                return ESP_OK;
            }
            cmd.omega = bounded_pid_update(&r_st, &r_pid, eh, dt);
            break;
        }
        }

        const esp_err_t command_err = drivetrain_set_advanced_body_velocity(
            context->drivetrain, cmd.vx, cmd.vy, cmd.omega);
        const esp_err_t update_err = command_err == ESP_OK
            ? drivetrain_update(context->drivetrain, now) : command_err;
        if (update_err != ESP_OK) {
            stop();
            return update_err;
        }
    }
}
