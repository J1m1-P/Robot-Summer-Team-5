#include "control/drivetrain_odometry.h"

#include <cmath>

esp_err_t drivetrain_odometry_update(DrivetrainOdometry &odo, const DrivetrainOdometryDelta &delta, bool valid) {
    if (!std::isfinite(delta.forward_mm) || !std::isfinite(delta.lateral_mm) || !std::isfinite(delta.heading_delta_rad)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid) {
        // Placeholder policy: hold pose steady, record the fault, don't
        // latch. This is the "decide the real fault policy" TODO from the
        // header -- revisit before relying on this for anything beyond
        // getting the math working.
        odo.last_fault = DrivetrainOdometryFault::InvalidCycle;
        return ESP_OK;
    }

    // Rotate (forward_mm, lateral_mm) from robot frame into world frame
    // using R(theta) at the pre-update heading and accumulate into
    // odo.pose. (Using the midpoint heading theta + heading_delta_rad/2
    // instead would be more accurate over the cycle -- not done here.)
    const float theta = odo.pose.heading_rad;
    const float c = std::cos(theta);
    const float s = std::sin(theta);

    odo.pose.x_mm += c * delta.forward_mm + s * delta.lateral_mm;
    odo.pose.y_mm += -s * delta.forward_mm + c * delta.lateral_mm;
    odo.pose.heading_rad += delta.heading_delta_rad;

    return ESP_OK;
}

void drivetrain_odometry_reset(DrivetrainOdometry &odo) {
    odo = DrivetrainOdometry{};
}
