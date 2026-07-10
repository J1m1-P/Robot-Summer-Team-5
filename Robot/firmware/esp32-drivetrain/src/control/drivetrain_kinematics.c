#include "control/drivetrain_kinematics.h"

#include <math.h>

static void normalize_wheel_duty(DrivetrainWheelDuty *wheels, float max_duty) {

    float max_mag = fabsf(wheels->fl);
    if (fabsf(wheels->fr) > max_mag) max_mag = fabsf(wheels->fr);
    if (fabsf(wheels->bl) > max_mag) max_mag = fabsf(wheels->bl);
    if (fabsf(wheels->br) > max_mag) max_mag = fabsf(wheels->br);

    if (max_mag > max_duty) {
        float scale = max_duty / max_mag;

        wheels->fl *= scale;
        wheels->fr *= scale;
        wheels->bl *= scale;
        wheels->br *= scale;
    }
}

esp_err_t drivetrain_kinematics_body_to_wheels(const DrivetrainKinematicsConfig *config, const DrivetrainBodyDuty *body, DrivetrainWheelDuty *wheels) {
    if (config == NULL || body == NULL || wheels == NULL) return ESP_ERR_INVALID_ARG;
    if (!isfinite(body->x) || !isfinite(body->y) || !isfinite(body->turn)) return ESP_ERR_INVALID_ARG;
    if (!isfinite(config->wheel_angle_rad) || !isfinite(config->max_duty) || config->max_duty <= 0.0f) return ESP_ERR_INVALID_ARG;

    float angle = config->wheel_angle_rad;
    float sin_a = sinf(angle);
    float cos_a = cosf(angle);

    if (fabsf(sin_a) < 1e-6f || fabsf(cos_a) < 1e-6f) return ESP_ERR_INVALID_ARG;

    /*
        Coordinate convention:

        +x_duty     = strafe right
        +y_duty     = drive forward
        +turn_duty  = turn counterclockwise/left

        Need to flip signs after real testing depending on:
        - motor wiring
        - wheel mounting direction
        - motor direction_inverted config
    */

    float x = body->x / sin_a;
    float y = body->y / cos_a;
    float r = body->turn;

    wheels->fl = y + x - r;
    wheels->fr = y - x + r;
    wheels->bl = y - x - r;
    wheels->br = y + x + r;

    normalize_wheel_duty(wheels, config->max_duty);

    return ESP_OK;
}