/*
 * Implements small mathematical helpers shared by the robot firmware.
 */
#include <robot_common/math_utils.h>

// Constrains a floating-point value to the inclusive minimum and maximum.
float clamp(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
