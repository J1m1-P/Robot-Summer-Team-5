#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Constrains value to [min_value, max_value]. Requires min_value <= max_value.
float clamp(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif
