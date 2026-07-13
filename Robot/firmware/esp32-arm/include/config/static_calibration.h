#pragma once

#include <stdbool.h>

#include "config/fusion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Loads the unit rotation matrices from /calibration.json in LittleFS and
// applies the fixed CPI-derived count/mm scale required by fusion. Returns
// false when the filesystem, JSON document, or matrix values are invalid.
bool static_calibration_load(FusionConfig *config);

#ifdef __cplusplus
}
#endif
