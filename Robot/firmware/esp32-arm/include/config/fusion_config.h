#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Maps [forward_mm, lateral_mm] -> [dx_counts, dy_counts] for one sensor:
//     [dx]   [m00 m01] [forward_mm]
//     [dy] = [m10 m11] [lateral_mm]
// Maps [forward_mm, lateral_mm] to raw sensor counts. The static JSON stores
// only a unit rotation; static_calibration_load() applies the fixed count/mm
// scale before configuring fusion.
typedef struct {
    float m00, m01, m10, m11;
} SensorCalibration;

typedef struct {
    SensorCalibration cal_left;
    SensorCalibration cal_right;
    float baseline_mm;  // physical sensor separation, direct caliper measurement
} FusionConfig;

#ifdef __cplusplus
}
#endif
