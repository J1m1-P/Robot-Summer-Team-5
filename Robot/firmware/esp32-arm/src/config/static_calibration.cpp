/* Loads and validates fixed PMW3610 calibration matrices from LittleFS JSON. */
#include "config/static_calibration.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <math.h>

#include "config/pmw3610_config.h"

namespace {

// Reads one matrix from JSON and rejects missing, non-finite, or singular values.
bool read_matrix(JsonObjectConst json, SensorCalibration *matrix) {
    if (json.isNull()) return false;

    matrix->m00 = json["m00"] | NAN;
    matrix->m01 = json["m01"] | NAN;
    matrix->m10 = json["m10"] | NAN;
    matrix->m11 = json["m11"] | NAN;

    const float determinant = matrix->m00 * matrix->m11 - matrix->m01 * matrix->m10;
    return isfinite(matrix->m00) && isfinite(matrix->m01) && isfinite(matrix->m10) &&
           isfinite(matrix->m11) && isfinite(determinant) && fabsf(determinant) > 1e-6f;
}

// Checks that a stored matrix represents rotation without an embedded scale.
bool rotation_only(const SensorCalibration *matrix) {
    const float scale = hypotf(matrix->m00, matrix->m10);
    return fabsf(scale - 1.0f) < 0.01f && fabsf(matrix->m00 - matrix->m11) < 0.01f &&
           fabsf(matrix->m01 + matrix->m10) < 0.01f;
}

// Applies the current CPI-derived scale to a unit rotation matrix.
void apply_counts_per_mm(SensorCalibration *matrix, float counts_per_mm) {
    matrix->m00 *= counts_per_mm;
    matrix->m01 *= counts_per_mm;
    matrix->m10 *= counts_per_mm;
    matrix->m11 *= counts_per_mm;
}

}  // namespace

// Loads, validates, scales, and returns the deployed calibration document.
bool static_calibration_load(FusionConfig *config) {
    // Do not format on failure: the JSON is a required deployment artifact.
    if (config == nullptr || !LittleFS.begin(false)) return false;

    File file = LittleFS.open("/calibration.json", FILE_READ);
    if (!file) return false;

    JsonDocument document;
    const DeserializationError error = deserializeJson(document, file);
    file.close();
    if (error) return false;

    FusionConfig loaded = {};
    loaded.baseline_mm = document["baseline_mm"] | NAN;
    if (!isfinite(loaded.baseline_mm) || loaded.baseline_mm <= 0.0f ||
        !read_matrix(document["left"].as<JsonObjectConst>(), &loaded.cal_left) ||
        !read_matrix(document["right"].as<JsonObjectConst>(), &loaded.cal_right) ||
        !rotation_only(&loaded.cal_left) || !rotation_only(&loaded.cal_right)) {
        return false;
    }

    // The JSON intentionally stores direction only. The fixed physical scale
    // comes from the sensor CPI configured in pmw3610_config, so raw counts
    // are converted to millimetres before fusion/pose integration.
    const float counts_per_mm = pmw3610_get_cpi() / 25.4f;
    apply_counts_per_mm(&loaded.cal_left, counts_per_mm);
    apply_counts_per_mm(&loaded.cal_right, counts_per_mm);
    *config = loaded;
    return true;
}
