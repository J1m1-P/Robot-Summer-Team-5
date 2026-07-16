/*
 * Public logging interface shared by both firmware targets.
 * It provides consistent tags and convenience macros around ESP-IDF logging.
 */
#pragma once

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Identifies the firmware subsystem that produced a log message.
typedef enum {
    LOG_TAG_MAIN = 0,
    LOG_TAG_UART,
    LOG_TAG_I2C,
    LOG_TAG_MOTOR,
    LOG_TAG_ENCODER,
    LOG_TAG_DRIVETRAIN,
    LOG_TAG_PMW3610,
    LOG_TAG_FUSION,
    LOG_TAG_MAX
} LogTag;

// Applies the default global and subsystem-specific log levels.
void app_log_init(void);

// Returns the text label for a log tag, or "unknown" for an invalid tag.
const char *app_log_tag(LogTag tag);

#ifdef __cplusplus
}
#endif

// Writes an error-level message using a shared LogTag.
#define APP_LOGE(tag, fmt, ...) ESP_LOGE(app_log_tag(tag), fmt, ##__VA_ARGS__)

// Writes a warning-level message using a shared LogTag.
#define APP_LOGW(tag, fmt, ...) ESP_LOGW(app_log_tag(tag), fmt, ##__VA_ARGS__)

// Writes an info-level message using a shared LogTag.
#define APP_LOGI(tag, fmt, ...) ESP_LOGI(app_log_tag(tag), fmt, ##__VA_ARGS__)

// Writes a debug-level message using a shared LogTag.
#define APP_LOGD(tag, fmt, ...) ESP_LOGD(app_log_tag(tag), fmt, ##__VA_ARGS__)

// Writes a verbose-level message using a shared LogTag.
#define APP_LOGV(tag, fmt, ...) ESP_LOGV(app_log_tag(tag), fmt, ##__VA_ARGS__)
