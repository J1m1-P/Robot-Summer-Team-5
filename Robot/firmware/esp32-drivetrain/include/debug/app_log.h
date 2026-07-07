#pragma once 

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_TAG_MAIN = 0,
    LOG_TAG_MOTOR,
    LOG_TAG_ENCODER,
    LOG_TAG_DRIVETRAIN,
    LOG_TAG_MAX
} LogTag;

void app_log_init(void);

const char *app_log_tag(LogTag tag);

#ifdef __cplusplus
}
#endif

#define APP_LOGE(tag, fmt, ...) ESP_LOGE(app_log_tag(tag), fmt, ##__VA_ARGS__)
#define APP_LOGW(tag, fmt, ...) ESP_LOGW(app_log_tag(tag), fmt, ##__VA_ARGS__)
#define APP_LOGI(tag, fmt, ...) ESP_LOGI(app_log_tag(tag), fmt, ##__VA_ARGS__)
#define APP_LOGD(tag, fmt, ...) ESP_LOGD(app_log_tag(tag), fmt, ##__VA_ARGS__)
#define APP_LOGV(tag, fmt, ...) ESP_LOGV(app_log_tag(tag), fmt, ##__VA_ARGS__)