#include <robot_common/app_log.h>

static const char *LOG_TAGS[LOG_TAG_MAX] = {
    [LOG_TAG_MAIN] = "main",
    [LOG_TAG_UART] = "uart",
    [LOG_TAG_I2C] = "i2c",
    [LOG_TAG_MOTOR] = "motor",
    [LOG_TAG_ENCODER] = "encoder",
    [LOG_TAG_DRIVETRAIN] = "drivetrain",
    [LOG_TAG_PMW3610] = "pmw3610",
    [LOG_TAG_FUSION] = "fusion",
};

const char *app_log_tag(LogTag tag) {
    if (tag < 0 || tag >= LOG_TAG_MAX) {
        return "unknown";
    }

    return LOG_TAGS[tag];
}

void app_log_init(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_log_level_set(app_log_tag(LOG_TAG_MAIN), ESP_LOG_INFO);
    esp_log_level_set(app_log_tag(LOG_TAG_UART), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_I2C), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_MOTOR), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_ENCODER), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_DRIVETRAIN), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_PMW3610), ESP_LOG_DEBUG);
    esp_log_level_set(app_log_tag(LOG_TAG_FUSION), ESP_LOG_DEBUG);
}
