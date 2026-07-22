/* Production composition root for arm sensing and remote task execution. */
#include <Arduino.h>

#include "esp_random.h"

#include <robot_common/app_log.h>
#include <robot_common/uart_link.h>

#include "communication/task_server.h"
#include "config/task_link_config.h"
#include "config/tof_config.h"
#include "config/uart_link_config.h"
#include "control/time_of_flight/tof_manager.h"
#include "task/arm_manager.h"

static TofManager tof_manager = {};
static ArmManager arm_manager = {};
static UartLink drivetrain_uart = {};
static ArmTaskServer task_server = {};
static bool application_ready = false;

static uint32_t new_session_id() {
    uint32_t value = esp_random();
    return value == 0U ? 1U : value;
}

void setup() {
    app_log_init();
    ESP_ERROR_CHECK(tof_manager_init(&tof_manager, &ARM_TOF_CONFIG));
    ESP_ERROR_CHECK(tof_manager_start(&tof_manager));
    ESP_ERROR_CHECK(uart_link_init(&drivetrain_uart,
                                   &DRIVETRAIN_UART_LINK_CONFIG));

    arm_manager_init(&arm_manager);
    if (!arm_task_server_init(&task_server, &drivetrain_uart, &arm_manager,
                              new_session_id(), &ARM_TASK_SERVER_CONFIG)) {
        APP_LOGE(LOG_TAG_UART, "Arm task server initialization failed");
        return;
    }
    application_ready = true;
}

void loop() {
    const esp_err_t sensor_error = tof_manager_poll(&tof_manager);
    if (sensor_error != ESP_OK && sensor_error != ESP_ERR_NOT_FINISHED) {
        APP_LOGE(LOG_TAG_MAIN, "Arm ToF polling failed: %s",
                 esp_err_to_name(sensor_error));
        (void)arm_manager_report_failed(&arm_manager,
                                        TASK_FAILURE_STEP_FAILED);
    }
    if (application_ready) {
        arm_task_server_update(&task_server, millis());
    }
    delay(1);
}
