/* Production composition root for drivetrain task coordination. */
#include <Arduino.h>

#include "esp_random.h"
#include "esp_timer.h"

#include <robot_common/app_log.h>
#include <robot_common/uart_link.h>

#include "communication/arm_task_client.h"
#include "config/communication/task_link_config.h"
#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "task/drivetrain_manager.h"
#include "task/task_coordinator.h"

static Drivetrain drivetrain = {};
static DrivetrainManager drivetrain_manager = {};
static UartLink arm_uart = {};
static ArmTaskClient arm_client = {};
static TaskCoordinator task_coordinator = {};
static bool application_ready = false;

static uint32_t new_session_id() {
    uint32_t value = esp_random();
    return value == 0U ? 1U : value;
}

void setup() {
    Serial.begin(115200);
    app_log_init();

    esp_err_t error = drivetrain_init(&drivetrain, &DRIVETRAIN_CONFIG);
    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Drivetrain initialization failed: %s",
                 esp_err_to_name(error));
        return;
    }

    error = uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_UART, "Arm UART initialization failed: %s",
                 esp_err_to_name(error));
        return;
    }

    drivetrain_manager_init(&drivetrain_manager, &drivetrain);
    if (!arm_task_client_init(&arm_client, &arm_uart, new_session_id(),
                              &ARM_TASK_CLIENT_CONFIG)) {
        APP_LOGE(LOG_TAG_UART, "Arm task client initialization failed");
        return;
    }

    const TaskActionExecutor drivetrain_executor =
        drivetrain_manager_executor(&drivetrain_manager);
    const TaskActionExecutor arm_executor =
        arm_task_client_executor(&arm_client);
    if (!task_coordinator_init(&task_coordinator, &TASK_COORDINATOR_CONFIG,
                               &drivetrain_executor, &arm_executor)) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Task coordinator initialization failed");
        return;
    }

    application_ready = true;
    APP_LOGI(LOG_TAG_DRIVETRAIN,
             "Task coordinator ready; drivetrain remains safely braked");
}

void loop() {
    if (!application_ready) {
        delay(100);
        return;
    }

    const uint32_t now_ms = millis();
    arm_task_client_update_link(&arm_client, now_ms);

    TaskFailure peer_failure = TASK_FAILURE_NONE;
    if (arm_task_client_take_peer_failure(&arm_client, &peer_failure)) {
        (void)task_coordinator_fail(&task_coordinator, peer_failure, now_ms);
    }

    task_coordinator_update(&task_coordinator, now_ms);
    if (drivetrain.status.enabled) {
        const esp_err_t error = drivetrain_update(&drivetrain,
                                                  esp_timer_get_time());
        if (error != ESP_OK) {
            (void)drivetrain_manager_report_failed(
                &drivetrain_manager, TASK_FAILURE_STEP_FAILED);
        }
    }
    delay(1);
}
