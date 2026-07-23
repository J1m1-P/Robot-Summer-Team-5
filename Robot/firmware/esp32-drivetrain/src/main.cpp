/* Production composition root for drivetrain task coordination. */
#include <Arduino.h>

#include "esp_random.h"
#include "esp_timer.h"

#include <robot_common/app_log.h>
#include <robot_common/packet_router.h>
#include <robot_common/task/task_link_client.h>
#include <robot_common/uart_link.h>

#include "config/communication/task_link_config.h"
#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "config/tape_following/tape_following_config.h"
#include "config/task/production_task_config.h"
#include "control/drivetrain/drivetrain.h"
#include "task/drivetrain_manager.h"
#include "task/task_coordinator.h"

static Drivetrain drivetrain = {};
static DrivetrainManager drivetrain_manager = {};
static UartLink arm_uart = {};
static PacketRouter arm_packet_router = {};
static TaskLinkClient top_client = {};
static TaskCoordinator task_coordinator = {};
static bool application_ready = false;
static bool production_task_started = false;
static uint32_t ready_since_ms = 0;

static uint32_t new_session_id() {
    uint32_t value = esp_random();
    return value == 0U ? 1U : value;
}

static bool enter_safe_drivetrain_state(void *context) {
    Drivetrain *safe_drivetrain = static_cast<Drivetrain *>(context);
    if (drivetrain_brake(safe_drivetrain) != ESP_OK) return false;
    return !safe_drivetrain->status.enabled &&
           safe_drivetrain->status.brake_engaged;
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

    // Tape-following hardware failure is logged internally and leaves
    // follow-tape actions disabled rather than blocking the whole robot.
    (void)drivetrain_manager_init(&drivetrain_manager, &drivetrain,
                                  &TAPE_SENSOR_MUX_CONFIG,
                                  &FRONT_TAPE_SENSOR_CONFIG,
                                  &BACK_TAPE_SENSOR_CONFIG,
                                  &LEFT_TAPE_SENSOR_CONFIG,
                                  &TAPE_FOLLOWER_CONFIG);
    if (!task_link_client_init(&top_client, &arm_uart, new_session_id(),
                               &TOP_TASK_CLIENT_CONFIG)) {
        APP_LOGE(LOG_TAG_UART, "Top task client initialization failed");
        return;
    }
    if (!packet_router_init(&arm_packet_router, &arm_uart) ||
        !packet_router_set_handler(&arm_packet_router, PACKET_TYPE_TASK_STATUS,
                                   task_link_client_process_packet, &top_client) ||
        !packet_router_set_handler(&arm_packet_router, PACKET_TYPE_HEARTBEAT,
                                   task_link_client_process_packet, &top_client)) {
        APP_LOGE(LOG_TAG_UART, "Arm UART packet routing initialization failed");
        return;
    }

    const TaskActionExecutor drivetrain_executor =
        drivetrain_manager_executor(&drivetrain_manager);
    const TaskActionExecutor top_executor =
        task_link_client_executor(&top_client);
    const TaskSafeStateHandler safe_state = {
        .context = &drivetrain,
        .enter = enter_safe_drivetrain_state,
    };
    if (!task_coordinator_init(&task_coordinator, &TASK_COORDINATOR_CONFIG,
                               &drivetrain_executor, &top_executor,
                               &safe_state)) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Task coordinator initialization failed");
        return;
    }

    application_ready = true;
    ready_since_ms = millis();
    APP_LOGI(LOG_TAG_DRIVETRAIN,
             "Task coordinator ready; drivetrain remains safely braked");
}

void loop() {
    if (!application_ready) {
        delay(100);
        return;
    }

    const uint32_t now_ms = millis();

    if (!production_task_started &&
        now_ms - ready_since_ms >= PRODUCTION_TASK_START_DELAY_MS) {
        production_task_started = true;
        if (!task_coordinator_start(&task_coordinator, &PRODUCTION_TASK_REQUEST,
                                    new_session_id())) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to start production task");
        }
    }

    if (packet_router_update(&arm_packet_router, now_ms) != ESP_OK) {
        task_link_client_handle_link_error(&top_client);
    }
    task_link_client_update(&top_client, now_ms);

    TaskFailure peer_failure = TASK_FAILURE_NONE;
    if (task_link_client_take_peer_failure(&top_client, &peer_failure)) {
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
