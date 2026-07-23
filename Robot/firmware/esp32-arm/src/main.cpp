/* Production composition root for top sensing and remote task execution. */
#include <Arduino.h>

#include "esp_random.h"

#include <robot_common/app_log.h>
#include <robot_common/packet_router.h>
#include <robot_common/task/task_link_client.h>
#include <robot_common/task/task_link_server.h>
#include <robot_common/uart_link.h>

#include "config/task_link_config.h"
#include "config/tof_config.h"
#include "config/uart_link_config.h"
#include "control/time_of_flight/tof_manager.h"
#include "task/arm_manager.h"
#include "task/top_action_dispatcher.h"

static TofManager tof_manager = {};
static ArmManager arm_manager = {};
static UartLink drivetrain_uart = {};
static PacketRouter drivetrain_router = {};
static UartLink pi_uart = {};
static PacketRouter pi_router = {};
static TaskLinkClient pi_scan_client = {};
static TopActionDispatcher top_dispatcher = {};
static TaskLinkServer drivetrain_task_server = {};
static bool application_ready = false;
static bool tof_ready = false;

static uint32_t new_session_id() {
    const uint32_t value = esp_random();
    return value == 0U ? 1U : value;
}

void setup() {
    app_log_init();

    esp_err_t tof_error = tof_manager_init(&tof_manager, &ARM_TOF_CONFIG);
    if (tof_error == ESP_OK) tof_error = tof_manager_start(&tof_manager);
    if (tof_error != ESP_OK) {
        APP_LOGE(LOG_TAG_MAIN, "Arm ToF init failed, continuing without it: %s",
                 esp_err_to_name(tof_error));
    } else {
        tof_ready = true;
    }

    ESP_ERROR_CHECK(uart_link_init(&drivetrain_uart,
                                   &DRIVETRAIN_UART_LINK_CONFIG));
    ESP_ERROR_CHECK(uart_link_init(&pi_uart, &PI_UART_LINK_CONFIG));

    const uint32_t top_session_id = new_session_id();
    arm_manager_init(&arm_manager);
    if (!task_link_client_init(&pi_scan_client, &pi_uart, top_session_id,
                               &PI_SCAN_CLIENT_CONFIG)) {
        APP_LOGE(LOG_TAG_UART, "Pi scan client initialization failed");
        return;
    }
    const TaskActionExecutor pi_scan_executor =
        task_link_client_executor(&pi_scan_client);
    if (!top_action_dispatcher_init(&top_dispatcher, &arm_manager,
                                    &pi_scan_executor)) {
        APP_LOGE(LOG_TAG_MAIN, "Top action dispatcher initialization failed");
        return;
    }
    const TaskActionExecutor top_executor =
        top_action_dispatcher_executor(&top_dispatcher);
    if (!task_link_server_init(&drivetrain_task_server, &drivetrain_uart,
                               &top_executor, top_session_id,
                               &TOP_TASK_SERVER_CONFIG)) {
        APP_LOGE(LOG_TAG_UART, "Drivetrain task server initialization failed");
        return;
    }

    if (!packet_router_init(&drivetrain_router, &drivetrain_uart) ||
        !packet_router_set_handler(&drivetrain_router, PACKET_TYPE_TASK_COMMAND,
                                   task_link_server_process_packet,
                                   &drivetrain_task_server) ||
        !packet_router_set_handler(&drivetrain_router, PACKET_TYPE_HEARTBEAT,
                                   task_link_server_process_packet,
                                   &drivetrain_task_server) ||
        !packet_router_init(&pi_router, &pi_uart) ||
        !packet_router_set_handler(&pi_router, PACKET_TYPE_TASK_STATUS,
                                   task_link_client_process_packet,
                                   &pi_scan_client) ||
        !packet_router_set_handler(&pi_router, PACKET_TYPE_HEARTBEAT,
                                   task_link_client_process_packet,
                                   &pi_scan_client)) {
        APP_LOGE(LOG_TAG_UART, "Top UART packet routing initialization failed");
        return;
    }
    application_ready = true;
}

void loop() {
    if (tof_ready) {
        const esp_err_t sensor_error = tof_manager_poll(&tof_manager);
        if (sensor_error != ESP_OK && sensor_error != ESP_ERR_NOT_FINISHED) {
            APP_LOGE(LOG_TAG_MAIN,
                     "Arm ToF polling failed; sensor disabled until reboot: %s",
                     esp_err_to_name(sensor_error));
            (void)tof_manager_stop(&tof_manager);
            tof_ready = false;
        }
    }
    if (application_ready) {
        const uint32_t now_ms = millis();
        if (packet_router_update(&drivetrain_router, now_ms) != ESP_OK) {
            task_link_server_handle_link_error(&drivetrain_task_server,
                                               now_ms);
        }
        if (packet_router_update(&pi_router, now_ms) != ESP_OK) {
            task_link_client_handle_link_error(&pi_scan_client);
        }
        task_link_client_update(&pi_scan_client, now_ms);
        task_link_server_update(&drivetrain_task_server, now_ms);
    }
    delay(1);
}
