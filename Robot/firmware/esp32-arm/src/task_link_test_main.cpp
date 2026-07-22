/* Interactive USB companion for exercising the arm side of the real UART task link. */
#include <Arduino.h>
#include "esp_random.h"
#include <robot_common/packet_router.h>
#include <robot_common/uart_link.h>
#include "communication/task_server.h"
#include "config/task_link_config.h"
#include "config/uart_link_config.h"
#include "task/arm_manager.h"

namespace {
UartLink drive_uart = {};
PacketRouter router = {};
ArmManager manager = {};
ArmTaskServer server = {};
String input_line;
bool ready = false;
bool stream_enabled = true;
uint32_t last_stream_ms = 0;
uint32_t auto_complete_ms = 0;
uint32_t action_started_ms = 0;
TaskStepStatus previous_status = TASK_STEP_NOT_STARTED;

const char *step_name(TaskStepStatus v) {
    static const char *names[] = {"not_started", "running", "succeeded", "cancelled", "failed"};
    return v <= TASK_STEP_FAILED ? names[v] : "unknown";
}
const char *action_name(TaskAction v) {
    static const char *names[] = {"follow_tape", "align_pieces", "pick_up_block", "align_tape", "align_base", "build_tower"};
    return v < TASK_ACTION_COUNT ? names[v] : "unknown";
}
void print_state(const char *event) {
    Serial.printf(
        "{\"source\":\"arm\",\"event\":\"%s\",\"ms\":%lu,\"ready\":%s,"
        "\"armSession\":%lu,\"coordinatorSession\":%lu,\"hasCommand\":%s,"
        "\"execution\":%lu,\"command\":%lu,\"step\":%u,\"action\":\"%s\","
        "\"stepStatus\":\"%s\",\"autoCompleteMs\":%lu,\"tx\":%lu,\"rx\":%lu,"
        "\"checksumErrors\":%lu,\"parseErrors\":%lu,\"dropped\":%lu,"
        "\"duplicates\":%lu,\"staleCommands\":%lu,\"protocolErrors\":%lu}\n",
        event, (unsigned long)millis(), ready ? "true" : "false",
        (unsigned long)server.arm_session_id, (unsigned long)server.coordinator_session_id,
        server.has_command ? "true" : "false", (unsigned long)server.command.step.execution_id,
        (unsigned long)server.command.command_id, server.command.step.step,
        action_name(server.command.step.action), step_name(manager.status),
        (unsigned long)auto_complete_ms, (unsigned long)drive_uart.packets_sent,
        (unsigned long)drive_uart.packets_received, (unsigned long)drive_uart.checksum_errors,
        (unsigned long)drive_uart.parse_errors, (unsigned long)drive_uart.packets_dropped,
        (unsigned long)server.diagnostics.duplicate_command_count,
        (unsigned long)server.diagnostics.stale_command_count,
        (unsigned long)server.diagnostics.protocol_error_count);
}
void reply(const char *level, const char *message) {
    Serial.printf("{\"source\":\"arm\",\"event\":\"message\",\"level\":\"%s\",\"message\":\"%s\"}\n", level, message);
}
void handle_command(String line) {
    line.trim();
    if (line == "status") print_state("state");
    else if (line == "arm succeed") reply(arm_manager_report_succeeded(&manager) ? "ok" : "error", "arm step success requested");
    else if (line == "arm fail") reply(arm_manager_report_failed(&manager, TASK_FAILURE_STEP_FAILED) ? "ok" : "error", "arm step failure requested");
    else if (line.startsWith("auto ")) {
        auto_complete_ms = (uint32_t)line.substring(5).toInt();
        reply("ok", auto_complete_ms ? "auto-complete enabled" : "auto-complete disabled");
    } else if (line == "stream on" || line == "stream off") {
        stream_enabled = line.endsWith("on"); reply("ok", stream_enabled ? "telemetry enabled" : "telemetry disabled");
    } else if (line == "help") reply("ok", "arm succeed, arm fail, auto <milliseconds|0>, status, stream on|off");
    else reply("error", "unknown command; send help");
}
void read_usb() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n') { handle_command(input_line); input_line = ""; }
        else if (c != '\r' && input_line.length() < 100) input_line += c;
    }
}
}

void setup() {
    Serial.begin(115200); delay(300);
    const uint32_t session = esp_random() ?: 1U;
    arm_manager_init(&manager);
    if (uart_link_init(&drive_uart, &DRIVETRAIN_UART_LINK_CONFIG) != ESP_OK) return;
    if (!arm_task_server_init(&server, &drive_uart, &manager, session, &ARM_TASK_SERVER_CONFIG)) return;
    if (!packet_router_init(&router, &drive_uart) ||
        !packet_router_set_handler(&router, PACKET_TYPE_TASK_COMMAND, arm_task_server_process_packet, &server) ||
        !packet_router_set_handler(&router, PACKET_TYPE_HEARTBEAT, arm_task_server_process_packet, &server)) return;
    ready = true; reply("ok", "arm UART harness ready"); print_state("boot");
}

void loop() {
    read_usb();
    if (!ready) { delay(20); return; }
    const uint32_t now = millis();
    if (packet_router_update(&router, now) != ESP_OK) arm_task_server_handle_link_error(&server);
    arm_task_server_update(&server, now);
    if (manager.status == TASK_STEP_RUNNING && previous_status != TASK_STEP_RUNNING) action_started_ms = now;
    previous_status = manager.status;
    if (auto_complete_ms && manager.status == TASK_STEP_RUNNING &&
        (uint32_t)(now - action_started_ms) >= auto_complete_ms) {
        (void)arm_manager_report_succeeded(&manager); print_state("auto_completed");
    }
    if (stream_enabled && (uint32_t)(now - last_stream_ms) >= 250U) {
        last_stream_ms = now; print_state("state");
    }
    delay(1);
}
