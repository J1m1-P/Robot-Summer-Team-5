/* Interactive USB companion for exercising the arm side of the real UART task link. */
#include <Arduino.h>
#include "esp_random.h"
#include <robot_common/packet_router.h>
#include <robot_common/task/task_link_server.h>
#include <robot_common/uart_link.h>
#include "config/task_link_config.h"
#include "config/uart_link_config.h"
#include "task/arm_manager.h"

#ifdef SYSTEM_TEST_BUILD
#include "harnesses/system_test_mode.h"
#define setup top_task_link_test_setup
#define loop top_task_link_test_loop
#endif

namespace {
UartLink drive_uart = {};
PacketRouter router = {};
TaskLinkServer server = {};
ArmManager arm_manager = {};
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
    switch (v) {
        case TASK_ACTION_FOLLOW_TAPE: return "follow_tape";
        case TASK_ACTION_ALIGN_TO_PIECES: return "align_to_pieces";
        case TASK_ACTION_PICK_UP_BLOCK: return "pick_up_block";
        case TASK_ACTION_ALIGN_TO_TAPE: return "align_to_tape";
        case TASK_ACTION_BUILD_TOWER: return "build_tower";
        case TASK_ACTION_SCAN_TELETUBBIES: return "scan_teletubbies";
        case TASK_ACTION_FOLLOW_PIECES_TAPE: return "follow_pieces_tape";
        case TASK_ACTION_FOLLOW_TASK_TAPE: return "follow_task_tape";
        case TASK_ACTION_POSITION_TOWER_X: return "position_tower_x";
        case TASK_ACTION_OPEN_TOWER_CLAWS: return "open_tower_claws";
        case TASK_ACTION_TOWER_FACE_DOWN: return "tower_face_down";
        case TASK_ACTION_LOWER_TOWER: return "lower_tower";
        case TASK_ACTION_CLOSE_TOWER_CLAWS: return "close_tower_claws";
        case TASK_ACTION_RAISE_TOWER: return "raise_tower";
        case TASK_ACTION_TOWER_FACE_FRONT: return "tower_face_front";
        case TASK_ACTION_BACK_OFF_PIECES: return "back_off_pieces";
        default: return "unknown";
    }
}
void print_state(const char *event) {
    Serial.printf(
        "{\"source\":\"arm\",\"event\":\"%s\",\"ms\":%lu,\"ready\":%s,"
        "\"armSession\":%lu,\"coordinatorSession\":%lu,\"hasCommand\":%s,"
        "\"execution\":%lu,\"command\":%lu,\"step\":%u,\"action\":\"%s\","
        "\"amount\":%.5f,\"speed\":%.5f,\"settleMs\":%lu,"
        "\"stepStatus\":\"%s\",\"autoCompleteMs\":%lu,\"tx\":%lu,\"rx\":%lu,"
        "\"checksumErrors\":%lu,\"parseErrors\":%lu,\"dropped\":%lu,"
        "\"duplicates\":%lu,\"staleCommands\":%lu,\"protocolErrors\":%lu}\n",
        event, (unsigned long)millis(), ready ? "true" : "false",
        (unsigned long)server.executor_session_id,
        (unsigned long)server.requester_session_id,
        server.has_command ? "true" : "false", (unsigned long)server.command.step.execution_id,
        (unsigned long)server.command.command_id, server.command.step.step,
        action_name(server.command.step.action),
        server.command.step.parameters.amount,
        server.command.step.parameters.speed,
        (unsigned long)server.command.step.parameters.settle_ms,
        step_name(server.result.status),
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

void clear_link_runtime() {
    if (drive_uart.config != nullptr) {
        (void)uart_flush_input(drive_uart.config->uart_num);
    }
    drive_uart.parser = {};
    drive_uart.packet_queue_head = 0U;
    drive_uart.packet_queue_tail = 0U;
    drive_uart.packet_queue_count = 0U;
    drive_uart.packets_sent = 0U;
    drive_uart.packets_received = 0U;
    drive_uart.packets_dropped = 0U;
    drive_uart.checksum_errors = 0U;
    drive_uart.parse_errors = 0U;
}

bool initialize_task_runtime() {
    clear_link_runtime();
    previous_status = TASK_STEP_NOT_STARTED;
    const TaskActionExecutor executor = arm_manager_executor(&arm_manager);
    const uint32_t session = esp_random() ?: 1U;
    if (!task_link_server_init(&server, &drive_uart, &executor, session,
                               &TOP_TASK_SERVER_CONFIG)) return false;
    return packet_router_init(&router, &drive_uart) &&
           packet_router_set_handler(&router, PACKET_TYPE_TASK_COMMAND,
                                     task_link_server_process_packet, &server) &&
           packet_router_set_handler(&router, PACKET_TYPE_HEARTBEAT,
                                     task_link_server_process_packet, &server);
}

void reset_harness() {
    if (server.has_command && server.result.status == TASK_STEP_RUNNING) {
        server.executor.cancel(server.executor.context, millis());
    }
    ready = false;
    ready = initialize_task_runtime();
    reply(ready ? "ok" : "error",
          ready ? "top executor and link state reset"
                : "top executor reset failed");
    print_state("reset");
}

void handle_command(String line) {
    line.trim();
#ifdef SYSTEM_TEST_BUILD
    if (system_test_handle_mode_command(line)) return;
#endif
    if (line == "status") print_state("state");
    else if (line == "stop") {
        const bool active =
            server.has_command && server.result.status == TASK_STEP_RUNNING;
        if (active) server.executor.cancel(server.executor.context, millis());
        reply("ok", "LIVE TEST arm stop requested");
    }
    else if (line == "arm succeed") {
        const bool accepted = arm_manager_report_succeeded(&arm_manager);
        reply(accepted ? "ok" : "error", "top step success requested");
    } else if (line == "arm fail") {
        const bool accepted = arm_manager_report_failed(
            &arm_manager, TASK_FAILURE_STEP_FAILED);
        reply(accepted ? "ok" : "error", "top step failure requested");
    }
    else if (line.startsWith("auto ")) {
        auto_complete_ms = (uint32_t)line.substring(5).toInt();
        reply("ok", auto_complete_ms ? "auto-complete enabled" : "auto-complete disabled");
    } else if (line == "reset" || line == "restart") {
        reset_harness();
    } else if (line == "stream on" || line == "stream off") {
        stream_enabled = line.endsWith("on"); reply("ok", stream_enabled ? "telemetry enabled" : "telemetry disabled");
    } else if (line == "help") reply("ok", "arm succeed, arm fail, auto <milliseconds|0>, reset, status, stream on|off");
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
    if (uart_link_init(&drive_uart, &DRIVETRAIN_UART_LINK_CONFIG) != ESP_OK) return;
    arm_manager_init(&arm_manager);
    if (!arm_manager.pick_up_block.hardware_ready) {
        reply("error", "live arm hardware initialization failed");
        return;
    }
    ready = initialize_task_runtime();
    if (!ready) return;
    reply("ok", "LIVE TEST/TUNING arm executor ready; commands move hardware");
    print_state("boot");
}

void loop() {
    read_usb();
    if (!ready) { delay(20); return; }
    const uint32_t now = millis();
    if (packet_router_update(&router, now) != ESP_OK) {
        task_link_server_handle_link_error(&server, now);
    }
    task_link_server_update(&server, now);
    if (server.result.status == TASK_STEP_RUNNING &&
        previous_status != TASK_STEP_RUNNING) action_started_ms = now;
    previous_status = server.result.status;
    if (auto_complete_ms && server.result.status == TASK_STEP_RUNNING &&
        (uint32_t)(now - action_started_ms) >= auto_complete_ms) {
        (void)arm_manager_report_succeeded(&arm_manager);
        print_state("auto_completed");
    }
    if (stream_enabled && (uint32_t)(now - last_stream_ms) >= 250U) {
        last_stream_ms = now; print_state("state");
    }
    delay(1);
}
