/* Interactive USB test harness for the coordinator and real ESP-to-ESP UART link. */
#include <Arduino.h>

#include "esp_random.h"

#include <robot_common/packet_router.h>
#include <robot_common/uart_link.h>

#include "communication/arm_task_client.h"
#include "config/communication/task_link_config.h"
#include "config/communication/uart_link_config.h"
#include "task/task_coordinator.h"

namespace {

UartLink arm_uart = {};
PacketRouter router = {};
ArmTaskClient arm_client = {};
TaskActionResult drive_result = {TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
TaskCoordinator coordinator = {};
String input_line;
bool ready = false;
bool stream_enabled = true;
uint32_t last_stream_ms = 0;
uint32_t next_execution_id = 1;

bool drive_start(void *, const TaskStepCommand *command, uint32_t) {
    if (command == nullptr || drive_result.status == TASK_STEP_RUNNING ||
        command->action == TASK_ACTION_PICK_UP_BLOCK ||
        command->action == TASK_ACTION_BUILD_TOWER) return false;
    drive_result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}
TaskActionResult drive_update(void *, uint32_t) { return drive_result; }
void drive_cancel(void *, uint32_t) {
    drive_result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

const char *task_name(TaskType value) {
    static const char *names[] = {"tape", "picking", "building", "routine"};
    return value < TASK_TYPE_COUNT ? names[value] : "unknown";
}

const char *task_status_name(TaskStatus value) {
    static const char *names[] = {"idle", "running", "succeeded", "cancelled", "failed"};
    return value <= TASK_STATUS_FAILED ? names[value] : "unknown";
}

const char *step_status_name(TaskStepStatus value) {
    static const char *names[] = {"not_started", "running", "succeeded", "cancelled", "failed"};
    return value <= TASK_STEP_FAILED ? names[value] : "unknown";
}

const char *failure_name(TaskFailure value) {
    static const char *names[] = {"none", "busy", "invalid_step", "step_rejected",
                                  "step_failed", "step_timeout", "link_timeout",
                                  "peer_reset", "stale_message", "protocol"};
    return value < TASK_FAILURE_COUNT ? names[value] : "unknown";
}

void print_state(const char *event) {
    const TaskRuntime &r = coordinator.runtime;
    Serial.printf(
        "{\"source\":\"drive\",\"event\":\"%s\",\"ms\":%lu,\"ready\":%s,"
        "\"task\":\"%s\",\"execution\":%lu,\"step\":%u,\"taskStatus\":\"%s\","
        "\"stepStatus\":\"%s\",\"failure\":\"%s\",\"armSession\":%lu,"
        "\"coordinatorSession\":%lu,\"commandActive\":%s,\"cancelPending\":%s,"
        "\"tx\":%lu,\"rx\":%lu,\"checksumErrors\":%lu,\"parseErrors\":%lu,"
        "\"dropped\":%lu,\"routed\":%lu,\"unhandled\":%lu,\"retries\":%lu,"
        "\"staleStatus\":%lu,\"protocolErrors\":%lu}\n",
        event, (unsigned long)millis(), ready ? "true" : "false", task_name(r.request.type),
        (unsigned long)r.execution_id, r.current_step, task_status_name(r.status),
        step_status_name(r.step_status), failure_name(r.failure),
        (unsigned long)arm_client.arm_session_id,
        (unsigned long)arm_client.coordinator_session_id,
        arm_client.command_active ? "true" : "false",
        arm_client.cancel_pending ? "true" : "false",
        (unsigned long)arm_uart.packets_sent, (unsigned long)arm_uart.packets_received,
        (unsigned long)arm_uart.checksum_errors, (unsigned long)arm_uart.parse_errors,
        (unsigned long)arm_uart.packets_dropped, (unsigned long)router.packets_routed,
        (unsigned long)router.packets_unhandled,
        (unsigned long)arm_client.diagnostics.retry_count,
        (unsigned long)arm_client.diagnostics.stale_status_count,
        (unsigned long)arm_client.diagnostics.protocol_error_count);
}

void reply(const char *level, const String &message) {
    Serial.printf("{\"source\":\"drive\",\"event\":\"message\",\"level\":\"%s\",\"message\":\"%s\"}\n",
                  level, message.c_str());
}

bool parse_task(const String &name, TaskType &type) {
    if (name == "tape") type = TASK_TYPE_TAPE_FOLLOWING;
    else if (name == "picking") type = TASK_TYPE_TOWER_PICKING;
    else if (name == "building") type = TASK_TYPE_TOWER_BUILDING;
    else if (name == "routine") type = TASK_TYPE_TOWER_ROUTINE;
    else return false;
    return true;
}

void start_task(String args) {
    args.trim();
    const int split = args.indexOf(' ');
    const String name = split < 0 ? args : args.substring(0, split);
    String values = split < 0 ? "" : args.substring(split + 1);
    TaskType type;
    if (!parse_task(name, type)) {
        reply("error", "usage: start tape|picking|building|routine [speed distance speed distance]");
        return;
    }

    TaskRequest request = {};
    request.type = type;
    request.params.tape_following = {TAPE_DIRECTION_FORWARD, 0.20f, 1.0f};
    if (type == TASK_TYPE_TOWER_ROUTINE) {
        float s1 = 0.20f, d1 = 1.0f, s2 = 0.20f, d2 = 1.0f;
        if (values.length()) sscanf(values.c_str(), "%f %f %f %f", &s1, &d1, &s2, &d2);
        request.params.tower_routine.tape_to_pieces = {TAPE_DIRECTION_FORWARD, s1, d1};
        request.params.tower_routine.tape_to_base = {TAPE_DIRECTION_FORWARD, s2, d2};
    } else if (type == TASK_TYPE_TAPE_FOLLOWING && values.length()) {
        float speed = 0.20f, distance = 1.0f;
        sscanf(values.c_str(), "%f %f", &speed, &distance);
        request.params.tape_following = {TAPE_DIRECTION_FORWARD, speed, distance};
    }
    if (!task_coordinator_start(&coordinator, &request, next_execution_id++)) {
        reply("error", "task rejected (busy or invalid parameters)");
        return;
    }
    task_coordinator_update(&coordinator, millis());
    print_state("task_started");
}

void handle_command(String line) {
    line.trim();
    if (!line.length()) return;
    if (line == "status") print_state("state");
    else if (line.startsWith("start ")) start_task(line.substring(6));
    else if (line == "drive succeed") {
        const bool accepted = drive_result.status == TASK_STEP_RUNNING;
        if (accepted) drive_result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
        reply(accepted ? "ok" : "error", "drivetrain step success requested");
    } else if (line.startsWith("drive fail")) {
        const bool accepted = drive_result.status == TASK_STEP_RUNNING;
        if (accepted) drive_result = {TASK_STEP_FAILED, TASK_FAILURE_STEP_FAILED};
        reply(accepted ? "ok" : "error", "drivetrain step failure requested");
    } else if (line == "cancel") {
        reply(task_coordinator_cancel(&coordinator, millis()) ? "ok" : "error",
              "task cancel requested");
    } else if (line == "stream on" || line == "stream off") {
        stream_enabled = line.endsWith("on");
        reply("ok", stream_enabled ? "telemetry enabled" : "telemetry disabled");
    } else if (line == "help") {
        reply("ok", "start <tape|picking|building|routine>, drive succeed, drive fail, cancel, status, stream on|off");
    } else reply("error", "unknown command; send help");
}

void read_usb() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n') { handle_command(input_line); input_line = ""; }
        else if (c != '\r' && input_line.length() < 160) input_line += c;
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    const uint32_t session = esp_random() ?: 1U;
    if (uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG) != ESP_OK) return;
    if (!arm_task_client_init(&arm_client, &arm_uart, session, &ARM_TASK_CLIENT_CONFIG)) return;
    if (!packet_router_init(&router, &arm_uart) ||
        !packet_router_set_handler(&router, PACKET_TYPE_TASK_STATUS, arm_task_client_process_packet, &arm_client) ||
        !packet_router_set_handler(&router, PACKET_TYPE_HEARTBEAT, arm_task_client_process_packet, &arm_client)) return;
    const TaskActionExecutor drive = {nullptr, drive_start, drive_update, drive_cancel};
    const TaskActionExecutor arm = arm_task_client_executor(&arm_client);
    if (!task_coordinator_init(&coordinator, &TASK_COORDINATOR_CONFIG, &drive, &arm)) return;
    ready = true;
    reply("ok", "task coordinator harness ready");
    print_state("boot");
}

void loop() {
    read_usb();
    if (!ready) { delay(20); return; }
    const uint32_t now = millis();
    if (packet_router_update(&router, now) != ESP_OK) arm_task_client_handle_link_error(&arm_client);
    arm_task_client_update(&arm_client, now);
    TaskFailure failure = TASK_FAILURE_NONE;
    if (arm_task_client_take_peer_failure(&arm_client, &failure)) {
        (void)task_coordinator_fail(&coordinator, failure, now);
        print_state("peer_failure");
    }
    task_coordinator_update(&coordinator, now);
    if (stream_enabled && (uint32_t)(now - last_stream_ms) >= 250U) {
        last_stream_ms = now;
        print_state("state");
    }
    delay(1);
}
