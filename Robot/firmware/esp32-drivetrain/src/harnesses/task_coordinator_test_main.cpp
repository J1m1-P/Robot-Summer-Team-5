/* Interactive USB test harness for the coordinator and real ESP-to-ESP UART link. */
#include <Arduino.h>

#include "esp_random.h"

#include <robot_common/packet_router.h>
#include <robot_common/task/task_link_client.h>
#include <robot_common/uart_link.h>

#include "config/communication/task_link_config.h"
#include "config/communication/uart_link_config.h"
#include "task/task_coordinator.h"

#ifdef SYSTEM_TEST_BUILD
#include "system_test_mode.h"
#define setup task_coordinator_test_setup
#define loop task_coordinator_test_loop
#endif

namespace {

UartLink arm_uart = {};
PacketRouter router = {};
TaskLinkClient top_client = {};
TaskActionResult drive_result = {TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
TaskCoordinator coordinator = {};
String input_line;
bool ready = false;
bool stream_enabled = true;
uint32_t last_stream_ms = 0;
uint32_t next_execution_id = 1;

bool drive_start(void *, const TaskStepCommand *command, uint32_t) {
    if (command == nullptr || drive_result.status == TASK_STEP_RUNNING ||
        (command->action != TASK_ACTION_FOLLOW_TAPE &&
         command->action != TASK_ACTION_ALIGN_TO_PIECES &&
         command->action != TASK_ACTION_ALIGN_TO_TAPE)) return false;
    drive_result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}
TaskActionResult drive_update(void *, uint32_t) { return drive_result; }
void drive_cancel(void *, uint32_t) {
    drive_result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

const char *task_name(TaskType value) {
    static const char *names[] = {"tape", "picking", "building", "scan"};
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
                                  "peer_reset", "stale_message", "protocol",
                                  "not_implemented", "safe_state_failed",
                                  "executor_unavailable", "target_not_found"};
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
        (unsigned long)top_client.executor_session_id,
        (unsigned long)top_client.requester_session_id,
        top_client.command_active ? "true" : "false",
        top_client.cancel_pending ? "true" : "false",
        (unsigned long)arm_uart.packets_sent, (unsigned long)arm_uart.packets_received,
        (unsigned long)arm_uart.checksum_errors, (unsigned long)arm_uart.parse_errors,
        (unsigned long)arm_uart.packets_dropped, (unsigned long)router.packets_routed,
        (unsigned long)router.packets_unhandled,
        (unsigned long)top_client.diagnostics.retry_count,
        (unsigned long)top_client.diagnostics.stale_status_count,
        (unsigned long)top_client.diagnostics.protocol_error_count);
}

void reply(const char *level, const String &message) {
    Serial.printf("{\"source\":\"drive\",\"event\":\"message\",\"level\":\"%s\",\"message\":\"%s\"}\n",
                  level, message.c_str());
}

bool parse_task(const String &name, TaskType &type) {
    if (name == "tape") type = TASK_TYPE_TAPE_FOLLOWING;
    else if (name == "picking") type = TASK_TYPE_TOWER_PICKING;
    else if (name == "building") type = TASK_TYPE_TOWER_BUILDING;
    else if (name == "scan") type = TASK_TYPE_TELETUBBY_SCAN;
    else return false;
    return true;
}

bool enter_test_safe_state(void *) { return true; }

void clear_link_runtime() {
    if (arm_uart.config != nullptr) {
        (void)uart_flush_input(arm_uart.config->uart_num);
    }
    arm_uart.parser = {};
    arm_uart.packet_queue_head = 0U;
    arm_uart.packet_queue_tail = 0U;
    arm_uart.packet_queue_count = 0U;
    arm_uart.packets_sent = 0U;
    arm_uart.packets_received = 0U;
    arm_uart.packets_dropped = 0U;
    arm_uart.checksum_errors = 0U;
    arm_uart.parse_errors = 0U;
}

bool initialize_task_runtime() {
    const uint32_t session = esp_random() ?: 1U;
    clear_link_runtime();
    if (!task_link_client_init(&top_client, &arm_uart, session,
                               &TOP_TASK_CLIENT_CONFIG)) return false;
    if (!packet_router_init(&router, &arm_uart) ||
        !packet_router_set_handler(&router, PACKET_TYPE_TASK_STATUS,
                                   task_link_client_process_packet, &top_client) ||
        !packet_router_set_handler(&router, PACKET_TYPE_HEARTBEAT,
                                   task_link_client_process_packet, &top_client)) {
        return false;
    }
    drive_result = {TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
    const TaskActionExecutor drive = {
        .context = nullptr,
        .start = drive_start,
        .update = drive_update,
        .cancel = drive_cancel,
    };
    const TaskActionExecutor top = task_link_client_executor(&top_client);
    const TaskSafeStateHandler safe_state = {
        .context = nullptr,
        .enter = enter_test_safe_state,
    };
    return task_coordinator_init(&coordinator, &TASK_COORDINATOR_CONFIG, &drive,
                                 &top, &safe_state);
}

void reset_harness() {
    if (coordinator.runtime.status == TASK_STATUS_RUNNING) {
        (void)task_coordinator_cancel(&coordinator, millis());
    }
    ready = false;
    ready = initialize_task_runtime();
    reply(ready ? "ok" : "error",
          ready ? "coordinator and link state reset"
                : "coordinator reset failed");
    print_state("reset");
}

void start_task(String args) {
    args.trim();
    const int split = args.indexOf(' ');
    const String name = split < 0 ? args : args.substring(0, split);
    String values = split < 0 ? "" : args.substring(split + 1);
    TaskType type;
    if (!parse_task(name, type)) {
        reply("error", "usage: start tape|picking|building|scan [speed distance]");
        return;
    }

    TaskRequest request = {};
    request.type = type;
    request.params.tape_following = {TAPE_DIRECTION_FORWARD, 0.20f, 1.0f};
    if (type == TASK_TYPE_TAPE_FOLLOWING && values.length()) {
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
#ifdef SYSTEM_TEST_BUILD
    if (system_test_handle_mode_command(line)) return;
#endif
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
    } else if (line == "reset" || line == "restart") {
        reset_harness();
    } else if (line == "stream on" || line == "stream off") {
        stream_enabled = line.endsWith("on");
        reply("ok", stream_enabled ? "telemetry enabled" : "telemetry disabled");
    } else if (line == "help") {
        reply("ok", "start <tape|picking|building|scan>, drive succeed, drive fail, cancel, reset, status, stream on|off");
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
    if (uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG) != ESP_OK) return;
    ready = initialize_task_runtime();
    if (!ready) return;
    reply("ok", "task coordinator harness ready");
    print_state("boot");
}

void loop() {
    read_usb();
    if (!ready) { delay(20); return; }
    const uint32_t now = millis();
    if (packet_router_update(&router, now) != ESP_OK) {
        task_link_client_handle_link_error(&top_client);
    }
    task_link_client_update(&top_client, now);
    TaskFailure failure = TASK_FAILURE_NONE;
    if (task_link_client_take_peer_failure(&top_client, &failure)) {
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
