/* One-flash entry point for all drivetrain-side interactive test harnesses. */
#include <Arduino.h>
#include <esp_attr.h>

#include "system_test_mode.h"

void drivetrain_test_setup();
void drivetrain_test_loop();
void drivetrain_tof_test_setup();
void drivetrain_tof_test_loop();
void task_coordinator_test_setup();
void task_coordinator_test_loop();

namespace {

enum class Mode : uint8_t { NONE = 0, COORDINATION = 1, DRIVETRAIN = 2, TOF = 3 };

constexpr uint32_t kModeMagic = 0x53595444U;  // "SYTD"
RTC_NOINIT_ATTR uint32_t retained_magic;
RTC_NOINIT_ATTR uint8_t retained_mode;

Mode active_mode = Mode::NONE;
String input_line;

const char *mode_name(Mode mode) {
    switch (mode) {
        case Mode::COORDINATION: return "coordination";
        case Mode::DRIVETRAIN: return "drivetrain";
        case Mode::TOF: return "tof";
        default: return "none";
    }
}

Mode parse_mode(String name) {
    name.trim();
    name.toLowerCase();
    if (name == "coordination" || name == "coordinator") return Mode::COORDINATION;
    if (name == "drivetrain" || name == "drive") return Mode::DRIVETRAIN;
    if (name == "tof") return Mode::TOF;
    if (name == "none" || name == "menu") return Mode::NONE;
    return static_cast<Mode>(0xff);
}

bool valid_mode(Mode mode) {
    return mode == Mode::NONE || mode == Mode::COORDINATION ||
           mode == Mode::DRIVETRAIN || mode == Mode::TOF;
}

void print_menu() {
    Serial.println("SYSTEM,READY,drivetrain");
    Serial.println("SYSTEM,MODES,coordination,drivetrain,tof");
    Serial.println("# Select a runtime with: mode coordination | mode drivetrain | mode tof");
}

void start_active_mode() {
    Serial.printf("SYSTEM,MODE,%s\n", mode_name(active_mode));
    switch (active_mode) {
        case Mode::COORDINATION: task_coordinator_test_setup(); break;
        case Mode::DRIVETRAIN: drivetrain_test_setup(); break;
        case Mode::TOF: drivetrain_tof_test_setup(); break;
        default: print_menu(); break;
    }
}

void run_active_mode() {
    switch (active_mode) {
        case Mode::COORDINATION: task_coordinator_test_loop(); break;
        case Mode::DRIVETRAIN: drivetrain_test_loop(); break;
        case Mode::TOF: drivetrain_tof_test_loop(); break;
        default: delay(2); break;
    }
}

}  // namespace

bool system_test_handle_mode_command(const String &raw_line) {
    String line = raw_line;
    line.trim();
    line.toLowerCase();
    if (line == "mode") {
        Serial.printf("SYSTEM,MODE,%s\n", mode_name(active_mode));
        Serial.println("SYSTEM,MODES,coordination,drivetrain,tof");
        return true;
    }
    if (!line.startsWith("mode ")) return false;

    const Mode requested = parse_mode(line.substring(5));
    if (!valid_mode(requested)) {
        Serial.println("SYSTEM,ERROR,unknown_mode,coordination|drivetrain|tof");
        return true;
    }
    if (requested == active_mode) {
        Serial.printf("SYSTEM,MODE,%s,already_active\n", mode_name(active_mode));
        return true;
    }

    retained_magic = kModeMagic;
    retained_mode = static_cast<uint8_t>(requested);
    Serial.printf("SYSTEM,RESTARTING,%s\n", mode_name(requested));
    Serial.flush();
    delay(100);
    ESP.restart();
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    if (retained_magic == kModeMagic) {
        const Mode retained = static_cast<Mode>(retained_mode);
        if (valid_mode(retained)) active_mode = retained;
    }
    start_active_mode();
}

void loop() {
    if (active_mode != Mode::NONE) {
        run_active_mode();
        return;
    }
    while (Serial.available()) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\n' || value == '\r') {
            if (input_line.length()) {
                if (!system_test_handle_mode_command(input_line)) print_menu();
                input_line = "";
            }
        } else if (input_line.length() < 80U) {
            input_line += value;
        }
    }
    delay(2);
}

