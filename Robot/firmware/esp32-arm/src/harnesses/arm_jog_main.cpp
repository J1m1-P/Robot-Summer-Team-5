/* Runs all arm servos and steppers through USB serial jog commands. */
#include <Arduino.h>

#include <limits.h>
#include <math.h>
#include <stdlib.h>

#include "config/servo_config.h"
#include "config/stepper_config.h"
#include "drivers/servo_driver.h"
#include "drivers/stepper_driver.h"

namespace {

constexpr size_t kServoCount = 7;
constexpr size_t kStepperCount = 4;
constexpr unsigned long kStepperTelemetryPeriodMs = 100;
constexpr unsigned long kServoTelemetryPeriodMs = 1000;
constexpr float kMaxConfiguredRangeMm = 10000.0f;
constexpr long kMaxStepperTimingUs = 10000000L;
constexpr size_t kMaxCommandLength = 160;

// These are the same mechanics used by stepper_move_distanceMM(). Keeping the
// conversion here lets this harness enforce positional limits before moving.
constexpr float kXStepsPerMm = (360.0f / 1.8f) / (2.0f * 20.0f);
constexpr float kZStepsPerMm = (360.0f / 1.8f) / 8.0f;

struct ServoRuntime {
    ServoDriver driver;
    bool ready;
    uint8_t current_angle;
};

struct StepperRuntime {
    StepperDriver driver;
    bool ready;
    float range_mm;
    long position_steps;
    long last_steps_remaining;
};

const ServoConfig *const kServoConfigs[kServoCount] = {
    &habitatLeftServoConfig,
    &habitatRightServoConfig,
    &towerRotateServoConfig,
    &towerLeftServoConfig,
    &towerMiddleServoConfig,
    &towerRightServoConfig,
    &solarPanelServoConfig,
};

const StepperConfig *const kStepperConfigs[kStepperCount] = {
    &towerXConfig,
    &towerZConfig,
    &habitatXConfig,
    &habitatZConfig,
};

ServoRuntime servos[kServoCount];
StepperRuntime steppers[kStepperCount];
String serial_line;
unsigned long last_stepper_telemetry_ms = 0;
unsigned long last_servo_telemetry_ms = 0;

float steps_per_mm(const StepperRuntime &stepper) {
    return stepper.driver.config.axis == X ? kXStepsPerMm : kZStepsPerMm;
}

long range_steps(const StepperRuntime &stepper) {
    return lroundf(stepper.range_mm * steps_per_mm(stepper));
}

float position_mm(const StepperRuntime &stepper) {
    return (float)stepper.position_steps / steps_per_mm(stepper);
}

// Sends one response to the USB serial dashboard.
void output_line(String line) {
    Serial.println(line);
}

void print_usage() {
    output_line("# usage: state | stop [servo|stepper] [id] | "
                "servo angle|a|b <id> <deg> | servo neutral <id> | "
                "servo goto <id> a|b | servo pwm <id> <hz> <min_us> <max_us> | "
                "stepper mm|steps <id> <delta> | stepper home <id> | "
                "stepper range <id> <mm> | stepper timing <id> <pulse_us> <delay_us> | "
                "stepper full <id> negative|positive");
}

bool parse_long_token(const String &token, long &value_out) {
    if (token.length() == 0) return false;
    char *end = nullptr;
    const long value = strtol(token.c_str(), &end, 10);
    if (end == token.c_str() || *end != '\0') return false;
    value_out = value;
    return true;
}

bool parse_float_token(const String &token, float &value_out) {
    if (token.length() == 0) return false;
    char *end = nullptr;
    const float value = strtof(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0' || !isfinite(value)) return false;
    value_out = value;
    return true;
}

bool parse_index(const String &token, size_t count, size_t &index_out) {
    long id = 0;
    if (!parse_long_token(token, id) || id < 1 || id > (long)count) {
        output_line("# invalid motor id: " + token);
        return false;
    }
    index_out = (size_t)(id - 1);
    return true;
}

int split_tokens(String line, String tokens[], int max_tokens) {
    int count = 0;
    line.trim();
    while (count < max_tokens && line.length() > 0) {
        const int space = line.indexOf(' ');
        if (space < 0) {
            tokens[count++] = line;
            break;
        }
        tokens[count++] = line.substring(0, space);
        line = line.substring(space + 1);
        line.trim();
    }
    return count;
}

uint8_t servo_limit_angle(const ServoRuntime &servo, float requested_angle) {
    const uint8_t angle_a = servo.driver.config.anglePositionA;
    const uint8_t angle_b = servo.driver.config.anglePositionB;
    const float low = min(angle_a, angle_b);
    const float high = max(angle_a, angle_b);
    return (uint8_t)lroundf(constrain(requested_angle, low, high));
}

void report_servo(size_t index) {
    const ServoRuntime &servo = servos[index];
    output_line("servo," + String(index + 1) + "," + String(servo.current_angle) + "," +
                String(servo.driver.config.anglePositionA) + "," +
                String(servo.driver.config.anglePositionB) + "," +
                String(servo.driver.config.frequencyHz) + "," +
                String(servo.driver.config.minPulseWidthUs) + "," +
                String(servo.driver.config.maxPulseWidthUs) + "," +
                String(servo.ready ? 1 : 0));
}

void report_stepper(size_t index) {
    const StepperRuntime &stepper = steppers[index];
    output_line("stepper," + String(index + 1) + "," + String(stepper.position_steps) + "," +
                String(position_mm(stepper), 3) + "," + String(stepper.range_mm, 3) + "," +
                String(range_steps(stepper)) + "," +
                String(stepper.driver.config.stepPulseUs) + "," +
                String(stepper.driver.config.stepDelayUs) + "," +
                String(stepper.ready && stepper.driver.isMoving ? 1 : 0) + "," +
                String(stepper.ready ? 1 : 0));
}

void report_all() {
    for (size_t index = 0; index < kServoCount; ++index) report_servo(index);
    for (size_t index = 0; index < kStepperCount; ++index) report_stepper(index);
}

void set_servo_angle(size_t index, float requested_angle) {
    ServoRuntime &servo = servos[index];
    if (!servo.ready) {
        output_line("# servo " + String(index + 1) + " is unavailable");
        return;
    }
    const uint8_t limited_angle = servo_limit_angle(servo, requested_angle);
    servo_set_angle(&servo.driver, limited_angle);
    servo.current_angle = limited_angle;
    if (fabsf(requested_angle - limited_angle) > 0.01f) {
        output_line("# servo " + String(index + 1) + " angle limited to " +
                    String(limited_angle) + " deg by positions A/B");
    }
    report_servo(index);
}

void set_servo_endpoint(size_t index, bool position_b, float requested_angle) {
    if (requested_angle < 0.0f || requested_angle > 180.0f) {
        output_line("# servo endpoint must be from 0 to 180 deg");
        return;
    }
    ServoRuntime &servo = servos[index];
    const uint8_t angle = (uint8_t)lroundf(requested_angle);
    if (position_b) {
        servo.driver.config.anglePositionB = angle;
    } else {
        servo.driver.config.anglePositionA = angle;
    }
    set_servo_angle(index, servo.current_angle);
}

bool valid_servo_pwm(long frequency_hz, long min_us, long max_us) {
    if (frequency_hz <= 0 || frequency_hz > 1000 ||
        min_us < MIN_PULSE_WIDTH || max_us > MAX_PULSE_WIDTH || min_us >= max_us) {
        return false;
    }
    return (unsigned long)max_us < (1000000UL / (unsigned long)frequency_hz);
}

void set_servo_pwm(size_t index, long frequency_hz, long min_us, long max_us) {
    ServoRuntime &servo = servos[index];
    if (!servo.ready || !valid_servo_pwm(frequency_hz, min_us, max_us)) {
        output_line("# invalid servo PWM: use a positive frequency, min < max, supported "
                    "pulse widths, and max pulse shorter than the PWM period");
        return;
    }

    const ServoConfig previous = servo.driver.config;
    ServoConfig updated = previous;
    updated.frequencyHz = (uint16_t)frequency_hz;
    updated.minPulseWidthUs = (uint16_t)min_us;
    updated.maxPulseWidthUs = (uint16_t)max_us;

    servo.driver.servo.detach();
    servo.driver.attached = false;
    const esp_err_t error = servo_init(&servo.driver, updated);
    if (error == ESP_OK) {
        servo_set_angle(&servo.driver, servo.current_angle);
        servo.ready = true;
        output_line("# servo " + String(index + 1) + " PWM updated in RAM");
    } else {
        const esp_err_t restore_error = servo_init(&servo.driver, previous);
        servo.ready = restore_error == ESP_OK;
        if (servo.ready) servo_set_angle(&servo.driver, servo.current_angle);
        output_line("# servo " + String(index + 1) + " PWM update failed; previous settings " +
                    String(servo.ready ? "restored" : "could not be restored"));
    }
    report_servo(index);
}

// Updates the logical position estimate from the pulses completed since the
// previous loop. The estimate deliberately survives a stop command.
void update_stepper_position(StepperRuntime &stepper) {
    const long remaining = stepper.driver.stepsRemaining;
    if (remaining < stepper.last_steps_remaining) {
        const long completed = stepper.last_steps_remaining - remaining;
        stepper.position_steps += stepper.driver.direction ? completed : -completed;
    }
    stepper.last_steps_remaining = remaining;
}

void start_stepper_move(size_t index, long requested_delta_steps) {
    StepperRuntime &stepper = steppers[index];
    if (!stepper.ready) {
        output_line("# stepper " + String(index + 1) + " is unavailable");
        return;
    }
    if (stepper_is_moving(&stepper.driver)) {
        output_line("# stepper " + String(index + 1) + " is already moving; stop it first");
        return;
    }

    const long maximum_steps = range_steps(stepper);
    const long accepted_delta = constrain(requested_delta_steps, -maximum_steps, maximum_steps);
    if (accepted_delta == 0) {
        output_line("# stepper jog must contain at least one whole step");
        report_stepper(index);
        return;
    }

    if (accepted_delta != requested_delta_steps) {
        output_line("# stepper " + String(index + 1) + " jog magnitude limited to its full range");
    }
    stepper_move_steps(&stepper.driver, accepted_delta);
    stepper.last_steps_remaining = stepper.driver.stepsRemaining;
    report_stepper(index);
}

void stop_stepper(size_t index) {
    StepperRuntime &stepper = steppers[index];
    if (!stepper.ready) return;
    update_stepper_position(stepper);
    stepper_stop(&stepper.driver);
    stepper.last_steps_remaining = 0;
    report_stepper(index);
}

void stop_all() {
    for (size_t index = 0; index < kStepperCount; ++index) stop_stepper(index);
    output_line("# all stepper motion stopped (servo outputs hold their current angles)");
}

void process_servo_command(String tokens[], int count) {
    if (count < 3) {
        print_usage();
        return;
    }
    size_t index = 0;
    const String &operation = tokens[1];
    if (!parse_index(tokens[2], kServoCount, index)) return;

    if (operation == "neutral") {
        const float neutral = (servos[index].driver.config.anglePositionA +
                               servos[index].driver.config.anglePositionB) / 2.0f;
        set_servo_angle(index, neutral);
    } else if (operation == "goto" && count >= 4) {
        if (tokens[3] == "a") {
            set_servo_angle(index, servos[index].driver.config.anglePositionA);
        } else if (tokens[3] == "b") {
            set_servo_angle(index, servos[index].driver.config.anglePositionB);
        } else {
            output_line("# servo goto endpoint must be a or b");
        }
    } else if ((operation == "angle" || operation == "a" || operation == "b") && count >= 4) {
        float angle = 0.0f;
        if (!parse_float_token(tokens[3], angle)) {
            output_line("# invalid servo angle");
            return;
        }
        if (operation == "angle") set_servo_angle(index, angle);
        if (operation == "a") set_servo_endpoint(index, false, angle);
        if (operation == "b") set_servo_endpoint(index, true, angle);
    } else if (operation == "pwm" && count >= 6) {
        long frequency_hz = 0;
        long min_us = 0;
        long max_us = 0;
        if (!parse_long_token(tokens[3], frequency_hz) ||
            !parse_long_token(tokens[4], min_us) ||
            !parse_long_token(tokens[5], max_us)) {
            output_line("# invalid servo PWM value");
            return;
        }
        set_servo_pwm(index, frequency_hz, min_us, max_us);
    } else {
        print_usage();
    }
}

void process_stepper_command(String tokens[], int count) {
    if (count < 3) {
        print_usage();
        return;
    }
    size_t index = 0;
    const String &operation = tokens[1];
    if (!parse_index(tokens[2], kStepperCount, index)) return;
    StepperRuntime &stepper = steppers[index];

    if (operation == "home") {
        stop_stepper(index);
        stepper.position_steps = 0;
        output_line("# stepper " + String(index + 1) + " logical home set at current position");
        report_stepper(index);
    } else if (operation == "full" && count >= 4) {
        if (tokens[3] == "negative") {
            start_stepper_move(index, -range_steps(stepper));
        } else if (tokens[3] == "positive") {
            start_stepper_move(index, range_steps(stepper));
        } else {
            output_line("# stepper full direction must be negative or positive");
        }
    } else if (operation == "mm" && count >= 4) {
        float delta_mm = 0.0f;
        if (!parse_float_token(tokens[3], delta_mm)) {
            output_line("# invalid stepper distance");
            return;
        }
        delta_mm = constrain(delta_mm, -stepper.range_mm, stepper.range_mm);
        const long delta_steps = lroundf(delta_mm * steps_per_mm(stepper));
        if (delta_steps == 0) {
            output_line("# requested distance is less than one whole step");
            return;
        }
        start_stepper_move(index, delta_steps);
    } else if (operation == "steps" && count >= 4) {
        long delta_steps = 0;
        if (!parse_long_token(tokens[3], delta_steps)) {
            output_line("# invalid step count");
            return;
        }
        start_stepper_move(index, delta_steps);
    } else if (operation == "timing" && count >= 5) {
        long pulse_us = 0;
        long delay_us = 0;
        if (!parse_long_token(tokens[3], pulse_us) ||
            !parse_long_token(tokens[4], delay_us) ||
            pulse_us <= 0 || delay_us <= 0 ||
            pulse_us > kMaxStepperTimingUs || delay_us > kMaxStepperTimingUs) {
            output_line("# stepper pulse and delay must each be from 1 to " +
                        String(kMaxStepperTimingUs) + " us");
            return;
        }
        if (stepper_is_moving(&stepper.driver)) {
            output_line("# stop stepper " + String(index + 1) +
                        " before changing its pulse timing");
            return;
        }
        stepper_set_pulse_us(&stepper.driver, (uint32_t)pulse_us);
        stepper_set_delay_us(&stepper.driver, (uint32_t)delay_us);
        output_line("# stepper " + String(index + 1) + " pulse timing updated in RAM");
        report_stepper(index);
    } else if (operation == "range" && count >= 4) {
        float range_mm_value = 0.0f;
        if (!parse_float_token(tokens[3], range_mm_value) || range_mm_value <= 0.0f ||
            range_mm_value > kMaxConfiguredRangeMm) {
            output_line("# stepper range must be greater than 0 and at most " +
                        String(kMaxConfiguredRangeMm, 0) + " mm");
            return;
        }
        if (stepper_is_moving(&stepper.driver)) {
            output_line("# stop stepper " + String(index + 1) + " before changing its range");
            return;
        }
        stepper.range_mm = range_mm_value;
        output_line("# stepper " + String(index + 1) + " full range updated in RAM");
        report_stepper(index);
    } else {
        print_usage();
    }
}

void process_stop_command(String tokens[], int count) {
    if (count == 1) {
        stop_all();
        return;
    }
    if (tokens[1] != "stepper" || count < 3) {
        output_line("# stop controls stepper pulses; servos continue holding their commanded angles");
        return;
    }
    size_t index = 0;
    if (parse_index(tokens[2], kStepperCount, index)) stop_stepper(index);
}

void process_command_line(String line) {
    line.trim();
    if (line.length() == 0) return;
    String tokens[8];
    const int count = split_tokens(line, tokens, 8);
    if (count == 0) return;

    if (tokens[0] == "state") {
        report_all();
    } else if (tokens[0] == "stop") {
        process_stop_command(tokens, count);
    } else if (tokens[0] == "servo") {
        process_servo_command(tokens, count);
    } else if (tokens[0] == "stepper") {
        process_stepper_command(tokens, count);
    } else {
        print_usage();
    }
}

// Consumes serial input without readStringUntil() so a partial command cannot
// pause the high-frequency step-pulse state machines.
void handle_serial_commands() {
    while (Serial.available()) {
        const char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (serial_line.length() > 0) {
                process_command_line(serial_line);
                serial_line = "";
            }
        } else if (serial_line.length() < kMaxCommandLength) {
            serial_line += ch;
        } else {
            serial_line = "";
            output_line("# serial command discarded: line too long");
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    for (size_t index = 0; index < kServoCount; ++index) {
        servos[index].current_angle = kServoConfigs[index]->anglePositionA;
        servos[index].ready = servo_init(&servos[index].driver, *kServoConfigs[index]) == ESP_OK;
        if (!servos[index].ready) {
            Serial.println("# servo " + String(index + 1) + " initialization failed");
        }
    }

    for (size_t index = 0; index < kStepperCount; ++index) {
        steppers[index].range_mm = kStepperConfigs[index]->motionlimitMM;
        steppers[index].position_steps = 0;
        steppers[index].last_steps_remaining = 0;
        steppers[index].ready = stepper_init(&steppers[index].driver, *kStepperConfigs[index]) == ESP_OK;
        if (!steppers[index].ready) {
            Serial.println("# stepper " + String(index + 1) + " initialization failed");
        }
    }

    output_line("# Arm jog harness ready. Positions are open-loop and start at logical zero.");
    print_usage();
    report_all();
}

void loop() {
    handle_serial_commands();

    for (size_t index = 0; index < kStepperCount; ++index) {
        StepperRuntime &stepper = steppers[index];
        if (!stepper.ready) continue;
        stepper_update(&stepper.driver);
        update_stepper_position(stepper);
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_stepper_telemetry_ms >= kStepperTelemetryPeriodMs) {
        last_stepper_telemetry_ms = now_ms;
        for (size_t index = 0; index < kStepperCount; ++index) report_stepper(index);
    }
    if (now_ms - last_servo_telemetry_ms >= kServoTelemetryPeriodMs) {
        last_servo_telemetry_ms = now_ms;
        for (size_t index = 0; index < kServoCount; ++index) report_servo(index);
    }
}
