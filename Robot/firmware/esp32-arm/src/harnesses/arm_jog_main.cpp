/* Runs all arm servos and steppers through USB serial jog commands. */
#include <Arduino.h>
#include <esp_system.h>

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
constexpr unsigned long kStepperTelemetryPeriodMs = 250;
constexpr unsigned long kServoTelemetryPeriodMs = 1000;
constexpr float kMaxConfiguredRangeMm = 10000.0f;
constexpr long kMaxStepperTimingUs = 10000000L;
constexpr size_t kMaxCommandLength = 256;
constexpr unsigned long kSequenceServoSettleMs = 1000;
constexpr float kSequenceStartToleranceMm = 0.5f;

constexpr size_t kHabitatLeftServoIndex = 0;
constexpr size_t kHabitatRightServoIndex = 1;
constexpr size_t kTowerRotateServoIndex = 2;
constexpr size_t kTowerLeftServoIndex = 3;
constexpr size_t kTowerMiddleServoIndex = 4;
constexpr size_t kTowerRightServoIndex = 5;
constexpr size_t kTowerXStepperIndex = 0;
constexpr size_t kTowerZStepperIndex = 1;
constexpr size_t kHabitatXStepperIndex = 2;
constexpr size_t kHabitatZStepperIndex = 3;

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

enum class TowerSequenceStage : uint8_t {
    Idle,
    // Pickup sequence.
    WaitVerticalAndOpen,
    PickupRaiseFirst,
    WaitHorizontal,
    PickupLowerReturn,
    WaitClosed,
    PickupRaiseSecond,
    WaitVerticalFinal,
    PickupRaiseFinal,
    // Assembly sequence.
    WaitAssemblyInitialized,
    AssemblyLowerToFirstZ,
    WaitAssemblyMiddleOpen,
    AssemblyRaiseToMiddleZ,
    AssemblyMoveLeft,
    AssemblyLowerAfterMiddle,
    WaitAssemblyRightOpen,
    AssemblyRaiseToRightZ,
    AssemblyMoveRight,
    AssemblyLowerAfterRight,
    WaitAssemblyLeftOpen,
};

enum class HabitatSequenceStage : uint8_t {
    Idle,
    // Pickup sequence.
    WaitOpen,
    RaiseFirst,
    LowerReturn,
    WaitClosed,
    RaiseFinal,
    // Placedown sequence.
    WaitPlacedownInitialized,
    PlacedownMoveRight,
    PlacedownLowerLeft,
    WaitPlacedownLeftOpen,
    PlacedownRaise,
    PlacedownMoveLeft,
    PlacedownLowerRight,
    WaitPlacedownRightOpen,
    PlacedownRaiseFinal,
};

struct TowerPickupParameters {
    float first_raise_by_mm = 50.0f;
    float return_lower_by_mm = 50.0f;
    float second_raise_by_mm = 50.0f;
    float final_raise_by_mm = 30.0f;
};

struct TowerAssemblyParameters {
    float required_start_z_mm = 80.0f;
    float first_z_target_mm = 50.0f;
    float middle_z_target_mm = 150.0f;
    float left_x_by_mm = 68.0f;
    float lower_after_middle_by_mm = 50.0f;
    float right_z_target_mm = 250.0f;
    float right_x_by_mm = 136.0f;
    float lower_after_right_by_mm = 50.0f;
};

struct TowerSequenceRuntime {
    TowerSequenceStage stage = TowerSequenceStage::Idle;
    unsigned long wait_until_ms = 0;
    TowerPickupParameters pickup;
    TowerAssemblyParameters assembly;
};

struct HabitatPickupParameters {
    float first_raise_by_mm = 30.0f;
    float return_lower_by_mm = 30.0f;
    float final_raise_by_mm = 35.0f;
};

struct HabitatPlacedownParameters {
    float required_start_z_mm = 35.0f;
    float right_x_by_mm = 92.0f;
    float first_z_target_mm = 0.0f;
    float raised_z_target_mm = 35.0f;
    float left_x_by_mm = 184.0f;
    float second_z_target_mm = 0.0f;
    float final_z_target_mm = 20.0f;
};

struct HabitatSequenceRuntime {
    HabitatSequenceStage stage = HabitatSequenceStage::Idle;
    unsigned long wait_until_ms = 0;
    HabitatPickupParameters pickup;
    HabitatPlacedownParameters placedown;
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
TowerSequenceRuntime tower_sequence;
HabitatSequenceRuntime habitat_sequence;
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

const char *reset_reason_name(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN: return "unknown";
        case ESP_RST_POWERON: return "power-on";
        case ESP_RST_EXT: return "external";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "SDIO";
        default: return "unrecognized";
    }
}

// Sends one response to the USB serial dashboard.
void output_line(String line) {
    Serial.println(line);
}

void print_usage() {
    output_line("# usage: state | tower pickup [raise1 lower raise2 raise3_mm] | "
                "tower assembly [start_z first_z middle_z left_x lower1 right_z right_x lower2_mm] | "
                "habitat pickup [raise1 lower raise2_mm] | "
                "habitat placedown [start_z right_x first_z raised_z left_x second_z final_z_mm] | "
                "stop [servo|stepper] [id] | "
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
    output_line("state_begin");
    output_line("# ESP32 reset reason: " + String(reset_reason_name(esp_reset_reason())));
    for (size_t index = 0; index < kServoCount; ++index) report_servo(index);
    for (size_t index = 0; index < kStepperCount; ++index) report_stepper(index);
    output_line("state_end");
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

bool tower_sequence_active() {
    return tower_sequence.stage != TowerSequenceStage::Idle;
}

bool habitat_sequence_active() {
    return habitat_sequence.stage != HabitatSequenceStage::Idle;
}

bool motor_sequence_active() {
    return tower_sequence_active() || habitat_sequence_active();
}

void set_sequence_servo_position(size_t index, ServoPosition position) {
    ServoRuntime &servo = servos[index];
    servo_set_position(&servo.driver, position);
    servo.current_angle = position == SERVO_POSITION_B
        ? servo.driver.config.anglePositionB
        : servo.driver.config.anglePositionA;
    report_servo(index);
}

void set_tower_claws(ServoPosition position) {
    set_sequence_servo_position(kTowerLeftServoIndex, position);
    set_sequence_servo_position(kTowerMiddleServoIndex, position);
    set_sequence_servo_position(kTowerRightServoIndex, position);
}

void set_habitat_claws(ServoPosition position) {
    set_sequence_servo_position(kHabitatLeftServoIndex, position);
    set_sequence_servo_position(kHabitatRightServoIndex, position);
}

void start_sequence_relative_move_mm(size_t index, float distance_mm) {
    StepperRuntime &stepper = steppers[index];
    start_stepper_move(
        index,
        lroundf(distance_mm * steps_per_mm(stepper)));
}

void start_sequence_move_to_mm(size_t index, float target_mm) {
    StepperRuntime &stepper = steppers[index];
    const long target_steps = lroundf(target_mm * steps_per_mm(stepper));
    start_stepper_move(index, target_steps - stepper.position_steps);
}

void abort_tower_sequence() {
    if (!tower_sequence_active()) return;
    tower_sequence.stage = TowerSequenceStage::Idle;
    stop_stepper(kTowerXStepperIndex);
    stop_stepper(kTowerZStepperIndex);
    output_line("# tower sequence aborted");
}

void abort_habitat_sequence() {
    if (!habitat_sequence_active()) return;
    habitat_sequence.stage = HabitatSequenceStage::Idle;
    stop_stepper(kHabitatZStepperIndex);
    output_line("# habitat pickup sequence aborted");
}

bool tower_hardware_ready(bool require_tower_x) {
    if (motor_sequence_active()) {
        output_line("# another automated sequence is already running");
        return false;
    }

    const size_t required_servos[] = {
        kTowerRotateServoIndex,
        kTowerLeftServoIndex,
        kTowerMiddleServoIndex,
        kTowerRightServoIndex,
    };
    for (size_t index : required_servos) {
        if (!servos[index].ready) {
            output_line("# tower sequence unavailable: servo " + String(index + 1) +
                        " is not initialized");
            return false;
        }
    }
    for (size_t index = 0; index < kStepperCount; ++index) {
        if (steppers[index].ready && stepper_is_moving(&steppers[index].driver)) {
            output_line("# tower sequence unavailable: stop all steppers first");
            return false;
        }
    }

    if (!steppers[kTowerZStepperIndex].ready) {
        output_line("# tower sequence unavailable: Tower Z stepper is not initialized");
        return false;
    }
    if (require_tower_x && !steppers[kTowerXStepperIndex].ready) {
        output_line("# tower sequence unavailable: Tower X stepper is not initialized");
        return false;
    }
    return true;
}

bool habitat_hardware_ready(bool require_habitat_x) {
    if (motor_sequence_active()) {
        output_line("# another automated sequence is already running");
        return false;
    }

    const size_t required_servos[] = {
        kHabitatLeftServoIndex,
        kHabitatRightServoIndex,
    };
    for (size_t index : required_servos) {
        if (!servos[index].ready) {
            output_line("# habitat sequence unavailable: servo " + String(index + 1) +
                        " is not initialized");
            return false;
        }
    }
    for (size_t index = 0; index < kStepperCount; ++index) {
        if (steppers[index].ready && stepper_is_moving(&steppers[index].driver)) {
            output_line("# habitat sequence unavailable: stop all steppers first");
            return false;
        }
    }
    if (!steppers[kHabitatZStepperIndex].ready) {
        output_line("# habitat sequence unavailable: Habitat Z stepper is not initialized");
        return false;
    }
    if (require_habitat_x && !steppers[kHabitatXStepperIndex].ready) {
        output_line("# habitat sequence unavailable: Habitat X stepper is not initialized");
        return false;
    }
    return true;
}

bool valid_sequence_dimension(float value_mm) {
    return isfinite(value_mm) && value_mm > 0.0f &&
        value_mm <= kMaxConfiguredRangeMm;
}

bool valid_sequence_position(float value_mm) {
    return isfinite(value_mm) && value_mm >= 0.0f &&
        value_mm <= kMaxConfiguredRangeMm;
}

void start_habitat_pickup(const HabitatPickupParameters &parameters) {
    if (!habitat_hardware_ready(false)) return;

    if (!valid_sequence_dimension(parameters.first_raise_by_mm) ||
        !valid_sequence_dimension(parameters.return_lower_by_mm) ||
        !valid_sequence_dimension(parameters.final_raise_by_mm)) {
        output_line("# habitat pickup dimensions must be greater than 0 and at most " +
                    String(kMaxConfiguredRangeMm, 0) + " mm");
        return;
    }

    StepperRuntime &habitat_z = steppers[kHabitatZStepperIndex];
    const float start_z_mm = position_mm(habitat_z);
    if (fabsf(start_z_mm) > kSequenceStartToleranceMm) {
        output_line("# habitat pickup must start within " +
                    String(kSequenceStartToleranceMm, 3) +
                    " mm of Habitat Z logical home (0 mm); current position is " +
                    String(start_z_mm, 3) + " mm");
        return;
    }

    const float largest_move = max(
        max(parameters.first_raise_by_mm, parameters.return_lower_by_mm),
        parameters.final_raise_by_mm);
    if (largest_move > habitat_z.range_mm) {
        output_line("# habitat pickup contains a Z move larger than the configured Habitat Z range");
        return;
    }

    habitat_sequence.pickup = parameters;
    output_line("# habitat pickup 1/5: open both habitat claws");
    set_habitat_claws(SERVO_POSITION_A);
    habitat_sequence.stage = HabitatSequenceStage::WaitOpen;
    habitat_sequence.wait_until_ms = millis() + kSequenceServoSettleMs;
}

void start_habitat_placedown(const HabitatPlacedownParameters &parameters) {
    if (!habitat_hardware_ready(true)) return;

    if (!valid_sequence_position(parameters.required_start_z_mm) ||
        !valid_sequence_dimension(parameters.right_x_by_mm) ||
        !valid_sequence_position(parameters.first_z_target_mm) ||
        !valid_sequence_position(parameters.raised_z_target_mm) ||
        !valid_sequence_dimension(parameters.left_x_by_mm) ||
        !valid_sequence_position(parameters.second_z_target_mm) ||
        !valid_sequence_position(parameters.final_z_target_mm)) {
        output_line("# habitat placedown positions must be from 0 to " +
                    String(kMaxConfiguredRangeMm, 0) +
                    " mm and X travel distances must be greater than 0");
        return;
    }

    if (parameters.first_z_target_mm >= parameters.required_start_z_mm ||
        parameters.raised_z_target_mm <= parameters.first_z_target_mm ||
        parameters.second_z_target_mm >= parameters.raised_z_target_mm ||
        parameters.final_z_target_mm <= parameters.second_z_target_mm) {
        output_line("# habitat placedown Z dimensions conflict with the required down/up order");
        return;
    }

    StepperRuntime &habitat_x = steppers[kHabitatXStepperIndex];
    StepperRuntime &habitat_z = steppers[kHabitatZStepperIndex];
    const float start_x_mm = position_mm(habitat_x);
    const float start_z_mm = position_mm(habitat_z);
    if (fabsf(start_x_mm) > kSequenceStartToleranceMm) {
        output_line("# habitat placedown must start within " +
                    String(kSequenceStartToleranceMm, 3) +
                    " mm of Habitat X logical home (0 mm); current position is " +
                    String(start_x_mm, 3) + " mm");
        return;
    }
    if (fabsf(start_z_mm - parameters.required_start_z_mm) >
        kSequenceStartToleranceMm) {
        output_line("# habitat placedown must start within " +
                    String(kSequenceStartToleranceMm, 3) +
                    " mm of Habitat Z = " +
                    String(parameters.required_start_z_mm, 3) +
                    " mm; current position is " +
                    String(start_z_mm, 3) + " mm");
        return;
    }

    const float z_moves[] = {
        fabsf(parameters.first_z_target_mm - parameters.required_start_z_mm),
        fabsf(parameters.raised_z_target_mm - parameters.first_z_target_mm),
        fabsf(parameters.second_z_target_mm - parameters.raised_z_target_mm),
        fabsf(parameters.final_z_target_mm - parameters.second_z_target_mm),
    };
    for (float move_mm : z_moves) {
        if (move_mm > habitat_z.range_mm) {
            output_line("# habitat placedown contains a Z move larger than the configured Habitat Z range");
            return;
        }
    }
    if (parameters.right_x_by_mm > habitat_x.range_mm ||
        parameters.left_x_by_mm > habitat_x.range_mm) {
        output_line("# habitat placedown contains an X move larger than the configured Habitat X range");
        return;
    }

    habitat_sequence.placedown = parameters;
    output_line("# habitat placedown preparation: hold Z at " +
                String(parameters.required_start_z_mm, 3) +
                " mm and keep both habitat claws closed");
    set_habitat_claws(SERVO_POSITION_B);
    habitat_sequence.stage = HabitatSequenceStage::WaitPlacedownInitialized;
    habitat_sequence.wait_until_ms = millis() + kSequenceServoSettleMs;
}

void start_tower_pickup(const TowerPickupParameters &parameters) {
    if (!tower_hardware_ready(false)) return;

    if (!valid_sequence_dimension(parameters.first_raise_by_mm) ||
        !valid_sequence_dimension(parameters.return_lower_by_mm) ||
        !valid_sequence_dimension(parameters.second_raise_by_mm) ||
        !valid_sequence_dimension(parameters.final_raise_by_mm)) {
        output_line("# tower pickup dimensions must be greater than 0 and at most " +
                    String(kMaxConfiguredRangeMm, 0) + " mm");
        return;
    }

    StepperRuntime &tower_z = steppers[kTowerZStepperIndex];
    if (tower_z.position_steps != 0) {
        output_line("# tower pickup must start at Tower Z logical home (0 mm)");
        return;
    }
    const float largest_move = max(
        max(parameters.first_raise_by_mm, parameters.return_lower_by_mm),
        max(parameters.second_raise_by_mm, parameters.final_raise_by_mm));
    if (largest_move > tower_z.range_mm) {
        output_line("# tower pickup contains a Z move larger than the configured Tower Z range");
        return;
    }

    tower_sequence.pickup = parameters;
    output_line("# tower pickup 1/8: rotate vertical and open all tower claws");
    set_sequence_servo_position(kTowerRotateServoIndex, SERVO_POSITION_B);
    set_tower_claws(SERVO_POSITION_A);
    tower_sequence.stage = TowerSequenceStage::WaitVerticalAndOpen;
    tower_sequence.wait_until_ms = millis() + kSequenceServoSettleMs;
}

void start_tower_assembly(const TowerAssemblyParameters &parameters) {
    if (!tower_hardware_ready(true)) return;

    const float dimensions[] = {
        parameters.required_start_z_mm,
        parameters.first_z_target_mm,
        parameters.middle_z_target_mm,
        parameters.left_x_by_mm,
        parameters.lower_after_middle_by_mm,
        parameters.right_z_target_mm,
        parameters.right_x_by_mm,
        parameters.lower_after_right_by_mm,
    };
    for (float value_mm : dimensions) {
        if (!valid_sequence_dimension(value_mm)) {
            output_line("# tower assembly dimensions must be greater than 0 and at most " +
                        String(kMaxConfiguredRangeMm, 0) + " mm");
            return;
        }
    }

    const float z_after_middle_lower =
        parameters.middle_z_target_mm - parameters.lower_after_middle_by_mm;
    if (parameters.first_z_target_mm >= parameters.required_start_z_mm ||
        parameters.middle_z_target_mm <= parameters.first_z_target_mm ||
        z_after_middle_lower < 0.0f ||
        parameters.right_z_target_mm <= z_after_middle_lower) {
        output_line("# tower assembly Z dimensions conflict with the required down/up order");
        return;
    }

    StepperRuntime &tower_x = steppers[kTowerXStepperIndex];
    StepperRuntime &tower_z = steppers[kTowerZStepperIndex];
    const float actual_start_z_mm = position_mm(tower_z);
    if (fabsf(actual_start_z_mm - parameters.required_start_z_mm) >
        kSequenceStartToleranceMm) {
        output_line("# tower assembly must start within " +
                    String(kSequenceStartToleranceMm, 3) +
                    " mm of Tower Z = " +
                    String(parameters.required_start_z_mm, 3) +
                    " mm; current position is " +
                    String(actual_start_z_mm, 3) + " mm");
        return;
    }
    if (tower_x.position_steps != 0) {
        output_line("# tower assembly must start at Tower X logical home (0 mm)");
        return;
    }

    const float z_moves[] = {
        fabsf(parameters.first_z_target_mm - parameters.required_start_z_mm),
        fabsf(parameters.middle_z_target_mm - parameters.first_z_target_mm),
        parameters.lower_after_middle_by_mm,
        fabsf(parameters.right_z_target_mm - z_after_middle_lower),
        parameters.lower_after_right_by_mm,
    };
    for (float move_mm : z_moves) {
        if (move_mm > tower_z.range_mm) {
            output_line("# tower assembly contains a Z move larger than the configured Tower Z range");
            return;
        }
    }
    if (parameters.left_x_by_mm > tower_x.range_mm ||
        parameters.right_x_by_mm > tower_x.range_mm) {
        output_line("# tower assembly contains an X move larger than the configured Tower X range");
        return;
    }

    tower_sequence.assembly = parameters;
    output_line("# tower assembly 1/11: hold Z at " +
                String(parameters.required_start_z_mm, 3) +
                " mm, rotate vertical, and close all tower claws");
    set_sequence_servo_position(kTowerRotateServoIndex, SERVO_POSITION_B);
    set_tower_claws(SERVO_POSITION_B);
    tower_sequence.stage = TowerSequenceStage::WaitAssemblyInitialized;
    tower_sequence.wait_until_ms = millis() + kSequenceServoSettleMs;
}

bool tower_servo_wait_complete(unsigned long now_ms) {
    return static_cast<int32_t>(now_ms - tower_sequence.wait_until_ms) >= 0;
}

void update_tower_sequence(unsigned long now_ms) {
    switch (tower_sequence.stage) {
        case TowerSequenceStage::Idle:
            return;

        case TowerSequenceStage::WaitVerticalAndOpen:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower pickup 2/8: raise Tower Z by " +
                        String(tower_sequence.pickup.first_raise_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, tower_sequence.pickup.first_raise_by_mm);
            tower_sequence.stage = TowerSequenceStage::PickupRaiseFirst;
            return;

        case TowerSequenceStage::PickupRaiseFirst:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower pickup 3/8: rotate horizontal");
            set_sequence_servo_position(kTowerRotateServoIndex, SERVO_POSITION_A);
            tower_sequence.stage = TowerSequenceStage::WaitHorizontal;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitHorizontal:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower pickup 4/8: lower Tower Z by " +
                        String(tower_sequence.pickup.return_lower_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, -tower_sequence.pickup.return_lower_by_mm);
            tower_sequence.stage = TowerSequenceStage::PickupLowerReturn;
            return;

        case TowerSequenceStage::PickupLowerReturn:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower pickup 5/8: close all tower claws");
            set_tower_claws(SERVO_POSITION_B);
            tower_sequence.stage = TowerSequenceStage::WaitClosed;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitClosed:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower pickup 6/8: raise Tower Z by " +
                        String(tower_sequence.pickup.second_raise_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, tower_sequence.pickup.second_raise_by_mm);
            tower_sequence.stage = TowerSequenceStage::PickupRaiseSecond;
            return;

        case TowerSequenceStage::PickupRaiseSecond:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower pickup 7/8: rotate vertical");
            set_sequence_servo_position(kTowerRotateServoIndex, SERVO_POSITION_B);
            tower_sequence.stage = TowerSequenceStage::WaitVerticalFinal;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitVerticalFinal:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower pickup 8/8: raise Tower Z by " +
                        String(tower_sequence.pickup.final_raise_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, tower_sequence.pickup.final_raise_by_mm);
            tower_sequence.stage = TowerSequenceStage::PickupRaiseFinal;
            return;

        case TowerSequenceStage::PickupRaiseFinal:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            tower_sequence.stage = TowerSequenceStage::Idle;
            output_line("# tower pickup complete; Tower Z = " +
                        String(position_mm(steppers[kTowerZStepperIndex]), 3) + " mm");
            return;

        case TowerSequenceStage::WaitAssemblyInitialized:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower assembly 2/11: move Tower Z down to absolute " +
                        String(tower_sequence.assembly.first_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kTowerZStepperIndex, tower_sequence.assembly.first_z_target_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyLowerToFirstZ;
            return;

        case TowerSequenceStage::AssemblyLowerToFirstZ:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower assembly 3/11: open tower middle claw");
            set_sequence_servo_position(kTowerMiddleServoIndex, SERVO_POSITION_A);
            tower_sequence.stage = TowerSequenceStage::WaitAssemblyMiddleOpen;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitAssemblyMiddleOpen:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower assembly 4/11: move Tower Z up to absolute " +
                        String(tower_sequence.assembly.middle_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kTowerZStepperIndex, tower_sequence.assembly.middle_z_target_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyRaiseToMiddleZ;
            return;

        case TowerSequenceStage::AssemblyRaiseToMiddleZ:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower assembly 5/11: move Tower X left by " +
                        String(tower_sequence.assembly.left_x_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerXStepperIndex, -tower_sequence.assembly.left_x_by_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyMoveLeft;
            return;

        case TowerSequenceStage::AssemblyMoveLeft:
            if (stepper_is_moving(&steppers[kTowerXStepperIndex].driver)) return;
            output_line("# tower assembly 6/11: move Tower Z down by " +
                        String(tower_sequence.assembly.lower_after_middle_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, -tower_sequence.assembly.lower_after_middle_by_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyLowerAfterMiddle;
            return;

        case TowerSequenceStage::AssemblyLowerAfterMiddle:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower assembly 7/11: open tower right claw");
            set_sequence_servo_position(kTowerRightServoIndex, SERVO_POSITION_A);
            tower_sequence.stage = TowerSequenceStage::WaitAssemblyRightOpen;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitAssemblyRightOpen:
            if (!tower_servo_wait_complete(now_ms)) return;
            output_line("# tower assembly 8/11: move Tower Z up to absolute " +
                        String(tower_sequence.assembly.right_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kTowerZStepperIndex, tower_sequence.assembly.right_z_target_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyRaiseToRightZ;
            return;

        case TowerSequenceStage::AssemblyRaiseToRightZ:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower assembly 9/11: move Tower X right by " +
                        String(tower_sequence.assembly.right_x_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerXStepperIndex, tower_sequence.assembly.right_x_by_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyMoveRight;
            return;

        case TowerSequenceStage::AssemblyMoveRight:
            if (stepper_is_moving(&steppers[kTowerXStepperIndex].driver)) return;
            output_line("# tower assembly 10/11: move Tower Z down by " +
                        String(tower_sequence.assembly.lower_after_right_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kTowerZStepperIndex, -tower_sequence.assembly.lower_after_right_by_mm);
            tower_sequence.stage = TowerSequenceStage::AssemblyLowerAfterRight;
            return;

        case TowerSequenceStage::AssemblyLowerAfterRight:
            if (stepper_is_moving(&steppers[kTowerZStepperIndex].driver)) return;
            output_line("# tower assembly 11/11: open tower left claw");
            set_sequence_servo_position(kTowerLeftServoIndex, SERVO_POSITION_A);
            tower_sequence.stage = TowerSequenceStage::WaitAssemblyLeftOpen;
            tower_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case TowerSequenceStage::WaitAssemblyLeftOpen:
            if (!tower_servo_wait_complete(now_ms)) return;
            tower_sequence.stage = TowerSequenceStage::Idle;
            output_line("# tower assembly complete; Tower X = " +
                        String(position_mm(steppers[kTowerXStepperIndex]), 3) +
                        " mm, Tower Z = " +
                        String(position_mm(steppers[kTowerZStepperIndex]), 3) + " mm");
            return;
    }
}

bool habitat_servo_wait_complete(unsigned long now_ms) {
    return static_cast<int32_t>(now_ms - habitat_sequence.wait_until_ms) >= 0;
}

void update_habitat_sequence(unsigned long now_ms) {
    switch (habitat_sequence.stage) {
        case HabitatSequenceStage::Idle:
            return;

        case HabitatSequenceStage::WaitOpen:
            if (!habitat_servo_wait_complete(now_ms)) return;
            output_line("# habitat pickup 2/5: move Habitat Z up by " +
                        String(habitat_sequence.pickup.first_raise_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kHabitatZStepperIndex, habitat_sequence.pickup.first_raise_by_mm);
            habitat_sequence.stage = HabitatSequenceStage::RaiseFirst;
            return;

        case HabitatSequenceStage::RaiseFirst:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            output_line("# habitat pickup 3/5: move Habitat Z down by " +
                        String(habitat_sequence.pickup.return_lower_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kHabitatZStepperIndex, -habitat_sequence.pickup.return_lower_by_mm);
            habitat_sequence.stage = HabitatSequenceStage::LowerReturn;
            return;

        case HabitatSequenceStage::LowerReturn:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            output_line("# habitat pickup 4/5: close both habitat claws");
            set_habitat_claws(SERVO_POSITION_B);
            habitat_sequence.stage = HabitatSequenceStage::WaitClosed;
            habitat_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case HabitatSequenceStage::WaitClosed:
            if (!habitat_servo_wait_complete(now_ms)) return;
            output_line("# habitat pickup 5/5: move Habitat Z up by " +
                        String(habitat_sequence.pickup.final_raise_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kHabitatZStepperIndex, habitat_sequence.pickup.final_raise_by_mm);
            habitat_sequence.stage = HabitatSequenceStage::RaiseFinal;
            return;

        case HabitatSequenceStage::RaiseFinal:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            habitat_sequence.stage = HabitatSequenceStage::Idle;
            output_line("# habitat pickup complete; Habitat Z = " +
                        String(position_mm(steppers[kHabitatZStepperIndex]), 3) + " mm");
            return;

        case HabitatSequenceStage::WaitPlacedownInitialized:
            if (!habitat_servo_wait_complete(now_ms)) return;
            output_line("# habitat placedown 1/8: move Habitat X right by " +
                        String(habitat_sequence.placedown.right_x_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kHabitatXStepperIndex, habitat_sequence.placedown.right_x_by_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownMoveRight;
            return;

        case HabitatSequenceStage::PlacedownMoveRight:
            if (stepper_is_moving(&steppers[kHabitatXStepperIndex].driver)) return;
            output_line("# habitat placedown 2/8: move Habitat Z down to absolute " +
                        String(habitat_sequence.placedown.first_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kHabitatZStepperIndex, habitat_sequence.placedown.first_z_target_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownLowerLeft;
            return;

        case HabitatSequenceStage::PlacedownLowerLeft:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            output_line("# habitat placedown 3/8: open left habitat claw");
            set_sequence_servo_position(kHabitatLeftServoIndex, SERVO_POSITION_A);
            habitat_sequence.stage = HabitatSequenceStage::WaitPlacedownLeftOpen;
            habitat_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case HabitatSequenceStage::WaitPlacedownLeftOpen:
            if (!habitat_servo_wait_complete(now_ms)) return;
            output_line("# habitat placedown 4/8: move Habitat Z up to absolute " +
                        String(habitat_sequence.placedown.raised_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kHabitatZStepperIndex, habitat_sequence.placedown.raised_z_target_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownRaise;
            return;

        case HabitatSequenceStage::PlacedownRaise:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            output_line("# habitat placedown 5/8: move Habitat X left by " +
                        String(habitat_sequence.placedown.left_x_by_mm, 3) + " mm");
            start_sequence_relative_move_mm(
                kHabitatXStepperIndex, -habitat_sequence.placedown.left_x_by_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownMoveLeft;
            return;

        case HabitatSequenceStage::PlacedownMoveLeft:
            if (stepper_is_moving(&steppers[kHabitatXStepperIndex].driver)) return;
            output_line("# habitat placedown 6/8: move Habitat Z down to absolute " +
                        String(habitat_sequence.placedown.second_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kHabitatZStepperIndex, habitat_sequence.placedown.second_z_target_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownLowerRight;
            return;

        case HabitatSequenceStage::PlacedownLowerRight:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            output_line("# habitat placedown 7/8: open right habitat claw");
            set_sequence_servo_position(kHabitatRightServoIndex, SERVO_POSITION_A);
            habitat_sequence.stage = HabitatSequenceStage::WaitPlacedownRightOpen;
            habitat_sequence.wait_until_ms = now_ms + kSequenceServoSettleMs;
            return;

        case HabitatSequenceStage::WaitPlacedownRightOpen:
            if (!habitat_servo_wait_complete(now_ms)) return;
            output_line("# habitat placedown 8/8: move Habitat Z up to absolute " +
                        String(habitat_sequence.placedown.final_z_target_mm, 3) + " mm");
            start_sequence_move_to_mm(
                kHabitatZStepperIndex, habitat_sequence.placedown.final_z_target_mm);
            habitat_sequence.stage = HabitatSequenceStage::PlacedownRaiseFinal;
            return;

        case HabitatSequenceStage::PlacedownRaiseFinal:
            if (stepper_is_moving(&steppers[kHabitatZStepperIndex].driver)) return;
            habitat_sequence.stage = HabitatSequenceStage::Idle;
            output_line("# habitat placedown complete; Habitat X = " +
                        String(position_mm(steppers[kHabitatXStepperIndex]), 3) +
                        " mm, Habitat Z = " +
                        String(position_mm(steppers[kHabitatZStepperIndex]), 3) + " mm");
            return;
    }
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

void process_tower_command(String tokens[], int count) {
    if (count < 2) {
        print_usage();
        return;
    }

    if (tokens[1] == "pickup" || tokens[1] == "sequence") {
        TowerPickupParameters parameters;
        if (count != 2 && count != 6) {
            output_line("# tower pickup expects 4 dimensions: first_raise, return_lower, "
                        "second_raise, final_raise (mm)");
            return;
        }
        if (count == 6 &&
            (!parse_float_token(tokens[2], parameters.first_raise_by_mm) ||
             !parse_float_token(tokens[3], parameters.return_lower_by_mm) ||
             !parse_float_token(tokens[4], parameters.second_raise_by_mm) ||
             !parse_float_token(tokens[5], parameters.final_raise_by_mm))) {
            output_line("# invalid tower pickup dimension");
            return;
        }
        start_tower_pickup(parameters);
        return;
    }

    if (tokens[1] == "assembly") {
        TowerAssemblyParameters parameters;
        if (count != 2 && count != 10) {
            output_line("# tower assembly expects 8 dimensions: start_z, first_z, middle_z, "
                        "left_x, lower1, right_z, right_x, lower2 (mm)");
            return;
        }
        if (count == 10 &&
            (!parse_float_token(tokens[2], parameters.required_start_z_mm) ||
             !parse_float_token(tokens[3], parameters.first_z_target_mm) ||
             !parse_float_token(tokens[4], parameters.middle_z_target_mm) ||
             !parse_float_token(tokens[5], parameters.left_x_by_mm) ||
             !parse_float_token(tokens[6], parameters.lower_after_middle_by_mm) ||
             !parse_float_token(tokens[7], parameters.right_z_target_mm) ||
             !parse_float_token(tokens[8], parameters.right_x_by_mm) ||
             !parse_float_token(tokens[9], parameters.lower_after_right_by_mm))) {
            output_line("# invalid tower assembly dimension");
            return;
        }
        start_tower_assembly(parameters);
        return;
    }

    print_usage();
}

void process_habitat_command(String tokens[], int count) {
    if (count < 2) {
        print_usage();
        return;
    }

    if (tokens[1] == "pickup") {
        HabitatPickupParameters parameters;
        if (count != 2 && count != 5) {
            output_line("# habitat pickup expects 3 dimensions: first_raise, return_lower, "
                        "final_raise (mm)");
            return;
        }
        if (count == 5 &&
            (!parse_float_token(tokens[2], parameters.first_raise_by_mm) ||
             !parse_float_token(tokens[3], parameters.return_lower_by_mm) ||
             !parse_float_token(tokens[4], parameters.final_raise_by_mm))) {
            output_line("# invalid habitat pickup dimension");
            return;
        }
        start_habitat_pickup(parameters);
        return;
    }

    if (tokens[1] == "placedown") {
        HabitatPlacedownParameters parameters;
        if (count != 2 && count != 9) {
            output_line("# habitat placedown expects 7 dimensions: start_z, right_x, first_z, "
                        "raised_z, left_x, second_z, final_z (mm)");
            return;
        }
        if (count == 9 &&
            (!parse_float_token(tokens[2], parameters.required_start_z_mm) ||
             !parse_float_token(tokens[3], parameters.right_x_by_mm) ||
             !parse_float_token(tokens[4], parameters.first_z_target_mm) ||
             !parse_float_token(tokens[5], parameters.raised_z_target_mm) ||
             !parse_float_token(tokens[6], parameters.left_x_by_mm) ||
             !parse_float_token(tokens[7], parameters.second_z_target_mm) ||
             !parse_float_token(tokens[8], parameters.final_z_target_mm))) {
            output_line("# invalid habitat placedown dimension");
            return;
        }
        start_habitat_placedown(parameters);
        return;
    }

    print_usage();
}

void process_command_line(String line) {
    line.trim();
    if (line.length() == 0) return;
    String tokens[12];
    const int count = split_tokens(line, tokens, 12);
    if (count == 0) return;

    if (tokens[0] == "state") {
        report_all();
    } else if (tokens[0] == "stop") {
        abort_tower_sequence();
        abort_habitat_sequence();
        process_stop_command(tokens, count);
    } else if (tokens[0] == "tower") {
        process_tower_command(tokens, count);
    } else if (tokens[0] == "habitat") {
        process_habitat_command(tokens, count);
    } else if (motor_sequence_active()) {
        output_line("# manual motor commands are locked while an automated sequence is running; "
                    "press STOP to abort");
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
        Serial.println("# servo " + String(index + 1) + " pin " +
                       String(kServoConfigs[index]->pin) + " init " +
                       String(servos[index].ready ? "ok" : "failed"));
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
    pinMode(13, OUTPUT); 
    digitalWrite(13, LOW);
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
    update_tower_sequence(now_ms);
    update_habitat_sequence(now_ms);
    if (now_ms - last_stepper_telemetry_ms >= kStepperTelemetryPeriodMs) {
        last_stepper_telemetry_ms = now_ms;
        for (size_t index = 0; index < kStepperCount; ++index) report_stepper(index);
    }
    if (now_ms - last_servo_telemetry_ms >= kServoTelemetryPeriodMs) {
        last_servo_telemetry_ms = now_ms;
        for (size_t index = 0; index < kServoCount; ++index) report_servo(index);
    }
}
