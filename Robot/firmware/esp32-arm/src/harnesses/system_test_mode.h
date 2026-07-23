#pragma once

#include <Arduino.h>

// Handles the shared "mode <name>" and "mode" commands. A mode change is
// retained across the software restart used to hand exclusive hardware
// ownership to the selected harness.
bool system_test_handle_mode_command(const String &line);

