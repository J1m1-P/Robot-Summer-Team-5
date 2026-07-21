#pragma once

#include <Arduino.h>
#include "config/servo_config.h"

// Initialize servo driver hardware if needed.
void servo_driver_init(void);

// Habitat servo controls.
void habitat_open(void);
void habitat_close(void);

// Tower servo controls.
void tower_open(void);
void tower_close(void);
void tower_vertical(void);
void tower_horizontal(void);

// Solar panel servo controls.
void solar_panel_extend(void);
void solar_panel_retract(void);