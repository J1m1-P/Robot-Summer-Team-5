#pragma once

#include <stdint.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define LOW 0x0
#define HIGH 0x1
#define INPUT_PULLUP 0x5
#define FALLING 0x2

using InterruptHandler = void (*)();

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
int digitalPinToInterrupt(uint8_t pin);
void attachInterrupt(int interrupt, InterruptHandler handler, int mode);
void noInterrupts();
void interrupts();
