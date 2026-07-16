/* Provides the current placeholder production entry point for the arm firmware. */
#include <Arduino.h>

// Declares the placeholder arithmetic helper used during setup.
int myFunction(int, int);

// Runs the current placeholder calculation once at startup.
void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

// Leaves the placeholder production firmware idle.
void loop() {
  // put your main code here, to run repeatedly:
}

// Adds two integers for the placeholder startup example.
int myFunction(int x, int y) {
  return x + y;
}
