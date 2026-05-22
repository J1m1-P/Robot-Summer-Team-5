#include <Arduino.h>
#include <Wire.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define I2C_SDA 15
#define I2C_SCL 16

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OledSetup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display_handler.display();
  delay(1000); 
  display_handler.clearDisplay();

  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
}

volatile bool detected = true;

void IRAM_ATTR tapeDetected() {
  detected = !detected;
  display_handler.clearDisplay();
}

void setup() {
  bool success = Wire.begin(I2C_SDA, I2C_SCL);
  OledSetup();
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(5), tapeDetected, CHANGE);
}

void loop() {
  display_handler.setCursor(0, 0);
  display_handler.println("Tape Detected: " + String(detected));
  display_handler.display();
  // if (digitalRead(5) == HIGH) {
  //   display_handler.clearDisplay();
  //   display_handler.setCursor(0, 0);
  //   display_handler.println("Tape Detected!");
  //   display_handler.display();
  // }
  // else {
  //   display_handler.clearDisplay();
  //   display_handler.setCursor(0, 0);
  //   display_handler.println("No Tape Detected");
  //   display_handler.display();
  // }
}
