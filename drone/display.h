#pragma once
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void display_setup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // If display fails, we just continue without it
    // but print error via Serial
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Drone init..."));
  display.display();
  delay(500);
}

// Call frequently; internally rate-limited
void display_status(bool armed,
                    bool failsafe,
                    float bat_v,
                    float throttle,
                    float pitch,
                    float roll) {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < 200) {
    return;  // 5 Hz update
  }
  lastUpdate = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: status
  display.setCursor(0, 0);
  display.print(armed ? F("ARMED  ") : F("DISARM "));
  if (failsafe) {
    display.print(F("FS!"));
  }

  // Line 2: battery + throttle
  display.setCursor(0, 10);
  display.print(F("Bat: "));
  display.print(bat_v, 1);
  display.print(F("V  T:"));
  display.print(throttle, 0);
  display.print(F("%"));

  // Line 3: pitch/roll
  display.setCursor(0, 20);
  display.print(F("P:"));
  display.print(pitch, 0);
  display.print(F(" R:"));
  display.print(roll, 0);

  display.display();
}
