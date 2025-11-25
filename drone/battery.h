#pragma once
#include <Arduino.h>

#define BAT_CYCLE_US          50000UL      // 50ms between checks
#define REFERENCE_VOLTAGE     5.0
#define VOLTAGE_DIVIDER_RATIO 2.5
#define ADC_RESOLUTION        1023.0

// Battery globals
unsigned long bat_timer_loop = 0;
float bat_v = 0.0f;
float bat_read = 0.0f;

// Offset to calibrate voltage reading (tune in practice)
float bat_offset = 4.8;

void battery_setup() {
  // Nothing needed beyond analog pin ready
  // (Serial already started in main)
}

void battery_loop() {
  unsigned long now = micros();
  if (now - bat_timer_loop < BAT_CYCLE_US) {
    return;
  }
  bat_timer_loop = now;

  // Read battery voltage (adjust pin as needed; A6 as in your original code)
  bat_read = analogRead(A6);
  bat_v = VOLTAGE_DIVIDER_RATIO * (bat_read * bat_offset / ADC_RESOLUTION) * REFERENCE_VOLTAGE;
}

float battery_voltage() {
  return bat_v;
}
