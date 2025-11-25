#pragma once
#include <Arduino.h>
#include <Servo.h>

const int MIN_PWM = 1000;  // microseconds, idle / min
const int MAX_PWM = 2000;  // microseconds, full throttle

Servo esc1, esc2, esc3, esc4;
int escPins[4] = {2, 3, 4, 5}; // ESC PWM pins

bool motors_armed = false;

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motors_setup() {
  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);

  // Ensure all motors are off at startup
  esc1.writeMicroseconds(MIN_PWM);
  esc2.writeMicroseconds(MIN_PWM);
  esc3.writeMicroseconds(MIN_PWM);
  esc4.writeMicroseconds(MIN_PWM);
}

void motors_arm(bool arm) {
  motors_armed = arm;
}

void motors_stop() {
  esc1.writeMicroseconds(MIN_PWM);
  esc2.writeMicroseconds(MIN_PWM);
  esc3.writeMicroseconds(MIN_PWM);
  esc4.writeMicroseconds(MIN_PWM);
}

// Flight mixer: X-quad
// Front:  M1 (front-left), M2 (front-right)
// Rear:   M3 (rear-right), M4 (rear-left)
//
// Inputs:
//  throttle_0_100 : 0..100 %
//  pitch_corr, roll_corr, yaw_corr: PID outputs in "µs" (corrections)
//                                  typical range [-200, 200]
void motors_write(float throttle_0_100,
                  float pitch_corr,
                  float roll_corr,
                  float yaw_corr) {
  if (!motors_armed || throttle_0_100 < 1.0f) {
    motors_stop();
    return;
  }

  throttle_0_100 = constrain(throttle_0_100, 0.0f, 100.0f);

  float baseThrottle = mapFloat(throttle_0_100, 0.0f, 100.0f, MIN_PWM, MAX_PWM);

  // X mixer (signs may need flip depending on frame & props)
  float m1 = baseThrottle + pitch_corr + roll_corr + yaw_corr;   // front-left
  float m2 = baseThrottle + pitch_corr - roll_corr - yaw_corr;   // front-right
  float m3 = baseThrottle - pitch_corr - roll_corr + yaw_corr;   // rear-right
  float m4 = baseThrottle - pitch_corr + roll_corr - yaw_corr;   // rear-left

  m1 = constrain(m1, MIN_PWM, MAX_PWM);
  m2 = constrain(m2, MIN_PWM, MAX_PWM);
  m3 = constrain(m3, MIN_PWM, MAX_PWM);
  m4 = constrain(m4, MIN_PWM, MAX_PWM);

  esc1.writeMicroseconds((int)m1);
  esc2.writeMicroseconds((int)m2);
  esc3.writeMicroseconds((int)m3);
  esc4.writeMicroseconds((int)m4);
}
