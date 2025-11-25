#pragma once
#include <Arduino.h>

struct PIDAxis {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float previous_error;
};

class PIDController {
public:
  PIDAxis pitch;
  PIDAxis roll;
  PIDAxis yaw;

  void begin(float Kp_pr, float Ki_pr, float Kd_pr,
             float Kp_y,  float Ki_y,  float Kd_y) {
    initAxis(pitch, Kp_pr, Ki_pr, Kd_pr);
    initAxis(roll,  Kp_pr, Ki_pr, Kd_pr);
    initAxis(yaw,   Kp_y,  Ki_y,  Kd_y);
  }

  float computePitch(float setpoint, float measurement, float dt) {
    return compute(pitch, setpoint, measurement, dt);
  }

  float computeRoll(float setpoint, float measurement, float dt) {
    return compute(roll, setpoint, measurement, dt);
  }

  float computeYaw(float setpoint, float measurement, float dt) {
    return compute(yaw, setpoint, measurement, dt);
  }

  void reset() {
    pitch.integral = 0.0f;
    pitch.previous_error = 0.0f;
    roll.integral = 0.0f;
    roll.previous_error = 0.0f;
    yaw.integral = 0.0f;
    yaw.previous_error = 0.0f;
  }

private:
  void initAxis(PIDAxis &axis, float Kp, float Ki, float Kd) {
    axis.Kp = Kp;
    axis.Ki = Ki;
    axis.Kd = Kd;
    axis.integral = 0.0f;
    axis.previous_error = 0.0f;
  }

  float compute(PIDAxis &axis, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) dt = 0.004f;

    float error = setpoint - measurement;
    axis.integral += error * dt;

    // Anti-windup
    axis.integral = constrain(axis.integral, -300.0f, 300.0f);

    float derivative = (error - axis.previous_error) / dt;
    axis.previous_error = error;

    float out = axis.Kp * error + axis.Ki * axis.integral + axis.Kd * derivative;
    // Limit PID correction to reasonable range for mixing
    out = constrain(out, -200.0f, 200.0f);
    return out;
  }
};
