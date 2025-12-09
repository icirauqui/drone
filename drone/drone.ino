// Racing drone – MKR WiFi 1010
// Main flight loop: IMU -> PID -> motor mix -> ESC

#include <Arduino.h>
#include "mpu.h"
#include "controller.h"
#include "battery.h"
#include "motors.h"
#include "display.h"
#include "pid.h"

// RC setpoints (from transmitter, scaled)
float rc_t = 0.0f;   // 0..100 %
float rc_p = 0.0f;   // deg
float rc_r = 0.0f;   // deg
float rc_y = 0.0f;   // deg/s (yaw rate)

// PID controller
PIDController attitudePID;

// Arming state
bool armed = false;
unsigned long armCmdStart   = 0;
unsigned long disarmCmdStart = 0;

// Main loop timing ( ~250 Hz )
unsigned long lastLoopMicros = 0;

// Forward-declare helpers
void update_arming_logic(float throttle, float yaw);
bool ready_to_arm();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    // wait for USB
  }

  // Init subsystems
  display_setup();
  battery_setup();
  mpu_setup();
  controller_setup();
  motors_setup();

  // PID tuning (you will need to tune these on the bench/flight)
  // pitch/roll: Kp, Ki, Kd   | yaw: Kp, Ki, Kd (rate)
  attitudePID.begin(
    4.0f, 0.0f, 0.05f,   // pitch/roll
    3.0f, 0.0f, 0.02f    // yaw
  );

  lastLoopMicros = micros();

  Serial.println("==== Drone firmware ready. DISARMED by default. ====");
  Serial.println("Arming:  throttle LOW + yaw RIGHT for >1s.");
  Serial.println("Disarm:  throttle LOW + yaw LEFT for >1s.");
}

void loop() {
  unsigned long now = micros();
  // ~250 Hz control loop
  if (now - lastLoopMicros < 4000) {
    return;
  }
  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.004f; // guard against weird dt
  }

  // 1) Sensors / inputs
  mpu_loop(false);                  // updates ang_p, ang_r, wz
  controller_loop(rc_t, rc_p, rc_r, rc_y);
  battery_loop();                   // updates bat_v (global in battery.h)

  bool rc_ok   = controller_signal_ok();
  bool rc_cal  = controller_is_calibrated();
  bool imu_ok  = acc_calib_ok;      // from mpu.h (set after calibration)

  if (!rc_ok) {
    // Failsafe: drop immediately on RC loss
    armed = false;
    motors_arm(false);
    motors_stop();
  }

  // 2) Handle arming / disarming based on sticks
  if (ready_to_arm()) {
    update_arming_logic(rc_t, rc_y, rc_r);
  } else {
    armed = false;
    motors_arm(false);
  }

  // 3) Flight control
  if (!armed) {
    // Safe state
    attitudePID.reset();
    motors_stop();
  } else {
    // Desired angles from RC
    float pitch_set = rc_p;  // deg
    float roll_set  = rc_r;  // deg
    float yaw_rate_set = rc_y; // deg/s (from controller mapping)

    // Measured from IMU (complementary filter)
    // ang_p, ang_r, wz are global in mpu.h
    float pitch_meas = ang_p;
    float roll_meas  = ang_r;
    float yaw_rate_meas = wz;

    // PID corrections (in "motor microsecond" units, roughly)
    float pitch_corr = attitudePID.computePitch(pitch_set, pitch_meas, dt);
    float roll_corr  = attitudePID.computeRoll(roll_set,  roll_meas,  dt);
    float yaw_corr   = attitudePID.computeYaw(yaw_rate_set, yaw_rate_meas, dt);

    // Write motors (handles throttle mapping, arming & constraints)
    motors_write(rc_t, pitch_corr, roll_corr, yaw_corr);
  }

  // 4) Simple visual status
  bool failsafe = !rc_ok;
  display_status(armed, failsafe, bat_v, rc_t, ang_p, ang_r);

  // LED status: slow blink disarmed, solid on armed
  if (armed) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
  }
}

// --- Helpers ---------------------------------------------------------

bool ready_to_arm() {
  // Require controller calibrated and IMU calibrated
  if (!controller_is_calibrated()) return false;
  if (!acc_calib_ok) return false;
  if (!controller_signal_ok()) return false;
  return true;
}

void update_arming_logic(float throttle, float yaw, float roll) {
  bool throttle_low = (throttle < 5.0f);
  bool yaw_right    = (yaw > 20.0f);   // arm command
  bool yaw_left     = (yaw < -20.0f);  // disarm command
  bool roll_right   = (roll > 20.0f);
  bool roll_left    = (roll < -20.0f);

  unsigned long now_ms = millis();

  if (!armed) {
    if (throttle_low && yaw_right && roll_right) {
      if (armCmdStart == 0) {
        armCmdStart = now_ms;
      }
      if (now_ms - armCmdStart > 1000) {
        armed = true;
        motors_arm(true);
        Serial.println("** ARMED **");
      }
    } else {
      armCmdStart = 0;
    }
    disarmCmdStart = 0;
  } else {
    if (throttle_low && yaw_left && roll_left) {
      if (disarmCmdStart == 0) {
        disarmCmdStart = now_ms;
      }
      if (now_ms - disarmCmdStart > 1000) {
        armed = false;
        motors_arm(false);
        motors_stop();
        Serial.println("** DISARMED **");
      }
    } else {
      disarmCmdStart = 0;
    }
    armCmdStart = 0;
  }
}
