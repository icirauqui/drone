#pragma once
#include <Arduino.h>

/*
 * RC mapping:
 *  ROLL    -> PIN_R
 *  PITCH   -> PIN_P
 *  THROTTLE-> PIN_T
 *  YAW     -> PIN_Y
 *
 * All channels are read via interrupts and converted to:
 *  rc_t: 0..100 (%)
 *  rc_p: -30..30 (deg)
 *  rc_r: -30..30 (deg)
 *  rc_y: -150..150 (deg/s target)
 */

#define PIN_R 6
#define PIN_P 7
#define PIN_T 8
#define PIN_Y 9

long loop_timer, exec_time;

// Consignment values
float rc_t_cons, rc_p_cons, rc_r_cons, rc_y_cons;

// Calibration
bool cal = false;
int cal_loops = 0;
int cal_loops_req = 3;
int center_delay = 0;
int center_delay_req = 200;
int center_raw_cum = 0;
int center_raw_cum_req = 200;

// Corner-loop states (stick pattern)
bool tl1 = false;
bool tr1 = false;
bool bl1 = false;
bool br1 = false;
bool tl2 = false;
bool tr2 = false;
bool bl2 = false;
bool br2 = false;

// RC signal loss detection
volatile unsigned long last_rc_update = 0;
bool rc_signal_lost = true;

// INTERRUPT ROLL
volatile long r_high_us;
volatile int rc_r_raw;
int range_r_raw[2] = {1500, 1500};
int center_r_raw = 1500;
int center_r_raw_cum = 0;
void IntR() {
  if (digitalRead(PIN_R) == HIGH) r_high_us = micros();
  else {
    rc_r_raw  = micros() - r_high_us;
    last_rc_update = micros();
    rc_signal_lost = false;
  }
}

// INTERRUPT PITCH
volatile long p_high_us;
volatile int rc_p_raw;
int range_p_raw[2] = {1500, 1500};
int center_p_raw = 1500;
int center_p_raw_cum = 0;
void IntP() {
  if (digitalRead(PIN_P) == HIGH) p_high_us = micros();
  else {
    rc_p_raw  = micros() - p_high_us;
    last_rc_update = micros();
    rc_signal_lost = false;
  }
}

// INTERRUPT THROTTLE
volatile long t_high_us;
volatile int rc_t_raw;
int range_t_raw[2] = {1500, 1500};
int center_t_raw = 1500;
int center_t_raw_cum = 0;
void IntT() {
  if (digitalRead(PIN_T) == HIGH) t_high_us = micros();
  else {
    rc_t_raw  = micros() - t_high_us;
    last_rc_update = micros();
    rc_signal_lost = false;
  }
}

// INTERRUPT YAW
volatile long y_high_us;
volatile int rc_y_raw;
int range_y_raw[2] = {1500, 1500};
int center_y_raw = 1500;
int center_y_raw_cum = 0;
void IntY() {
  if (digitalRead(PIN_Y) == HIGH) y_high_us = micros();
  else {
    rc_y_raw  = micros() - y_high_us;
    last_rc_update = micros();
    rc_signal_lost = false;
  }
}

void controller_setup() {
  pinMode(PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_R), IntR, CHANGE);

  pinMode(PIN_P, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_P), IntP, CHANGE);

  pinMode(PIN_T, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_T), IntT, CHANGE);

  pinMode(PIN_Y, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_Y), IntY, CHANGE);

  cal_loops = 0;
  cal = false;

  Serial.println("Controller setup complete");
}

void CalibrateChannels() {
  if (rc_r_raw < range_r_raw[0] && rc_r_raw > 800)
    range_r_raw[0] = rc_r_raw;
  if (rc_r_raw > range_r_raw[1] && rc_r_raw < 2200)
    range_r_raw[1] = rc_r_raw;

  if (rc_p_raw < range_p_raw[0] && rc_p_raw > 800)
    range_p_raw[0] = rc_p_raw;
  if (rc_p_raw > range_p_raw[1] && rc_p_raw < 2200)
    range_p_raw[1] = rc_p_raw;

  if (rc_t_raw < range_t_raw[0] && rc_t_raw > 800)
    range_t_raw[0] = rc_t_raw;
  if (rc_t_raw > range_t_raw[1] && rc_t_raw < 2200)
    range_t_raw[1] = rc_t_raw;

  if (rc_y_raw < range_y_raw[0] && rc_y_raw > 800)
    range_y_raw[0] = rc_y_raw;
  if (rc_y_raw > range_y_raw[1] && rc_y_raw < 2200)
    range_y_raw[1] = rc_y_raw;
}

void PrintCalInfo(bool new_line) {
  Serial.print("Calibration (");
  Serial.print(cal_loops);
  Serial.print("/");
  Serial.print(cal_loops_req);
  Serial.print(") : ");

  Serial.print("R: [");
  Serial.print(range_r_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_r_raw);
  Serial.print(" - ");
  Serial.print(range_r_raw[1]);
  Serial.print("] ");

  Serial.print("P: [");
  Serial.print(range_p_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_p_raw);
  Serial.print(" - ");
  Serial.print(range_p_raw[1]);
  Serial.print("] ");

  Serial.print("T: [");
  Serial.print(range_t_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_t_raw);
  Serial.print(" - ");
  Serial.print(range_t_raw[1]);
  Serial.print("] ");

  Serial.print("Y: [");
  Serial.print(range_y_raw[0]);
  Serial.print(" - ");
  Serial.print(rc_y_raw);
  Serial.print(" - ");
  Serial.print(range_y_raw[1]);
  Serial.print("] ");

  if (new_line) {
    Serial.println();
  }
}

void PrintConsignment(bool new_line) {
  Serial.print("R: ");
  Serial.print(rc_r_cons);
  Serial.print("\tP: ");
  Serial.print(rc_p_cons);
  Serial.print("\tT: ");
  Serial.print(rc_t_cons);
  Serial.print("\tY: ");
  Serial.print(rc_y_cons);
  if (new_line) Serial.println();
}

// Stick pattern used for initial calibration loops
bool check_loops() {
  if (rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }
  if (tl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 1800 && rc_y_raw < 2200) {
    tr1 = true;
    tl1 = false;
  }
  if (tr1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 1800 && rc_y_raw < 2200) {
    br1 = true;
  }
  if (br1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    bl1 = true;
  }
  if (bl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }

  if (rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800 && rc_r_raw < 1200) {
    tl2 = true;
  }
  if (tl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 1800 && rc_r_raw < 2200) {
    tr2 = true;
    tl2 = false;
  }
  if (tr2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 1800 && rc_r_raw < 2200) {
    br2 = true;
  }
  if (br2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 800 && rc_r_raw < 1200) {
    bl2 = true;
  }
  if (bl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800 && rc_r_raw < 1200) {
    tl2 = true;
  }

  if (tl1 && tr1 && br1 && bl1 && tl2 && tr2 && br2 && bl2) {
    tl1 = tr1 = br1 = bl1 = false;
    tl2 = tr2 = br2 = bl2 = false;
    return true;
  } else {
    return false;
  }
}

void controller_loop(float &rc_t, float &rc_p, float &rc_r, float &rc_y) {
  // RC failsafe: if no pulse for >100ms -> lost
  if (micros() - last_rc_update > 100000) {
    rc_signal_lost = true;
  }

  if (micros() - loop_timer < 10000) {
    return;
  }

  exec_time = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  if (!cal) {
    CalibrateChannels();

    if (check_loops()) {
      cal_loops++;
    }

    if (cal_loops >= cal_loops_req) {
      // Stick centered, record neutral positions
      if ((rc_t_raw > 1400 && rc_t_raw < 1600 && rc_y_raw > 1400 && rc_y_raw < 1600) &&
          (rc_p_raw > 1400 && rc_p_raw < 1600 && rc_r_raw > 1400 && rc_r_raw < 1600)) {
        center_delay++;
        Serial.print("Recording standby position. DO NOT MOVE the controller!  ");
        Serial.print("[ ");
        Serial.print(center_delay);
        Serial.print(" / ");
        Serial.print(center_delay_req);
        Serial.print(" ] - [ ");
        Serial.print(center_raw_cum);
        Serial.print(" / ");
        Serial.print(center_raw_cum_req);
        Serial.println(" ]");

        if (center_delay >= center_delay_req) {
          center_t_raw_cum += rc_t_raw;
          center_y_raw_cum += rc_y_raw;
          center_p_raw_cum += rc_p_raw;
          center_r_raw_cum += rc_r_raw;
          center_raw_cum++;

          if (center_raw_cum == center_raw_cum_req) {
            center_t_raw = center_t_raw_cum / center_raw_cum_req;
            center_y_raw = center_y_raw_cum / center_raw_cum_req;
            center_p_raw = center_p_raw_cum / center_raw_cum_req;
            center_r_raw = center_r_raw_cum / center_raw_cum_req;
            Serial.println("Standby position recorded");
            Serial.println("You can now move the controller");
            cal = true;
          }
        }
      } else {
        center_delay = 0;
        center_raw_cum = 0;
        center_t_raw_cum = 0;
        center_y_raw_cum = 0;
        center_p_raw_cum = 0;
        center_r_raw_cum = 0;
      }
    } else {
      PrintCalInfo(true);
    }

    if (cal) {
      Serial.println("Calibration complete");
      Serial.println("Starting controller loop");
    }

    return;
  }

  // Map to control ranges
  rc_t_cons = map(rc_t_raw, range_t_raw[0], range_t_raw[1], 0, 100);        // 0..100 %
  rc_p_cons = map(rc_p_raw, range_p_raw[0], range_p_raw[1], -30, 30);       // deg
  rc_r_cons = map(rc_r_raw, range_r_raw[0], range_r_raw[1], -30, 30);       // deg
  rc_y_cons = map(rc_y_raw, range_y_raw[0], range_y_raw[1], -150, 150);     // yaw rate deg/s

  rc_t = constrain(rc_t_cons, 0, 100);
  rc_p = rc_p_cons;
  rc_r = rc_r_cons;
  rc_y = rc_y_cons;

  // Uncomment for debugging
  PrintConsignment(true);
}

// Public helpers
bool controller_is_calibrated() {
  return cal;
}

bool controller_signal_ok() {
  return !rc_signal_lost;
}
