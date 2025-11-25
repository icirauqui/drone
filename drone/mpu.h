#pragma once

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"

const int GYRO_CONFIG_REGISTER = 0x1B;
const int ACCEL_CONFIG_REGISTER = 0x1C;
const float GYRO_SCALE_FACTOR = 65.5;    // 500 dps
const float ACCEL_SCALE_FACTOR = 4096;   // +/- 8g
//const float RAD_TO_DEG = 57.2957795;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MPU6050_adress 0x68
MPU6050 mpu;

int16_t gx = 0, gy = 0, gz = 0;
int64_t gx_cal = 0, gy_cal = 0, gz_cal = 0;
float wx = 0.0f, wy = 0.0f, wz = 0.0f;

float a_xyz = 0.0f;
int16_t ax = 0, ay = 0, az = 0;
int64_t ax_cal = 0, ay_cal = 0, az_cal = 0;

float ang_p = 0.0f, ang_r = 0.0f;
float ang_p_a = 0.0f, ang_r_a = 0.0f;

bool set_gyro_ang = false;
bool acc_calib_ok = false;

float timer_run = 0.0f;
unsigned long mpu_timer_loop = 0;

// Config
#define OUTPUT_READABLE_ACCELGYRO
#define MPU_CYCLE 5000UL  // 5 ms

void mpu_init() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);
  Wire.write(0x08); // 500 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);
  Wire.write(0x10); // +/- 8g
  Wire.endTransmission();
}

void mpu_calib(int calib_loops = 3000, int delay_us = 100) {
  ax_cal = ay_cal = az_cal = 0;
  gx_cal = gy_cal = gz_cal = 0;

  for (int i = 0; i < calib_loops; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_cal += ax;
    ay_cal += ay;
    az_cal += az;
    gx_cal += gx;
    gy_cal += gy;
    gz_cal += gz;
    delayMicroseconds(delay_us);
  }

  ax_cal /= calib_loops;
  ay_cal /= calib_loops;
  az_cal /= calib_loops;
  gx_cal /= calib_loops;
  gy_cal /= calib_loops;
  gz_cal /= calib_loops;

  mpu_timer_loop = micros();
  acc_calib_ok = true;
}

void mpu_process() {
  ax -= ax_cal;
  ay -= ay_cal;
  az -= az_cal;
  az += 4096;

  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;

  wx = gx / 65.5f;
  wy = gy / 65.5f;
  wz = gz / 65.5f;

  // Integrate gyro angles (deg)
  ang_p += wx * timer_run / 1000.0f;
  ang_r += wy * timer_run / 1000.0f;

  // Simple gyro cross-coupling compensation
  ang_p += (ang_r * sin((gz - gz_cal) * timer_run * 0.000000266f));
  ang_r -= (ang_p * sin((gz - gz_cal) * timer_run * 0.000000266f));

  // Accel angles
  a_xyz = sqrt((float)ax * ax + (float)ay * ay + (float)az * az);
  ang_p_a = asin((float)ay / a_xyz) * 57.2957795f;
  ang_r_a = asin((float)ax / a_xyz) * -57.2957795f;

  // Complementary filter
  if (set_gyro_ang) {
    ang_p = ang_p * 0.99f + ang_p_a * 0.01f;
    ang_r = ang_r * 0.99f + ang_r_a * 0.01f;
  } else {
    ang_p = ang_p_a;
    ang_r = ang_r_a;
    set_gyro_ang = true;
  }
}

char mpu_read(bool verbose = false) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (verbose) {
    Serial.print("\ta/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz);
  }

  char ag = (uint8_t)(ax >> 8) + (uint8_t)(ax & 0xFF)
          + (uint8_t)(ay >> 8) + (uint8_t)(ay & 0xFF)
          + (uint8_t)(az >> 8) + (uint8_t)(az & 0xFF)
          + (uint8_t)(gx >> 8) + (uint8_t)(gx & 0xFF)
          + (uint8_t)(gy >> 8) + (uint8_t)(gy & 0xFF)
          + (uint8_t)(gz >> 8) + (uint8_t)(gz & 0xFF);

  return ag;
}

void mpu_setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.print("Initializing MPU6050... ");
  mpu.initialize();
  Serial.println("Done");

  Serial.print("Testing device connection... ");
  Serial.println(mpu.testConnection() ? "OK" : "Fail");

  Serial.print("Setting default params... ");
  mpu_init();
  Serial.println("Done");

  Serial.print("Calibrating MPU6050 (keep drone still)... ");
  mpu_calib(3000, 100);
  Serial.println("Done");
}

void mpu_loop(bool verbose = false) {
  while (micros() - mpu_timer_loop < MPU_CYCLE) {
    // wait for next cycle
  }

  timer_run = (micros() - mpu_timer_loop) / 1000.0f;
  mpu_timer_loop = micros();

  char ag = mpu_read(false);
  (void)ag; // unused checksum

  mpu_process();

  if (verbose) {
    Serial.print("  - Angle {Pitch, Roll} = {");
    Serial.print(ang_p);
    Serial.print(", ");
    Serial.print(ang_r);
    Serial.println("} deg");
  }
}
