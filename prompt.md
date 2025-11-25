I’m building a racing drone using an Arduino MKR WiFi 1010.

Below is my current codebase. I need you to:

1. Review and refactor the existing code for reliability and flight-control stability.
2. Complete the missing logic needed to have a functional flight stack.
3. Create or modify any files as needed (modules for sensors, PID control, motor mixing, ESC output, safety checks, etc.).
4. Output the final code directly in the chat, file by file, ready to upload.

Current hardware setup:
- Arduino MKR WiFi 1010
- IMU accelerometer (mpu6050)
- Small OLED display
- Four motors already wired to the appropriate ESC pins
- A radio controller already paired and providing input signals

Assumptions:
- Use clean, modular structure (e.g., imu.h/cpp, pid.h/cpp, motor_controller.h/cpp, main.ino).
- Implement a basic but stable attitude-control loop (read IMU → apply complementary filter → PID → motor mix → ESC write).
- Include failsafes (signal loss, disarm state, throttle safety).
- Include simple on-screen status info on the display.

Goal:
Deliver the complete, ready-to-flash project that allows the drone to:
- Arm/disarm safely
- Hover and stabilize
- Respond to controller pitch/roll/yaw/throttle inputs


Code below:

drone.ino:

```
// Drone control

//#include "mpu.h"
#include "controller.h"
//#include "battery.h"
#include "motors.h"
#include "display.h"

float rc_t = 0.0;
float rc_p = 0.0;
float rc_r = 0.0;
float rc_y = 0.0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while(!Serial);

  display_setup();

  //mpu_setup();
  controller_setup();
  motors_setup();
}

void loop() {
  //mpu_loop(false);
  controller_loop(rc_t, rc_p, rc_r, rc_y);
  motors_loop(rc_t, rc_p, rc_r, rc_y);

  //Serial.print("  - Angle {Pitch, Roll} = {");
  //Serial.print(ang_p);
  //Serial.print(", ");
  //Serial.print(ang_r);
  //Serial.println("} deg");

  //delay(1000);
}
```

battery.h:
```
#define BAT_CYCLE 50000  // Time interval in microseconds between battery checks
#define REFERENCE_VOLTAGE 5.0  // Default reference voltage
#define VOLTAGE_DIVIDER_RATIO 2.5  // Ratio defined by the voltage divider circuit
#define ADC_RESOLUTION 1023.0  // Resolution of ADC in Arduino

long timer_loop;  // Stores the last time the battery was checked
float bat_v, bat_read = 0.0;  // Variables to store battery read and voltage

float bat_offset = 4.8;  // Offset to calibrate voltage reading

void bat_setup() {
  Serial.begin(115200);  // Begin serial communication at 115200 baud
}

void bat_loop() {
  // Check if enough time has passed since the last battery check
  if (micros() - timer_loop >= BAT_CYCLE) {
    timer_loop = micros();  // Update last checked time

    // Read battery voltage from pin A6
    bat_read = analogRead(A6);
    // Calculate actual battery voltage
    bat_v = VOLTAGE_DIVIDER_RATIO * (bat_read * bat_offset / ADC_RESOLUTION) * REFERENCE_VOLTAGE;

    // Output the battery voltage to the serial monitor
    Serial.print("Battery Voltage: ");
    Serial.println(bat_v);
  }
}
```

controller.h:
```
/* 
 *  
 *
 * 
 * Roll -> R/r
 * Pitch -> P/p
 * Throttle -> T/t
 * Yaw -> Y/y
 * 
 * 
 */

#define PIN_R 6
#define PIN_P 7
#define PIN_T 8
#define PIN_Y 9


long loop_timer, exec_time;

// Consignment values
float rc_t_cons, rc_p_cons, rc_r_cons, rc_y_cons;

bool cal = false;
int cal_loops = 0;
int cal_loops_req = 3;
int center_delay = 0;
int center_delay_req = 200;
int center_raw_cum = 0;
int center_raw_cum_req = 200;

bool tl1 = false;
bool tr1 = false;
bool bl1 = false;
bool br1 = false;

bool tl2 = false;
bool tr2 = false;
bool bl2 = false;
bool br2 = false;





// INTERRUPT ROLL
volatile long r_high_us;
volatile int rc_r_raw;
int range_r_raw[2] = {1500, 1500};
int center_r_raw = 1500;
int center_r_raw_cum = 0;
void IntR() {
  if (digitalRead(PIN_R) == HIGH) r_high_us = micros();
  if (digitalRead(PIN_R) == LOW)  rc_r_raw  = micros() - r_high_us;
}

// INTERRUPT PITCH
volatile long p_high_us;
volatile int rc_p_raw;
int range_p_raw[2] = {1500, 1500};
int center_p_raw = 1500;
int center_p_raw_cum = 0;
void IntP() {
  if (digitalRead(PIN_P) == HIGH) p_high_us = micros();
  if (digitalRead(PIN_P) == LOW)  rc_p_raw  = micros() - p_high_us;
}

// INTERRUPT THROTTLE
volatile long t_high_us;
volatile int rc_t_raw;
int range_t_raw[2] = {1500, 1500};
int center_t_raw = 1500;
int center_t_raw_cum = 0;
void IntT() {
  if (digitalRead(PIN_T) == HIGH) t_high_us = micros();
  if (digitalRead(PIN_T) == LOW)  rc_t_raw  = micros() - t_high_us;
}

// INTERRUPT YAW
volatile long y_high_us;
volatile int rc_y_raw;
int range_y_raw[2] = {1500, 1500};
int center_y_raw = 1500;
int center_y_raw_cum = 0;
void IntY() {
  if (digitalRead(PIN_Y) == HIGH) y_high_us = micros();
  if (digitalRead(PIN_Y) == LOW)  rc_y_raw  = micros() - y_high_us;
}



void controller_setup() {
  pinMode(PIN_R, INPUT_PULLUP);
  attachInterrupt(PIN_R, IntR, CHANGE);
  
  pinMode(PIN_P, INPUT_PULLUP);
  attachInterrupt(PIN_P, IntP, CHANGE);
  
  pinMode(PIN_T, INPUT_PULLUP);
  attachInterrupt(PIN_T, IntT, CHANGE);
  
  pinMode(PIN_Y, INPUT_PULLUP);
  attachInterrupt(PIN_Y, IntY, CHANGE);

  Serial.println("Controller setup complete");
  cal_loops = 0;
  cal = false;


  Serial.begin(115200);
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
  // Monitor Serie
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

  Serial.print(tl1);
  Serial.print(" ");
  Serial.print(tr1);
  Serial.print(" ");
  Serial.print(br1);
  Serial.print(" ");
  Serial.print(bl1);
  Serial.print(" ");

  Serial.print(tl2);
  Serial.print(" ");
  Serial.print(tr2);
  Serial.print(" ");
  Serial.print(br2);
  Serial.print(" ");
  Serial.print(bl2);
  Serial.print(" ");

  
  if (new_line) {
    Serial.println();
  }

}

void PrintConsignment(bool new_line) {
  Serial.print("R: ");
  Serial.print(rc_r_cons);
  Serial.print("\t");
  Serial.print("P: ");
  Serial.print(rc_p_cons);
  Serial.print("\t");
  Serial.print("T: ");
  Serial.print(rc_t_cons);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(rc_y_cons);
  Serial.print("\t");
  if (new_line) {
    Serial.println();
  }
}



bool check_loops() {
  if (rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }
  if (tl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 1800  && rc_y_raw < 2200) {
    tr1 = true;
    tl1 = false;
  }
  if (tr1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 1800  && rc_y_raw < 2200) {
    br1 = true;
  } 
  if (br1 && rc_t_raw > 800 && rc_t_raw < 1200 && rc_y_raw > 800  && rc_y_raw < 1200) {
    bl1 = true;
  } 
  if (bl1 && rc_t_raw > 1800 && rc_t_raw < 2200 && rc_y_raw > 800 && rc_y_raw < 1200) {
    tl1 = true;
  }

  if (rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    tl2 = true;
  }
  if (tl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 1800  && rc_r_raw < 2200) {
    tr2 = true;
    tl2 = false;
  }
  if (tr2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 1800  && rc_r_raw < 2200) {
    br2 = true;
  } 
  if (br2 && rc_p_raw > 800 && rc_p_raw < 1200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    bl2 = true;
  } 
  if (bl2 && rc_p_raw > 1800 && rc_p_raw < 2200 && rc_r_raw > 800  && rc_r_raw < 1200) {
    tl2 = true;
  }

  if (tl1 && tr1 && br1 && bl1 && tl2 && tr2 && br2 && bl2) {
    tl1 = false;
    tr1 = false;
    br1 = false;
    bl1 = false;
    tl2 = false;
    tr2 = false;
    br2 = false;
    bl2 = false;
    return true;
  } else {
    return false;
  }

}



void controller_loop(float &rc_t, float &rc_p, float &rc_r, float &rc_y) {

  if (micros() - loop_timer < 10000) {
    return;
  }

  exec_time = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  if (cal == false) {
    CalibrateChannels();

    if (check_loops()) {
      cal_loops++;
    }

    if (cal_loops >= cal_loops_req) {

      // Set center
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
      }
      else {
        center_delay = 0;
        center_raw_cum = 0;
        center_t_raw_cum = 0;
        center_y_raw_cum = 0;
        center_p_raw_cum = 0;
        center_r_raw_cum = 0;
      }
    } 
    else {
      PrintCalInfo(true);
    }

    if (cal) {
      Serial.println("Calibration complete");
      Serial.println("Starting controller loop");
    }
  
    return;
  } 
  

  rc_t_cons = map(rc_t_raw, range_t_raw[0], range_t_raw[1], 0, 100);
  rc_p_cons = map(rc_p_raw, range_p_raw[0], range_p_raw[1], -30, 30);
  rc_r_cons = map(rc_r_raw, range_r_raw[0], range_r_raw[1], -30, 30);
  rc_y_cons = map(rc_y_raw, range_y_raw[0], range_y_raw[1], -30, 30);

  
  rc_t = rc_t_cons;
  rc_p = rc_p_cons;
  rc_r = rc_r_cons;
  rc_y = rc_y_cons;
  
  PrintConsignment(true);

}
```

display.h:
```
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}


void display_setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  testdrawline();      // Draw many lines

  testdrawrect();      // Draw rectangles (outlines)

  testfillrect();      // Draw rectangles (filled)

  testdrawcircle();    // Draw circles (outlines)

  testfillcircle();    // Draw circles (filled)

  testdrawroundrect(); // Draw rounded rectangles (outlines)

  testfillroundrect(); // Draw rounded rectangles (filled)

  testdrawtriangle();  // Draw triangles (outlines)

  testfilltriangle();  // Draw triangles (filled)

  testdrawchar();      // Draw characters of the default font

  testdrawstyles();    // Draw 'stylized' characters

  testscrolltext();    // Draw scrolling text

  testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);

  testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
}

void loop_display() {
}
```

motors.h:
```
const int MIN_PWM = 1000;
const int MAX_PWM = 2000;

#include <Servo.h>

Servo esc1, esc2, esc3, esc4; // Create servo objects for each ESC
int escPins[4] = {2, 3, 4, 5}; // PWM pins connected to the ESCs


void motors_setup() {
  esc1.attach(escPins[0]);
  esc2.attach(escPins[1]);
  esc3.attach(escPins[2]);
  esc4.attach(escPins[3]);
}

void motors_loop(float &throttle, float &pitch, float &roll, float &yaw) {

  // Map throttle from 0-100 to 1000-2000 microseconds
  float baseThrottle = map(throttle, 50, 100, MIN_PWM, MAX_PWM);

  // Calculate individual motor speeds based on yaw, pitch, and roll
  float m1Speed = baseThrottle + pitch + roll + yaw;
  float m2Speed = baseThrottle + pitch - roll - yaw;
  float m3Speed = baseThrottle - pitch + roll - yaw;
  float m4Speed = baseThrottle - pitch - roll + yaw;

  // Constrain motor speeds to stay within the ESC signal range
  m1Speed = constrain(m1Speed, MIN_PWM, MAX_PWM);
  m2Speed = constrain(m2Speed, MIN_PWM, MAX_PWM);
  m3Speed = constrain(m3Speed, MIN_PWM, MAX_PWM);
  m4Speed = constrain(m4Speed, MIN_PWM, MAX_PWM);

  // Send signals to ESCs
  esc1.writeMicroseconds((int)m1Speed);
  esc2.writeMicroseconds((int)m2Speed);
  esc3.writeMicroseconds((int)m3Speed);
  esc4.writeMicroseconds((int)m4Speed);
}
```

mpu.h:
```
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

const int GYRO_CONFIG_REGISTER = 0x1B;
const int ACCEL_CONFIG_REGISTER = 0x1C;
const float GYRO_SCALE_FACTOR = 65.5;  // Scale factor for 500 dps
const float ACCEL_SCALE_FACTOR = 4096;  // Scale factor for +/- 8g
const float RAD_TO_DEG = 57.2957795;  // Conversion factor from radians to degrees


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
#define MPU6050_adress 0x68
MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t gx, gy, gz = 0;
int64_t gx_cal, gy_cal, gz_cal = 0;
float wx, wy, wz = 0.0;

float a_xyz = 0.0;
int16_t ax, ay, az = 0;
int64_t ax_cal, ay_cal, az_cal = 0;

float ang_p, ang_r = 0.0;
float ang_p_a, ang_r_a = 0.0;

bool set_gyro_ang, acc_calib_ok = false;

float timer_run, timer_loop = 0.0;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define MPU_CYCLE 5000




void mpu_init() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                          // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);                          // 00000000 para activar
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                          // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);                          // 00001000: 500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);                          // 00010000: +/- 8g
  Wire.endTransmission();
}



void mpu_print_calib() {
  Serial.print("  * ");
  Serial.print(ax_cal); Serial.print("\t");
  Serial.print(ay_cal); Serial.print("\t");
  Serial.print(az_cal); Serial.print("\t");
  Serial.print(gx_cal); Serial.print("\t");
  Serial.print(gy_cal); Serial.print("\t");
  Serial.print(gz_cal);
  Serial.println("");
}



void mpu_print() {
  Serial.print("  - ");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz);
  Serial.println("");
}


void mpu_calib(int calib_loops = 3000, int delay = 100) {
  for (unsigned int i=0; i < calib_loops; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_cal += ax;
    ay_cal += ay;
    az_cal += az;
    gx_cal += gx;
    gy_cal += gy;
    gz_cal += gz;
    delayMicroseconds(delay);
  }

  ax_cal /= calib_loops;
  ay_cal /= calib_loops;
  az_cal /= calib_loops;
  gx_cal /= calib_loops;
  gy_cal /= calib_loops;
  gz_cal /= calib_loops;

  timer_loop = micros();

  acc_calib_ok = true;
}



void mpu_print_ang_speed() {
  Serial.print("  - Angular speeds (deg/s): ");
  Serial.print(wx); Serial.print("\t");
  Serial.print(wy); Serial.print("\t");
  Serial.print(wz);
  Serial.println("");
}

void mpu_process() {

  //Serial.println("Process");
  //mpu_print();
  //mpu_print_calib();

  ax -= ax_cal;
  ay -= ay_cal;
  az -= az_cal;
  az += 4096;

  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;

  //mpu_print();

  wx = gx / 65.5;
  wy = gy / 65.5;
  wz = gz / 65.5;

  //mpu_print_ang_speed();

  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  ang_p += wx * timer_run / 1000.0;
  ang_r += wy * timer_run / 1000.0;

  ang_p += (ang_r * sin((gz - gz_cal) * timer_run * 0.000000266));
  ang_r -= (ang_p * sin((gz - gz_cal) * timer_run * 0.000000266));

  // Compute acceleration vector
  // Rad to Deg = * 180 / PI
  a_xyz = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  ang_p_a = asin((float)ay / a_xyz) * 57.2957795;
  ang_r_a = asin((float)ax / a_xyz) * -57.2957795;

  if (set_gyro_ang) {
    ang_p = ang_p * 0.99 + ang_p_a * 0.01;
    ang_r = ang_r * 0.99 + ang_r_a * 0.01;
  }
  else {
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
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  
  // initialize device
  Serial.print("Initializing MPU6050... ");
  mpu.initialize();
  Serial.println("Done");

  // verify connection
  Serial.print("Testing device connection... ");
  Serial.println(mpu.testConnection() ? "OK" : "Fail");

  // Load default params
  Serial.print("Setting default params... ");
  mpu_init();
  Serial.println("Done");

  // Calibrate
  Serial.print("Calibrating MPU6050... ");
  mpu_calib(3000, 100);
  Serial.println("Done");
}



void mpu_loop(bool verbose = false) {
  while (micros() - timer_loop < MPU_CYCLE) {
    // do nothing
  }

  timer_run = (micros() - timer_loop) / 1000.0;
  timer_loop = micros();

  char ag = mpu_read(false);

  mpu_process();

  if (verbose) {
    Serial.print("  - Angle {Pitch, Roll} = {");
    Serial.print(ang_p);
    Serial.print(", ");
    Serial.print(ang_r);
    Serial.println("} deg");
  }
}
```

pid.h:
```
#ifndef PID_H
#define PID_H

#include "Arduino.h"

struct PIDContext {
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float integral;
    float previous_error;
    float setpoint; // Desired value
    float output; // Output value
    unsigned long lastTime;
};

class PID {
    public:
        PIDContext pitch;
        PIDContext roll;
        PIDContext yaw;
        
        void pid_setup(float Kp, float Ki, float Kd) {
            // Initialize PID contexts for pitch, roll, and yaw
            initializePID(pitch, Kp, Ki, Kd);
            initializePID(roll, Kp, Ki, Kd);
            initializePID(yaw, Kp, Ki, Kd);
        }
        
        void pid_loop(float current_pitch, float current_roll, float current_yaw, float dt) {
            pitch.setpoint = current_pitch; // Assuming setpoint is updated elsewhere
            roll.setpoint = current_roll; // Assuming setpoint is updated elsewhere
            yaw.setpoint = current_yaw; // Assuming setpoint is updated elsewhere

            pitch.output = computePID(pitch, dt);
            roll.output = computePID(roll, dt);
            yaw.output = computePID(yaw, dt);
        }
    
    private:
        void initializePID(PIDContext &ctx, float Kp, float Ki, float Kd) {
            ctx.Kp = Kp;
            ctx.Ki = Ki;
            ctx.Kd = Kd;
            ctx.integral = 0.0;
            ctx.previous_error = 0.0;
            ctx.lastTime = millis();
        }
        
        float computePID(PIDContext &ctx, float dt) {
            float error = ctx.setpoint - ctx.output; // Calculate error
            ctx.integral += error * dt; // Update integral
            float derivative = (error - ctx.previous_error) / dt; // Calculate derivative
            ctx.previous_error = error; // Update error history

            // Calculate PID output
            return (ctx.Kp * error) + (ctx.Ki * ctx.integral) + (ctx.Kd * derivative);
        }
};

#endif
```

Review the code and complete the project.