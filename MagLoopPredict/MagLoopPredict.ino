/*
 * Modified for Kenwood TS-590 radio.
 * 
 * Built based on the original Elecraft sample code.
 * Adjustments:
 * - Updated CAT commands for TS-590.
 * - Ensured compatibility with TS-590's response format for frequency queries.
 * - Default baud rate set to 57600, commonly used with Kenwood radios.
 * 
 * Pre-requisites:
 * - TMC2130 Stepper Driver:
 *    - Ensure proper jumper settings for your TMC2130 driver:
 *      - Right pads soldered for upper jumpers.
 *      - Left pads soldered for lower jumpers.
 * - NEMA17 Stepper Motor (wire length: 1m, adjust STALL_VALUE for different lengths).
 * - Arduino Uno (or compatible clone).
 * - Match CAT settings on the TS-590 with this script (baud rate, etc.).
 *
 * Configuration:
 * - Homing is configured to rotate counter-clockwise. If this doesn't work, reverse stepper wiring.
 * - CAT communication with TS-590 is via `FA;` command.
 */

#define STALL_VALUE      15 // [-64..63] - lower for higher sensitivity, higher for lower sensitivity

#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Pin Definitions
#define EN_PIN           7   // Stepper Driver Enable Pin
#define DIR_PIN          9   // Stepper Driver Direction Pin
#define STEP_PIN         8   // Stepper Driver Step Pin
#define CS_PIN           10  // TMC2130 Chip Select
#define BTN              19  // Button for calibration (A5 pin)

// Maximum number of calibration points per band
#define MAX_DATA_POINTS_PER_BAND 20

// Stepper Speed and Acceleration Configuration
#define MAX_SPEED        40  // Maximum stepper speed
#define MIN_SPEED      1000  // Minimum stepper speed
#define R_SENSE 0.11f        // R Sense resistor value (check your TMC2130 documentation)

// EEPROM Data Structure
#define MAGIC 0x4444         // Magic number to identify valid EEPROM data
#define MAX_INT 32767        // Maximum integer value (used for default position)

struct DataPoint {
  int pos;   // Stepper position
  long freq; // Frequency in Hz
};

// Driver, Stepper, and Software Serial Objects
TMC2130Stepper driver(CS_PIN, R_SENSE);    // TMC2130 stepper driver
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN); // Stepper motor
SoftwareSerial softSerial(4, 5, true);     // RX, TX, invert voltage for CAT communication

using namespace TMC2130_n; // Namespace for TMC2130 functions

// Global Variables
bool calibration = false;  // Calibration mode flag
int calibPoints = 0;       // Number of calibration points
DataPoint *calib;          // Array to store calibration points

void setup() {
  Serial.begin(57600);         // Initialize serial port for monitoring
  while (!Serial);             // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(BTN, INPUT_PULLUP);  // Button for calibration (active LOW)
  delay(1);
  bool btnState = digitalRead(BTN);
  calibration = (btnState == LOW); // Enter calibration mode if button is pressed at startup

  if (!calibration) {
    // Load calibration data from EEPROM
    Serial.println("Loading calibration data...");
    int magic;
    EEPROM.get(0, magic);
    if (magic == MAGIC) {
      EEPROM.get(2, calibPoints);
      calib = (DataPoint *)malloc(sizeof(DataPoint) * calibPoints);
      for (int i = 0; i < calibPoints; i++) {
        EEPROM.get(4 + sizeof(DataPoint) * i, calib[i]);
        Serial.print(calib[i].freq);
        Serial.print(':');
        Serial.print(calib[i].pos);
        Serial.print(';');
      }
      Serial.println();
    }
  }

  // Initialize stepper driver and motor
  stepper.setEnablePin(EN_PIN);
  SPI.begin();

  softSerial.begin(57600); // Set baud rate to match TS-590 CAT settings
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // Motor current in mA
  driver.microsteps(16);   // 16 microsteps per full step
  driver.TCOOLTHRS(0xFFFFF); // CoolStep threshold
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);   // StallGuard sensitivity
  driver.diag1_stall(true);  // Enable stall detection
  driver.diag1_pushpull(true);

  // Configure stepper speed and acceleration
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(200 * 120);
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(true, false, true);
  stepper.enableOutputs();
  delay(1);
  stepper.disableOutputs();
  delay(1);

  // Home the stepper motor
  home();
}

// Home the stepper motor (find the zero position)
void home() {
  Serial.println("Homing...");
  stepper.moveTo(-16);
  stepper.runToPosition();
  delay(10);

  bool foundLimit = false;
  stepper.enableOutputs();
  stepper.moveTo(-200 * 16 * 4); // Move to an estimated limit

  while (stepper.isRunning()) {
    for (int i = 0; i < 16 * 3; i++) {
      stepper.run();
    }

    if (!foundLimit && (stepper.currentPosition() < 500) && driver.stallguard()) {
      foundLimit = true;
      stepper.stop();
      Serial.println("Limit found.");
    }
  }
  stepper.disableOutputs();
  stepper.setCurrentPosition(0);
}

long lastFreq = 0;

// Main loop
void loop() {
  if (calibration) {
    calibrate();
  }

  // Require at least two calibration points
  if (calibPoints < 2) {
    delay(1000);
    return;
  }

  long freq = readFreq(); // Query frequency from the TS-590
  if (freq == lastFreq) {
    delay(500);
    return;
  }
  lastFreq = freq;

  // Find the two closest calibration points
  DataPoint foundLower = findLowerDataPoint(freq);
  DataPoint foundHigher = findHigherDataPoint(freq);

  if (foundLower.pos > 0 && foundHigher.pos > 0) {
    long dpBandwidth = foundHigher.freq - foundLower.freq;
    int steps = foundHigher.pos - foundLower.pos;
    double proportional = (double)dpBandwidth / steps;
    int newPos = foundLower.pos + (freq - foundLower.freq) / proportional;

    moveTo(newPos); // Move to the calculated position
  }
  delay(500);
}

// Query the TS-590 for the current frequency
long readFreq() {
  softSerial.print("FA;");
  softSerial.flush();
  String response = softSerial.readStringUntil(';');

  if (response.startsWith("FA")) {
    response = response.substring(2); // Remove "FA" prefix
    long freq = response.toInt();
    Serial.print("Freq: ");
    Serial.println(freq);
    return freq;
  } else {
    Serial.println("Invalid CAT Response");
    return 0;
  }
}

// Find the closest calibration point below the current frequency
DataPoint findLowerDataPoint(long freq) {
  DataPoint foundDp = {0, 0};
  for (int i = 0; i < calibPoints; i++) {
    DataPoint dp = calib[i];
    if (dp.freq <= freq && (dp.freq + 500000L > freq) && dp.pos > foundDp.pos) {
      foundDp = dp;
    }
  }
  return foundDp;
}

// Find the closest calibration point above the current frequency
DataPoint findHigherDataPoint(long freq) {
  DataPoint foundDp = {MAX_INT, 0};
  for (int i = 0; i < calibPoints; i++) {
    DataPoint dp = calib[i];
    if ((dp.freq > freq) && ((dp.freq - 500000L) < freq) && (dp.pos < foundDp.pos)) {
      foundDp = dp;
    }
  }
  if (foundDp.pos == MAX_INT) foundDp.pos = 0;
  return foundDp;
}

// Move the stepper motor to a specific position
void moveTo(int pos) {
  stepper.enableOutputs();
  stepper.moveTo(pos);
  stepper.runToPosition();
  stepper.disableOutputs();
}

// Calibration routine
void calibrate() {
  Serial.println("Calibration Mode...");
  int dp = 0;
  DataPoint dataPoints[MAX_DATA_POINTS_PER_BAND];

  for (int i = 0;; i += 50) {
    int pos = i;
