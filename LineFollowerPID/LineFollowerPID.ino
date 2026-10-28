#include <EEPROM.h>

const uint8_t PIN_PWMA = 5;
const uint8_t PIN_AIN1 = 3;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 6;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 9;
const uint8_t PIN_STBY = 8;

const uint8_t PIN_LED = 13;
const uint8_t PIN_BUTTON = 10; // Calibration and Start button

const uint8_t SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};
const int SENSOR_WEIGHTS[7]  = {-3000, -2000, -1000, 0, 1000, 2000, 3000};

const bool BLACK_LINE = true;

// Dynamic thresholds learned during 360 calibration
const uint16_t EEPROM_MAGIC = 0x5446;
const int EEPROM_ADDR = 0;
struct CalibData {
  uint16_t magic;
  int minVal[7];
  int maxVal[7];
  int threshold[7];
};
CalibData calib;
int sensorThreshold[7] = {0};

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

const int BASE_SPEED   = 145; // Increased from 110
const int MAX_SPEED    = 210; // Increased from 170
const int TURN_SPEED   = 125; // Increased from 105
const int INCH_TIME    = 110; // Scaled for proper axle placement at 145 speed

float Kp = 0.065;
float Ki = 0.0001;
float Kd = 0.35;

const unsigned long MIN_JUNCTION_DEBOUNCE = 15; // Minimal 20ms debounce for tightly spaced junctions
bool junctionArmed = true;
uint8_t junctionClearCounter = 0;

int lastError = 0;
float integral = 0;
unsigned long lastJunctionTime = 0;

bool isOnLine(int raw, uint8_t idx) {
  return BLACK_LINE ? (raw > sensorThreshold[idx]) : (raw < sensorThreshold[idx]);
}

bool isSensorOnLine(uint8_t idx) {
  return isOnLine(analogRead(SENSOR_PINS[idx]), idx);
}

void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, leftSpeed, INVERT_LEFT_MOTOR);
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, rightSpeed, INVERT_RIGHT_MOTOR);
}

void pauseAndBlink() {
  setMotors(0, 0);
  for (int i = 0; i < 4; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(250);
    digitalWrite(PIN_LED, LOW);
    delay(250);
  }
}

void pivot(bool turnToLeft) {
  int leftSpeed  = turnToLeft ? -TURN_SPEED : TURN_SPEED;
  int rightSpeed = turnToLeft ? TURN_SPEED  : -TURN_SPEED;

  setMotors(leftSpeed, rightSpeed);
  // 1. Mandatory blind rotation to clear current line and cross-tape
  delay(160);

  // 2. Rotate until center sensor (S4 = index 3) locks onto the new line
  unsigned long t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 600)) {
  }

  // 3. Active counter-torque braking to snap accurately onto heading
  setMotors(-leftSpeed / 2, -rightSpeed / 2);
  delay(25);
  setMotors(0, 0);
  delay(20);

  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
}

void turnAround() {
  setMotors(-TURN_SPEED, TURN_SPEED);
  delay(220); // Clear current line

  unsigned long t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 1500)) {
  }

  setMotors(TURN_SPEED / 2, -TURN_SPEED / 2);
  delay(25);
  setMotors(0, 0);
  delay(20);

  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
}

int readSensors(bool &hasLeft, bool &hasRight, bool &hasCross, bool &allWhite) {
  long weightedSum = 0;
  int totalActive = 0;
  int activeCount = 0;
  bool s[7];

  for (int i = 0; i < 7; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    s[i] = isOnLine(raw, i);
    if (s[i]) {
      int activeVal = BLACK_LINE ? (raw - sensorThreshold[i]) : (sensorThreshold[i] - raw);
      weightedSum += (long)activeVal * SENSOR_WEIGHTS[i];
      totalActive += activeVal;
      activeCount++;
    }
  }

  allWhite = (totalActive == 0);

  bool centerOnLine = (s[2] || s[3] || s[4]);

  hasCross = ((s[0] || s[1]) && (s[5] || s[6]) && centerOnLine) || (activeCount >= 5);
  hasLeft  = (s[0] || s[1]) && centerOnLine && !hasCross;
  hasRight = (s[5] || s[6]) && centerOnLine && !hasCross;

  if (totalActive > 0) {
    return (int)(weightedSum / totalActive);
  }

  return 0;
}

void waitForButton() {
  // Blink LED slowly (2 Hz) while waiting for user press
  while (digitalRead(PIN_BUTTON) == HIGH) {
    digitalWrite(PIN_LED, (millis() / 250) % 2);
    delay(10);
  }
  // Wait until user releases button
  while (digitalRead(PIN_BUTTON) == LOW) {
    delay(10);
  }
  delay(150); // Contact debounce
  digitalWrite(PIN_LED, LOW);
}

void calibrateSensors() {
  int minVal[7], maxVal[7];
  for (int i = 0; i < 7; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  // 360-degree rotation in place for 4 seconds
  setMotors(-TURN_SPEED, TURN_SPEED);
  unsigned long tStart = millis();

  while (millis() - tStart < 4000) {
    for (int i = 0; i < 7; i++) {
      int v = analogRead(SENSOR_PINS[i]);
      if (v < minVal[i]) minVal[i] = v;
      if (v > maxVal[i]) maxVal[i] = v;
    }
    // Fast strobe LED while calibrating
    digitalWrite(PIN_LED, (millis() / 70) % 2);
    delay(2);
  }

  setMotors(0, 0);

  // Compute individual sensor thresholds dynamically from the 360 sweep
  calib.magic = EEPROM_MAGIC;
  for (int i = 0; i < 7; i++) {
    calib.minVal[i] = minVal[i];
    calib.maxVal[i] = maxVal[i];
    calib.threshold[i] = (minVal[i] + maxVal[i]) / 2;
    sensorThreshold[i] = calib.threshold[i];
  }
  EEPROM.put(EEPROM_ADDR, calib);

  digitalWrite(PIN_LED, LOW);
  delay(300);
}

void setup() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_LED, LOW);

  digitalWrite(PIN_STBY, HIGH);
  Serial.begin(9600);

  // Load previous calibration from EEPROM if available
  EEPROM.get(EEPROM_ADDR, calib);
  if (calib.magic == EEPROM_MAGIC) {
    for (int i = 0; i < 7; i++) {
      sensorThreshold[i] = calib.threshold[i];
    }
  }

  // 1. First button click on D10 -> Start 360-degree 4-second calibration
  waitForButton();
  delay(300);
  calibrateSensors();

  // 2. Second button click on D10 -> Start running
  waitForButton();

  // 3-flash countdown to let user release hands
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }

  lastJunctionTime = millis();
  junctionArmed = true;
}

void loop() {
  bool hasLeft  = false;
  bool hasRight = false;
  bool hasCross = false;
  bool allWhite = false;

  int error = readSensors(hasLeft, hasRight, hasCross, allWhite);

  // Spatial re-arming: re-arm as soon as outer sensors leave the intersection branch
  if (!junctionArmed) {
    bool outerClear = !isSensorOnLine(0) && !isSensorOnLine(1) && 
                      !isSensorOnLine(5) && !isSensorOnLine(6);
    if (outerClear && (millis() - lastJunctionTime > MIN_JUNCTION_DEBOUNCE)) {
      junctionClearCounter++;
      if (junctionClearCounter >= 1) { // 1 clean sample is enough to re-arm
        junctionArmed = true;
        junctionClearCounter = 0;
      }
    } else {
      junctionClearCounter = 0;
    }
  }

  if ((hasLeft || hasRight || hasCross) && junctionArmed) {
    bool confL = false, confR = false, confCross = false, confWhite = false;
    readSensors(confL, confR, confCross, confWhite);

    if (hasLeft || hasRight || hasCross || confL || confR || confCross) {
      junctionArmed = false;
      junctionClearCounter = 0;
      lastJunctionTime = millis();

      digitalWrite(PIN_LED, HIGH);

      // Track branches observed before and DURING the inch
      bool seenRight = hasRight || hasCross || confR || confCross || isSensorOnLine(5) || isSensorOnLine(6);
      bool seenLeft  = hasLeft  || hasCross || confL || confCross || isSensorOnLine(0) || isSensorOnLine(1);

      // Smooth inching to align drive axle over junction node while actively sampling wings
      setMotors(BASE_SPEED, BASE_SPEED);
      unsigned long tInch = millis();
      while (millis() - tInch < INCH_TIME) {
        if (isSensorOnLine(5) || isSensorOnLine(6)) seenRight = true;
        if (isSensorOnLine(0) || isSensorOnLine(1)) seenLeft  = true;
      }

      bool straightAhead = isSensorOnLine(2) || isSensorOnLine(3) || isSensorOnLine(4);

      // STRICT PRIORITY: RIGHT -> STRAIGHT -> LEFT -> U-TURN
      if (seenRight) {
        pivot(false); // Turn Right (Priority 1)
      } else if (straightAhead) {
        setMotors(BASE_SPEED, BASE_SPEED); // Go Straight (Priority 2)
        delay(80);
        integral = 0;
        lastError = 0;
        lastJunctionTime = millis();
        junctionArmed = false;
        junctionClearCounter = 0;
      } else if (seenLeft) {
        pivot(true);  // Turn Left (Priority 3)
      } else {
        turnAround(); // U-Turn (Priority 4)
      }
      digitalWrite(PIN_LED, LOW);
      return;
    }
  }

  if (allWhite) {
    delay(30);
    bool confL = false, confR = false, confCross = false, confWhite = false;
    readSensors(confL, confR, confCross, confWhite);

    if (confWhite) {
      turnAround();
      return;
    }
  }

  integral = constrain(integral + error, -2000, 2000);
  float derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * integral + Kd * derivative);

  int leftSpeed  = constrain(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
  lastError = error;
}
