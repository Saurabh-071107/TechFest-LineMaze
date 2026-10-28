#include <EEPROM.h>

const uint8_t PIN_PWMA = 5;
const uint8_t PIN_AIN1 = 3;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 6;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 9;
const uint8_t PIN_STBY = 8;

const uint8_t PIN_LED = 13;

// 6-reading data protection buffers
const uint8_t HISTORY_LEN = 6;
int errorHistory[HISTORY_LEN] = {0};
uint8_t errorHistCount = 0;
uint8_t errorHistIdx = 0;
uint8_t patternHistory[HISTORY_LEN] = {0};

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

float Kp = 0.048; // Smooth, precise proportional response without oscillation
float Ki = 0.00005; // Low integral to prevent slow hunting
float Kd = 0.22;  // Controlled damping without high-frequency derivative twitch

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

  // 1. Initial spin off current line: wait until middle sensor (S4) completely leaves old line
  delay(140);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 300)) {
  }

  // 2. Turn completely until the main line reaches the middle sensor (S4 = index 3)
  t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 1000)) {
  }

  // 3. Active counter-torque braking to snap accurately onto heading with middle sensor on line
  setMotors(-leftSpeed / 2, -rightSpeed / 2);
  delay(25);
  setMotors(0, 0);
  delay(20);

  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
  errorHistCount = 0;
  errorHistIdx = 0;
}

void turnAround() {
  // Spin 360 on the spot until middle sensor (S4 = index 3) gets the line
  setMotors(-TURN_SPEED, TURN_SPEED);

  // 1. Initial spin off current position: wait until middle sensor (S4) leaves current line/dead-end
  delay(140);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 300)) {
  }

  // 2. Spin 360 on the spot until middle sensor (S4 = index 3) gets the line
  t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 3500)) {
  }

  // 3. Counter-torque brake to lock heading with middle sensor on line
  setMotors(TURN_SPEED / 2, -TURN_SPEED / 2);
  delay(25);
  setMotors(0, 0);
  delay(20);

  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
  errorHistCount = 0;
  errorHistIdx = 0;
}

int readSensors(bool &hasLeft, bool &hasRight, bool &hasCross, bool &allWhite, uint8_t &outPattern) {
  long weightedSum = 0;
  int totalActive = 0;
  int activeCount = 0;
  uint8_t pattern = 0;

  for (int i = 0; i < 7; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    if (isOnLine(raw, i)) {
      pattern |= (1 << (6 - i)); // S1 is bit 6 (MSB), S7 is bit 0 (LSB)
      int activeVal = BLACK_LINE ? (raw - sensorThreshold[i]) : (sensorThreshold[i] - raw);
      weightedSum += (long)activeVal * SENSOR_WEIGHTS[i];
      totalActive += activeVal;
      activeCount++;
    }
  }

  outPattern = pattern;

  // --- D. Recovery and Unusual Patterns ---
  // Pattern 23: 0000000 - Possible line loss / dead end
  allWhite = (pattern == 0b0000000);

  // Pattern 24 (1000001) & Pattern 25 (1010101): Noise / unusual patterns
  if (pattern == 0b1000001 || pattern == 0b1010101) {
    hasLeft = false;
    hasRight = false;
    hasCross = false;
    return lastError; // Reject transient noise; hold previous steering
  }

  bool leftWing     = (pattern & 0b1100000) != 0; // S1 or S2
  bool rightWing    = (pattern & 0b0000011) != 0; // S6 or S7
  bool centerOnLine = (pattern & 0b0011100) != 0; // S3, S4, or S5

  // --- C. Junction and Intersection Candidates ---
  // Pattern 20: 1111111 (all sensors active)
  // Pattern 21: 1110111 (broad intersection)
  // Pattern 22: 1100110 (two separated black regions)
  // Pattern 18: 1111100 (broad left-side junction)
  // Pattern 19: 0011111 (broad right-side junction)
  // Pattern 12: 1111000 (wide left curve/branch)
  // Pattern 13: 0001111 (wide right curve/branch)
  // Pattern 16: 1101000 (irregular left bend)
  // Pattern 17: 0001011 (irregular right bend)
  hasCross = (pattern == 0b1111111) || (pattern == 0b1110111) || (pattern == 0b1100110) ||
             (leftWing && rightWing && centerOnLine) || (activeCount >= 5);

  hasRight = (pattern == 0b0011111) || (pattern == 0b0001111) || (pattern == 0b0001011) ||
             (pattern == 0b0000111) || (rightWing && centerOnLine && !hasCross);

  hasLeft  = (pattern == 0b1111100) || (pattern == 0b1111000) || (pattern == 0b1101000) ||
             (pattern == 0b1110000) || (leftWing && centerOnLine && !hasCross);

  // --- A & B. Normal Line Following & Continuous Tracking ---
  if (totalActive > 0) {
    int continuousError = (int)(weightedSum / totalActive);
    // Center deadband (±40 units) to keep the nose locked dead straight without twitching
    if (abs(continuousError) < 40) {
      continuousError = 0;
    }
    return continuousError;
  }

  return 0;
}

int readSensors(bool &hasLeft, bool &hasRight, bool &hasCross, bool &allWhite) {
  uint8_t dummyPattern = 0;
  return readSensors(hasLeft, hasRight, hasCross, allWhite, dummyPattern);
}

int applyDataProtection(int rawError, uint8_t rawPattern, bool &junctionSupported) {
  // 1. Calculate average of last 6 readings
  int sum = 0;
  uint8_t count = errorHistCount < HISTORY_LEN ? errorHistCount : HISTORY_LEN;
  for (uint8_t i = 0; i < count; i++) {
    sum += errorHistory[i];
  }
  int avgError = (count > 0) ? (sum / count) : rawError;

  // 2. Outlier Rejection: Compare current reading to the 6-sample moving average
  int protectedError = rawError;
  if (count >= 3) {
    int deviation = rawError - avgError;
    // Reject single-cycle optical anomalies
    if (deviation > 1800) {
      protectedError = avgError + 1200;
    } else if (deviation < -1800) {
      protectedError = avgError - 1200;
    }
  }

  // 3. Store into circular history buffer
  errorHistory[errorHistIdx] = protectedError;
  patternHistory[errorHistIdx] = rawPattern;
  errorHistIdx = (errorHistIdx + 1) % HISTORY_LEN;
  if (errorHistCount < HISTORY_LEN) errorHistCount++;

  // 4. Calculate smoothed error for jitter-free PID steering
  sum = 0;
  for (uint8_t i = 0; i < errorHistCount; i++) {
    sum += errorHistory[i];
  }
  int smoothedError = sum / errorHistCount;

  // 5. Junction Data Protection: check if recent history confirms junction candidate
  uint8_t junctionVotes = 0;
  for (uint8_t i = 0; i < errorHistCount; i++) {
    uint8_t p = patternHistory[i];
    bool hasLeftFeature  = (p & 0b1100000) != 0;
    bool hasRightFeature = (p & 0b0000011) != 0;
    bool hasCenter       = (p & 0b0011100) != 0;
    if ((hasLeftFeature || hasRightFeature) && hasCenter) {
      junctionVotes++;
    }
  }
  junctionSupported = (junctionVotes >= 2);

  return smoothedError;
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
  digitalWrite(PIN_LED, LOW);

  digitalWrite(PIN_STBY, HIGH);
  Serial.begin(9600);

  // Load previous calibration from EEPROM as baseline
  EEPROM.get(EEPROM_ADDR, calib);
  if (calib.magic == EEPROM_MAGIC) {
    for (int i = 0; i < 7; i++) {
      sensorThreshold[i] = calib.threshold[i];
    }
  }

  // 1. Initial 2-second countdown delay to set robot down on track
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(125);
    digitalWrite(PIN_LED, LOW);
    delay(125);
  }

  // 2. Automatic 360-degree 4-second calibration on power-on
  calibrateSensors();

  // 3. 1.5-second countdown (3 flashes) to position robot on start line
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(250);
    digitalWrite(PIN_LED, LOW);
    delay(250);
  }

  lastJunctionTime = millis();
  junctionArmed = true;
  errorHistCount = 0;
  errorHistIdx = 0;
}

void loop() {
  bool hasLeft  = false;
  bool hasRight = false;
  bool hasCross = false;
  bool allWhite = false;
  uint8_t pattern = 0;

  int rawError = readSensors(hasLeft, hasRight, hasCross, allWhite, pattern);

  // 6-reading data protection: outlier rejection, PID error smoothing & junction consistency
  bool junctionSupported = false;
  int error = applyDataProtection(rawError, pattern, junctionSupported);

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

  if ((hasLeft || hasRight || hasCross) && junctionArmed && junctionSupported) {
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

      bool straightAhead = isSensorOnLine(1) || isSensorOnLine(2) || isSensorOnLine(3) || 
                           isSensorOnLine(4) || isSensorOnLine(5);

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

  // Line loss protection: requires 100ms continuous allWhite to eliminate false turns on straight line
  static unsigned long lostLineStart = 0;
  if (allWhite) {
    if (lostLineStart == 0) {
      lostLineStart = millis();
    } else if (millis() - lostLineStart > 100) {
      turnAround(); // Spin 360 on the spot until middle sensor gets the line
      lostLineStart = 0;
      return;
    }
  } else {
    lostLineStart = 0; // Resets immediately when any sensor is on line
  }

  if (abs(error) > 40) {
    integral = constrain(integral + error, -1500, 1500);
  } else {
    integral = 0; // Zero windup when aligned with line
  }
  float derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * integral + Kd * derivative);

  int leftSpeed  = constrain(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
  lastError = error;
}
