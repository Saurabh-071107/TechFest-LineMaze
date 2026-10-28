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

  // 1. Initial 50ms unstick spin to clear center sensor S4 off previous orientation
  delay(50);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 120)) {
    delay(1);
  }

  // 2. Rotate until center sensor (S4 = index 3) acquires the arriving 90° branch
  t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 800)) {
    delay(1);
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
  // Determine spin direction toward side where line was last tracked
  int leftSpeed  = (lastError > 0) ? TURN_SPEED  : -TURN_SPEED;
  int rightSpeed = (lastError > 0) ? -TURN_SPEED : TURN_SPEED;
  setMotors(leftSpeed, rightSpeed);

  // 1. Initial spin to clear dead-end orientation (min 80ms)
  delay(80);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 200)) {
    delay(1);
  }

  // 2. Spin on the spot until middle sensor (S4 = index 3) locks onto the return line
  t = millis();
  while (millis() - t < 1500) {
    if (isSensorOnLine(3)) {
      delay(3);
      if (isSensorOnLine(3)) {
        break; // Double-confirmed middle sensor is securely on the line
      }
    }
    delay(1);
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

  allWhite = (pattern == 0b0000000);

  if (pattern == 0b1000001 || pattern == 0b1010101) {
    hasLeft = false;
    hasRight = false;
    hasCross = false;
    return lastError;
  }

  bool leftBranch  = ((pattern & 0b1000000) != 0) && ((pattern & 0b0110000) != 0);
  bool rightBranch = ((pattern & 0b0000001) != 0) && ((pattern & 0b0000110) != 0);

  hasCross = (leftBranch && rightBranch) || (pattern == 0b1111111) || (pattern == 0b1110111) ||
             (activeCount >= 5 && ((pattern & 0b1000000) != 0) && ((pattern & 0b0000001) != 0));

  hasRight = rightBranch && !hasCross;
  hasLeft  = leftBranch && !hasCross;

  if (totalActive > 0) {
    int continuousError = (int)(weightedSum / totalActive);
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

  // 5. Junction Data Protection: check if recent history confirms genuine branch candidate
  uint8_t junctionVotes = 0;
  for (uint8_t i = 0; i < errorHistCount; i++) {
    uint8_t p = patternHistory[i];
    bool hasLeftFeature  = ((p & 0b1000000) != 0) && ((p & 0b0110000) != 0);
    bool hasRightFeature = ((p & 0b0000001) != 0) && ((p & 0b0000110) != 0);
    if (hasLeftFeature || hasRightFeature || (p == 0b1111111)) {
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

  // Line loss handling: when all sensors see white, filter seams/gaps before declaring true dead end
  static unsigned long lostLineStart = 0;
  if (allWhite) {
    if (lostLineStart == 0) {
      lostLineStart = millis();
    }
    // Filter optical seams/gaps: require 120ms continuous white before declaring true dead end
    if (millis() - lostLineStart > 120) {
      turnAround(); // True dead end confirmed -> 180° U-turn
      lostLineStart = 0;
      return;
    }
    // During seam debounce: coast straight ahead smoothly without turning or accelerating into white space
    setMotors(BASE_SPEED / 2, BASE_SPEED / 2);
    return;
  }
  lostLineStart = 0;

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

      // Track genuine branches observed upon arrival
      bool branchRight = hasRight || hasCross || confR || confCross || (isSensorOnLine(6) && isSensorOnLine(5));
      bool branchLeft  = hasLeft  || hasCross || confL || confCross || (isSensorOnLine(0) && isSensorOnLine(1));

      // Inching forward to align drive axle over junction node while actively sampling wings
      setMotors(BASE_SPEED, BASE_SPEED);
      unsigned long tInch = millis();
      while (millis() - tInch < INCH_TIME) {
        if (isSensorOnLine(6) && isSensorOnLine(5)) branchRight = true;
        if (isSensorOnLine(0) && isSensorOnLine(1)) branchLeft  = true;
      }

      // Active brake to kill forward momentum directly over junction node before decision
      setMotors(-BASE_SPEED / 2, -BASE_SPEED / 2);
      delay(20);
      setMotors(0, 0);
      delay(15);

      // Evaluate straight ahead path using center sensors S3, S4, S5 (indices 2, 3, 4)
      bool straightAhead = isSensorOnLine(2) || isSensorOnLine(3) || isSensorOnLine(4);

      // STRICT PRIORITY: RIGHT -> STRAIGHT -> LEFT -> U-TURN
      if (branchRight) {
        pivot(false); // Priority 1: Turn Right
      } else if (straightAhead) {
        setMotors(BASE_SPEED, BASE_SPEED); // Priority 2: Go Straight
        delay(60);
        integral = 0;
        lastError = 0;
        lastJunctionTime = millis();
        junctionArmed = false;
        junctionClearCounter = 0;
      } else if (branchLeft) {
        pivot(true);  // Priority 3: Turn Left
      } else {
        turnAround(); // Priority 4: U-Turn (Dead End)
      }
      digitalWrite(PIN_LED, LOW);
      return;
    }
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
