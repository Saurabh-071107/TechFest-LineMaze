#include <EEPROM.h>

const uint8_t PIN_PWMA = 5;
const uint8_t PIN_AIN1 = 3;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 6;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 9;
const uint8_t PIN_STBY = 8;

const uint8_t PIN_LED = 13;

const uint8_t HISTORY_LEN = 6;
int errorHistory[HISTORY_LEN] = {0};
uint8_t errorHistCount = 0;
uint8_t errorHistIdx = 0;
uint8_t patternHistory[HISTORY_LEN] = {0};

const uint8_t SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};
const int SENSOR_WEIGHTS[7]  = {-3000, -2000, -1000, 0, 1000, 2000, 3000};

const bool BLACK_LINE = true;

const uint16_t EEPROM_MAGIC = 0x5446;
const int EEPROM_ADDR = 0;
struct CalibData {
  uint16_t magic;
  int threshold[7];
  int minVal[7];
  int maxVal[7];
};
CalibData calib;

int initialMin[7] = {0};
int initialMax[7] = {1023};
int initialThreshold[7] = {512};

float currentMin[7] = {0};
float currentMax[7] = {1023};
int sensorThreshold[7] = {0};

int rawHistory[7][3] = {0};

const float ADAPT_ALPHA       = 0.01f;
const int   MAX_CALIB_DRIFT   = 120;
const int   MIN_CALIB_SPAN    = 160;
const int   NOISE_DELTA_MAX   = 30;

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

const int BASE_SPEED   = 145;
const int MAX_SPEED    = 210;
const int TURN_SPEED   = 125;
const int INCH_TIME    = 200;

float Kp = 0.048;
float Ki = 0.00005;
float Kd = 0.22;

const unsigned long MIN_JUNCTION_DEBOUNCE = 15;
bool junctionArmed = true;
uint8_t junctionClearCounter = 0;

int lastError = 0;
float integral = 0;
unsigned long lastJunctionTime = 0;

uint8_t consecutiveJunctionCount = 0;
const uint8_t FINISH_JUNCTION_TARGET = 3;
const unsigned long BACK_TO_BACK_WINDOW = 650;
uint16_t sustainedLineCycles = 0;

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

void blinkJunction() {
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(25);
    digitalWrite(PIN_LED, LOW);
    delay(20);
  }
}

void finishAndStop() {
  setMotors(0, 0);
  while (true) {
    digitalWrite(PIN_LED, HIGH);
    delay(150);
    digitalWrite(PIN_LED, LOW);
    delay(150);
  }
}

void pivot(bool turnToLeft) {
  int leftSpeed  = turnToLeft ? -TURN_SPEED : TURN_SPEED;
  int rightSpeed = turnToLeft ? TURN_SPEED  : -TURN_SPEED;

  setMotors(leftSpeed, rightSpeed);

  delay(40);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 150)) {
    delay(1);
  }

  t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 900)) {
    delay(1);
  }

  setMotors(-leftSpeed / 2, -rightSpeed / 2);
  delay(28);
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

void search360AndStop() {

  int leftSpeed  = (lastError >= 0) ? -TURN_SPEED : TURN_SPEED;
  int rightSpeed = (lastError >= 0) ? TURN_SPEED  : -TURN_SPEED;

  setMotors(leftSpeed, rightSpeed);

  bool lineFound = false;
  unsigned long tStart = millis();
  const unsigned long SWEEP_360_TIME = 3800;

  while (millis() - tStart < SWEEP_360_TIME) {
    if (isSensorOnLine(3)) {
      delay(3);
      if (isSensorOnLine(3)) {
        lineFound = true;
        break;
      }
    }

    digitalWrite(PIN_LED, (millis() / 80) % 2);
    delay(1);
  }

  if (lineFound) {

    setMotors(-leftSpeed / 2, -rightSpeed / 2);
    delay(28);
    setMotors(0, 0);
    delay(20);

    digitalWrite(PIN_LED, LOW);
    integral = 0;
    lastError = 0;
    lastJunctionTime = millis();
    junctionArmed = false;
    junctionClearCounter = 0;
    errorHistCount = 0;
    errorHistIdx = 0;
  } else {

    setMotors(0, 0);
    while (true) {
      digitalWrite(PIN_LED, HIGH);
      delay(400);
      digitalWrite(PIN_LED, LOW);
      delay(400);
    }
  }
}

int readSensors(bool &hasLeft, bool &hasRight, bool &hasCross, bool *allWhite = nullptr, uint8_t *outPattern = nullptr) {
  long weightedSum = 0;
  int totalActive = 0;
  int activeCount = 0;
  uint8_t pattern = 0;

  for (int i = 0; i < 7; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    rawHistory[i][2] = rawHistory[i][1];
    rawHistory[i][1] = rawHistory[i][0];
    rawHistory[i][0] = raw;

    if (isOnLine(raw, i)) {
      pattern |= (1 << (6 - i));
      int activeVal = BLACK_LINE ? (raw - sensorThreshold[i]) : (sensorThreshold[i] - raw);
      weightedSum += (long)activeVal * SENSOR_WEIGHTS[i];
      totalActive += activeVal;
      activeCount++;
    }
  }

  if (outPattern) *outPattern = pattern;
  if (allWhite)   *allWhite   = (pattern == 0b0000000);

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

int applyDataProtection(int rawError, uint8_t rawPattern, bool &junctionSupported) {

  int sum = 0;
  uint8_t count = errorHistCount;
  for (uint8_t i = 0; i < count; i++) {
    sum += errorHistory[i];
  }
  int avgError = (count > 0) ? (sum / count) : rawError;

  int protectedError = rawError;
  if (count >= 3) {
    int deviation = rawError - avgError;

    if (deviation > 1800) {
      protectedError = avgError + 1200;
    } else if (deviation < -1800) {
      protectedError = avgError - 1200;
    }
  }

  errorHistory[errorHistIdx] = protectedError;
  patternHistory[errorHistIdx] = rawPattern;
  errorHistIdx = (errorHistIdx + 1) % HISTORY_LEN;
  if (errorHistCount < HISTORY_LEN) errorHistCount++;

  sum = 0;
  for (uint8_t i = 0; i < errorHistCount; i++) {
    sum += errorHistory[i];
  }
  int smoothedError = sum / errorHistCount;

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

bool calibrationAllowed(int error, uint8_t pattern, bool hasLeft, bool hasRight, bool hasCross, bool allWhite) {

  if (allWhite || pattern == 0) return false;

  if (hasLeft || hasRight || hasCross) return false;

  if (!junctionArmed || (millis() - lastJunctionTime < 250)) return false;

  uint8_t activeCount = 0;
  for (uint8_t b = 0; b < 7; b++) {
    if (pattern & (1 << b)) activeCount++;
  }
  if (activeCount < 1 || activeCount > 3) return false;

  if (abs(error) > 800) return false;

  return true;
}

void updateOnlineCalibration() {
  for (int i = 0; i < 7; i++) {
    int curr     = rawHistory[i][0];
    int prev     = rawHistory[i][1];
    int prevPrev = rawHistory[i][2];

    if (abs(curr - prev) > NOISE_DELTA_MAX || abs(prev - prevPrev) > NOISE_DELTA_MAX) continue;
    if (curr < 20 || curr > 1000) continue;

    if (curr > sensorThreshold[i] + 40) {
      currentMax[i] += ADAPT_ALPHA * ((float)curr - currentMax[i]);
    } else if (curr < sensorThreshold[i] - 40) {
      currentMin[i] += ADAPT_ALPHA * ((float)curr - currentMin[i]);
    }

    if ((currentMax[i] - currentMin[i]) < MIN_CALIB_SPAN) {
      currentMin[i] = (float)initialMin[i];
      currentMax[i] = (float)initialMax[i];
    }

    int targetThreshold = (int)((currentMin[i] + currentMax[i]) / 2.0f);
    sensorThreshold[i] = constrain(targetThreshold,
                                   initialThreshold[i] - MAX_CALIB_DRIFT,
                                   initialThreshold[i] + MAX_CALIB_DRIFT);
  }
}

void calibrateSensors() {
  int minVal[7], maxVal[7];
  for (int i = 0; i < 7; i++) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  setMotors(-TURN_SPEED, TURN_SPEED);
  unsigned long tStart = millis();

  while (millis() - tStart < 4000) {
    for (int i = 0; i < 7; i++) {
      int v = analogRead(SENSOR_PINS[i]);
      if (v < minVal[i]) minVal[i] = v;
      if (v > maxVal[i]) maxVal[i] = v;
    }

    digitalWrite(PIN_LED, (millis() / 70) % 2);
    delay(2);
  }

  setMotors(0, 0);

  calib.magic = EEPROM_MAGIC;
  for (int i = 0; i < 7; i++) {
    calib.minVal[i] = minVal[i];
    calib.maxVal[i] = maxVal[i];
    calib.threshold[i] = (minVal[i] + maxVal[i]) / 2;

    initialMin[i] = minVal[i];
    initialMax[i] = maxVal[i];
    initialThreshold[i] = calib.threshold[i];

    currentMin[i] = (float)minVal[i];
    currentMax[i] = (float)maxVal[i];
    sensorThreshold[i] = calib.threshold[i];

    for (int s = 0; s < 3; s++) {
      rawHistory[i][s] = calib.threshold[i];
    }
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

  EEPROM.get(EEPROM_ADDR, calib);
  if (calib.magic == EEPROM_MAGIC) {
    for (int i = 0; i < 7; i++) {
      initialMin[i] = calib.minVal[i];
      initialMax[i] = calib.maxVal[i];
      initialThreshold[i] = calib.threshold[i];

      currentMin[i] = (float)calib.minVal[i];
      currentMax[i] = (float)calib.maxVal[i];
      sensorThreshold[i] = calib.threshold[i];

      for (int s = 0; s < 3; s++) {
        rawHistory[i][s] = calib.threshold[i];
      }
    }
  }

  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(125);
    digitalWrite(PIN_LED, LOW);
    delay(125);
  }

  calibrateSensors();

  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(250);
    digitalWrite(PIN_LED, LOW);
    delay(250);
  }

  lastJunctionTime = millis();
  junctionArmed = true;
  consecutiveJunctionCount = 0;
  sustainedLineCycles = 0;
  errorHistCount = 0;
  errorHistIdx = 0;
}

void loop() {
  bool hasLeft  = false;
  bool hasRight = false;
  bool hasCross = false;
  bool allWhite = false;
  uint8_t pattern = 0;

  int rawError = readSensors(hasLeft, hasRight, hasCross, &allWhite, &pattern);

  static unsigned long lostLineStart = 0;
  if (allWhite) {
    if (lostLineStart == 0) {
      lostLineStart = millis();
    }

    if (millis() - lostLineStart > 100) {
      search360AndStop();
      lostLineStart = 0;
      return;
    }

    setMotors(BASE_SPEED / 2, BASE_SPEED / 2);
    return;
  }
  lostLineStart = 0;

  bool junctionSupported = false;
  int error = applyDataProtection(rawError, pattern, junctionSupported);

  if (!junctionArmed) {
    bool outerClear = !isSensorOnLine(0) && !isSensorOnLine(1) &&
                      !isSensorOnLine(5) && !isSensorOnLine(6);
    unsigned long timeSinceJunction = millis() - lastJunctionTime;
    if ((outerClear && timeSinceJunction > MIN_JUNCTION_DEBOUNCE) ||
        (timeSinceJunction > 120 && isSensorOnLine(3))) {
      junctionClearCounter++;
      if (junctionClearCounter >= 1) {
        junctionArmed = true;
        junctionClearCounter = 0;
      }
    } else {
      junctionClearCounter = 0;
    }
  }

  if ((hasLeft || hasRight || hasCross) && junctionArmed && junctionSupported) {
    unsigned long timeSinceLast = millis() - lastJunctionTime;
    if (timeSinceLast < BACK_TO_BACK_WINDOW) {
      consecutiveJunctionCount++;
    } else {
      consecutiveJunctionCount = 1;
    }
    sustainedLineCycles = 0;

    if (consecutiveJunctionCount >= FINISH_JUNCTION_TARGET) {
      setMotors(-BASE_SPEED / 2, -BASE_SPEED / 2);
      delay(30);
      finishAndStop();
      return;
    }

    bool confL = false, confR = false, confCross = false;
    readSensors(confL, confR, confCross);

    junctionArmed = false;
    junctionClearCounter = 0;
    lastJunctionTime = millis();

    bool branchRight = hasRight || hasCross || confR || confCross || (isSensorOnLine(6) && isSensorOnLine(5));
    bool branchLeft  = hasLeft  || hasCross || confL || confCross || (isSensorOnLine(0) && isSensorOnLine(1));

    setMotors(BASE_SPEED, BASE_SPEED);
    unsigned long tInch = millis();
    while (millis() - tInch < INCH_TIME) {
      if (isSensorOnLine(6) && isSensorOnLine(5)) branchRight = true;
      if (isSensorOnLine(0) && isSensorOnLine(1)) branchLeft  = true;
    }

    setMotors(-BASE_SPEED / 2, -BASE_SPEED / 2);
    delay(25);
    setMotors(0, 0);

    blinkJunction();

    bool straightAhead = isSensorOnLine(2) || isSensorOnLine(3) || isSensorOnLine(4);

    if (branchRight) {
      pivot(false);
    } else if (straightAhead) {
      setMotors(BASE_SPEED, BASE_SPEED);
      delay(60);
      integral = 0;
      lastError = 0;
      lastJunctionTime = millis();
      junctionArmed = false;
      junctionClearCounter = 0;
    } else if (branchLeft) {
      pivot(true);
    } else {
      search360AndStop();
    }
    return;
  }

  sustainedLineCycles++;
  if (sustainedLineCycles > 30) {
    consecutiveJunctionCount = 0;
  }

  if (calibrationAllowed(error, pattern, hasLeft, hasRight, hasCross, allWhite)) {
    updateOnlineCalibration();
  }

  if (abs(error) > 40) {
    integral = constrain(integral + error, -1500, 1500);
  } else {
    integral = 0;
  }
  float derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * integral + Kd * derivative);

  int leftSpeed  = constrain(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
  lastError = error;
}
