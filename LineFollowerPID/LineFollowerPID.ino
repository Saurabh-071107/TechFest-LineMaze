#include <EEPROM.h>

const uint8_t PIN_PWMA = 5;
const uint8_t PIN_AIN1 = 3;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 6;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 9;
const uint8_t PIN_STBY = 8;

const uint8_t PIN_BTN = 10;
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
const int INCH_TIME    = 220;

float Kp = 0.048;
float Ki = 0.00005;
float Kd = 0.22;

const unsigned long MIN_JUNCTION_DEBOUNCE = 15;
bool junctionArmed = true;
uint8_t junctionClearCounter = 0;

int lastError = 0;
float integral = 0;
unsigned long lastJunctionTime = 0;

uint8_t tJunctionCount = 0;
const uint8_t FINISH_JUNCTION_TARGET = 3;
unsigned long lastTJunctionTime = 0;
const unsigned long MAX_T_JUNCTION_GAP = 750;

enum BotMode : uint8_t { MODE_DRY_RUN = 0, MODE_SPEED_RUN = 1 };
BotMode botMode = MODE_DRY_RUN;
uint8_t nextStage = 1;

const uint8_t MAX_NODES = 28;
const uint8_t MAX_PATH_LEN = 64;
const int16_t SNAP_RADIUS = 12;

const int8_t DIR_DX[4] = {0, 1, 0, -1};
const int8_t DIR_DY[4] = {1, 0, -1, 0};

struct MazeNode {
  int16_t x, y;
  uint8_t exitsMask;
  uint8_t visitedMask;
  uint8_t neighbor[4];
  uint8_t edgeCost[4];
};

MazeNode nodes[MAX_NODES];
uint8_t nodeCount = 0;
uint8_t currentNode = 0;
uint8_t currentHeading = 0;
uint8_t lastExitDir = 0;
int16_t curX = 0, curY = 0;
unsigned long segmentStartTime = 0;
uint8_t consecutiveRightTurns = 0;
bool lastTurnWasBack = false;

char shortestPath[MAX_PATH_LEN];
uint8_t shortestPathLen = 0;
uint8_t speedRunIdx = 0;

const uint16_t PATH_MAGIC = 0x5046;
const int PATH_EEPROM_ADDR = sizeof(CalibData);
struct StoredPath {
  uint16_t magic;
  uint8_t len;
  char turns[MAX_PATH_LEN];
};

uint8_t stackNode[MAX_PATH_LEN];
uint8_t stackInHeading[MAX_PATH_LEN];
char    stackTurn[MAX_PATH_LEN];
uint8_t stackLen = 0;
uint8_t preFinishStackLen = 0;
uint8_t preFinishNode = 0;

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

void resetTrackingState() {
  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
  errorHistCount = 0;
  errorHistIdx = 0;
}

uint8_t turnToDir(uint8_t h, char t) {
  if (t == 'R') return (h + 1) & 3;
  if (t == 'L') return (h + 3) & 3;
  if (t == 'B') return (h + 2) & 3;
  return h;
}

char dirsToTurn(uint8_t inDir, uint8_t outDir) {
  uint8_t d = (outDir - inDir + 4) & 3;
  return (d == 0) ? 'S' : (d == 1) ? 'R' : (d == 3) ? 'L' : 'B';
}

uint8_t allocNode(int16_t x, int16_t y, uint8_t exitsMask) {
  if (nodeCount >= MAX_NODES) return 0xFF;
  uint8_t id = nodeCount++;
  nodes[id].x = x;
  nodes[id].y = y;
  nodes[id].exitsMask = exitsMask;
  nodes[id].visitedMask = 0;
  for (uint8_t d = 0; d < 4; d++) {
    nodes[id].neighbor[d] = 0xFF;
    nodes[id].edgeCost[d] = 0xFF;
  }
  return id;
}

void initMazeSolver() {
  nodeCount = 0;
  curX = 0;
  curY = 0;
  currentHeading = 0;
  lastExitDir = 0;
  currentNode = allocNode(0, 0, (1 << 0));
  nodes[0].visitedMask = (1 << 0);
  stackLen = 0;
  preFinishStackLen = 0;
  preFinishNode = 0;
  shortestPathLen = 0;
  speedRunIdx = 0;
  consecutiveRightTurns = 0;
  lastTurnWasBack = false;
  botMode = MODE_DRY_RUN;
  segmentStartTime = millis();
}

uint8_t findOrCreateNode(int16_t x, int16_t y, uint8_t exitsMask) {
  for (uint8_t i = 1; i < nodeCount; i++) {
    if (abs(x - nodes[i].x) <= SNAP_RADIUS && abs(y - nodes[i].y) <= SNAP_RADIUS) {
      if ((nodes[i].exitsMask & exitsMask) != 0) {
        curX = nodes[i].x;
        curY = nodes[i].y;
        nodes[i].exitsMask |= exitsMask;
        return i;
      }
    }
  }
  return allocNode(x, y, exitsMask);
}

void collapseBStack() {
  while (stackLen >= 3 && stackTurn[stackLen - 2] == 'B') {
    uint8_t origIn = stackInHeading[stackLen - 3];
    uint8_t curOut = turnToDir(stackInHeading[stackLen - 1], stackTurn[stackLen - 1]);
    char merged = dirsToTurn(origIn, curOut);
    stackTurn[stackLen - 3] = merged;
    if (merged == 'B') stackNode[stackLen - 3] = 0xFF;
    stackLen -= 2;
  }
}

void recordAndReduceTurn(uint8_t nodeId, uint8_t inHead, char turn, uint8_t outDir) {
  if (stackLen < MAX_PATH_LEN) {
    stackNode[stackLen] = nodeId;
    stackInHeading[stackLen] = inHead;
    stackTurn[stackLen] = turn;
    stackLen++;
  }
  collapseBStack();
  if (nodeId != 0xFF && stackLen >= 2 && stackTurn[stackLen - 1] != 'B') {
    for (uint8_t m = 0; m + 1 < stackLen; m++) {
      if (stackNode[m] == nodeId) {
        char merged = dirsToTurn(stackInHeading[m], outDir);
        stackTurn[m] = merged;
        if (merged == 'B') stackNode[m] = 0xFF;
        stackLen = m + 1;
        collapseBStack();
        break;
      }
    }
  }
}

void recordDeadEnd() {
  if (botMode != MODE_DRY_RUN) return;
  if (stackLen == 0 || stackTurn[stackLen - 1] != 'B') {
    if (stackLen < MAX_PATH_LEN) {
      stackNode[stackLen] = 0xFF;
      stackInHeading[stackLen] = currentHeading;
      stackTurn[stackLen] = 'B';
      stackLen++;
    }
  }
  currentHeading = (currentHeading + 2) & 3;
  lastTurnWasBack = true;
  consecutiveRightTurns = 0;
  segmentStartTime = millis();
}

void saveShortestPathToEEPROM() {
  StoredPath sp;
  sp.magic = PATH_MAGIC;
  sp.len = shortestPathLen;
  for (uint8_t i = 0; i < MAX_PATH_LEN; i++) {
    sp.turns[i] = (i < shortestPathLen) ? shortestPath[i] : 0;
  }
  EEPROM.put(PATH_EEPROM_ADDR, sp);
}

void loadShortestPathFromEEPROM() {
  StoredPath sp;
  EEPROM.get(PATH_EEPROM_ADDR, sp);
  if (sp.magic == PATH_MAGIC && sp.len <= MAX_PATH_LEN) {
    shortestPathLen = sp.len;
    for (uint8_t i = 0; i < shortestPathLen; i++) {
      shortestPath[i] = sp.turns[i];
    }
  }
}

void computeShortestPath() {
  uint8_t validStackLen = preFinishStackLen;
  while (validStackLen > 0 && stackTurn[validStackLen - 1] == 'B') {
    validStackLen--;
  }
  shortestPathLen = 0;
  for (uint8_t i = 0; i < validStackLen && shortestPathLen < MAX_PATH_LEN; i++) {
    if (stackTurn[i] != 'B') {
      shortestPath[shortestPathLen++] = stackTurn[i];
    }
  }

  if (preFinishNode > 0 && preFinishNode < nodeCount) {
    uint16_t dist[MAX_NODES];
    uint8_t prevNode[MAX_NODES];
    uint8_t prevDir[MAX_NODES];
    bool used[MAX_NODES];

    for (uint8_t i = 0; i < nodeCount; i++) {
      dist[i] = 0xFFFF;
      prevNode[i] = 0xFF;
      prevDir[i] = 0;
      used[i] = false;
    }
    dist[0] = 0;

    for (uint8_t step = 0; step < nodeCount; step++) {
      uint8_t u = 0xFF;
      uint16_t best = 0xFFFF;
      for (uint8_t i = 0; i < nodeCount; i++) {
        if (!used[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u == 0xFF || u == preFinishNode) break;
      used[u] = true;

      for (uint8_t d = 0; d < 4; d++) {
        uint8_t v = nodes[u].neighbor[d];
        if (v < nodeCount && !used[v]) {
          uint16_t nd = dist[u] + nodes[u].edgeCost[d];
          if (nd < dist[v]) {
            dist[v] = nd;
            prevNode[v] = u;
            prevDir[v] = d;
          }
        }
      }
    }

    if (dist[preFinishNode] != 0xFFFF) {
      uint8_t revDirs[MAX_NODES];
      uint8_t count = 0;
      uint8_t cur = preFinishNode;
      while (cur != 0 && cur < nodeCount && count < MAX_NODES) {
        revDirs[count] = prevDir[cur];
        cur = prevNode[cur];
        count++;
      }
      if (cur == 0 && count >= 2 && (count - 1 < shortestPathLen)) {
        shortestPathLen = 0;
        uint8_t inDir = revDirs[count - 1];
        for (int8_t k = (int8_t)count - 2; k >= 0; k--) {
          uint8_t outDir = revDirs[k];
          shortestPath[shortestPathLen++] = dirsToTurn(inDir, outDir);
          inDir = outDir;
        }
      }
    }
  }
  saveShortestPathToEEPROM();
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

uint8_t readButtonPresses() {
  while (digitalRead(PIN_BTN) == HIGH) {
    digitalWrite(PIN_LED, ((millis() % 800) < 60) ? HIGH : LOW);
    delay(10);
  }
  uint8_t presses = 0;
  unsigned long lastRelease = millis();
  while (true) {
    if (digitalRead(PIN_BTN) == LOW) {
      digitalWrite(PIN_LED, HIGH);
      delay(30);
      while (digitalRead(PIN_BTN) == LOW) {
        delay(5);
      }
      digitalWrite(PIN_LED, LOW);
      delay(30);
      presses++;
      lastRelease = millis();
    }
    if (presses > 0 && (millis() - lastRelease >= 550)) {
      break;
    }
    delay(5);
  }
  return presses;
}

void waitForButtonCommand() {
  setMotors(0, 0);
  while (digitalRead(PIN_BTN) == LOW) {
    delay(10);
  }
  delay(30);

  while (true) {
    uint8_t p = readButtonPresses();
    uint8_t cmd = (p >= 2) ? p : nextStage;

    if (cmd == 1) {
      delay(400);
      calibrateSensors();
      nextStage = 2;
    } else if (cmd == 2) {
      for (uint8_t i = 0; i < 2; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(150);
        digitalWrite(PIN_LED, LOW);
        delay(150);
      }
      delay(400);
      initMazeSolver();
      botMode = MODE_DRY_RUN;
      tJunctionCount = 0;
      lastTJunctionTime = 0;
      resetTrackingState();
      junctionArmed = true;
      segmentStartTime = millis();
      nextStage = 3;
      return;
    } else {
      for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(120);
        digitalWrite(PIN_LED, LOW);
        delay(120);
      }
      delay(400);
      botMode = MODE_SPEED_RUN;
      speedRunIdx = 0;
      tJunctionCount = 0;
      lastTJunctionTime = 0;
      resetTrackingState();
      junctionArmed = true;
      segmentStartTime = millis();
      nextStage = 3;
      return;
    }
  }
}

void finishAndStop() {
  setMotors(0, 0);
  if (botMode == MODE_DRY_RUN) {
    computeShortestPath();
  }
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  nextStage = 3;
  waitForButtonCommand();
}

void pivot(bool turnToLeft) {
  int leftSpeed  = turnToLeft ? -TURN_SPEED : TURN_SPEED;
  int rightSpeed = turnToLeft ? TURN_SPEED  : -TURN_SPEED;

  setMotors(leftSpeed, rightSpeed);

  delay(100);
  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 150)) {
    delay(1);
  }

  t = millis();
  while (!isSensorOnLine(3) && (millis() - t < 1200)) {
    delay(1);
  }

  setMotors(-leftSpeed / 2, -rightSpeed / 2);
  delay(30);
  setMotors(0, 0);
  delay(30);

  resetTrackingState();
}

void driveStraightPastJunction() {
  setMotors(BASE_SPEED, BASE_SPEED);
  delay(60);
  resetTrackingState();
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
    resetTrackingState();
  } else {
    setMotors(0, 0);
    waitForButtonCommand();
  }
}

void executeTurn(char turn) {
  if (turn == 'R') {
    pivot(false);
  } else if (turn == 'S') {
    driveStraightPastJunction();
  } else if (turn == 'L') {
    pivot(true);
  } else {
    search360AndStop();
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

void setup() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  pinMode(PIN_BTN, INPUT_PULLUP);
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
  } else {
    for (int i = 0; i < 7; i++) {
      sensorThreshold[i] = 512;
      initialMin[i] = 0;
      initialMax[i] = 1023;
      initialThreshold[i] = 512;
      currentMin[i] = 0.0f;
      currentMax[i] = 1023.0f;
    }
  }

  loadShortestPathFromEEPROM();

  nextStage = 1;
  waitForButtonCommand();
}

void loop() {
  if (digitalRead(PIN_BTN) == LOW) {
    waitForButtonCommand();
    return;
  }

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
      recordDeadEnd();
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

    bool isTJunction = (branchLeft && branchRight) || hasCross || confCross;
    if (isTJunction) {
      unsigned long timeSinceLastT = millis() - lastTJunctionTime;
      if (tJunctionCount == 0 || timeSinceLastT > MAX_T_JUNCTION_GAP) {
        tJunctionCount = 1;
        preFinishStackLen = stackLen;
        preFinishNode = currentNode;
      } else {
        tJunctionCount++;
      }
      lastTJunctionTime = millis();

      if (tJunctionCount >= FINISH_JUNCTION_TARGET) {
        finishAndStop();
        return;
      }
    } else {
      tJunctionCount = 0;
    }

    bool straightAhead = isSensorOnLine(2) || isSensorOnLine(3) || isSensorOnLine(4);

    if (botMode == MODE_SPEED_RUN) {
      if (speedRunIdx < shortestPathLen) {
        char cmd = shortestPath[speedRunIdx++];
        if (cmd == 'R' && !branchRight && !straightAhead && branchLeft) cmd = 'L';
        else if (cmd == 'L' && !branchLeft && !straightAhead && branchRight) cmd = 'R';
        else if (cmd == 'S' && !straightAhead) cmd = branchRight ? 'R' : (branchLeft ? 'L' : 'B');
        executeTurn(cmd);
      } else if (isTJunction && straightAhead) {
        driveStraightPastJunction();
      } else if (branchRight) {
        pivot(false);
      } else if (straightAhead) {
        driveStraightPastJunction();
      } else if (branchLeft) {
        pivot(true);
      } else {
        search360AndStop();
      }
      return;
    }

    if (!branchRight && !straightAhead && !branchLeft) {
      search360AndStop();
      recordDeadEnd();
      return;
    }

    uint8_t inHead   = currentHeading;
    uint8_t entryDir = (inHead + 2) & 3;
    uint8_t rightDir = (inHead + 1) & 3;
    uint8_t leftDir  = (inHead + 3) & 3;

    uint8_t exitsMask = (1 << entryDir);
    if (branchRight)   exitsMask |= (1 << rightDir);
    if (straightAhead) exitsMask |= (1 << inHead);
    if (branchLeft)    exitsMask |= (1 << leftDir);

    uint8_t segCost = (uint8_t)constrain((millis() - segmentStartTime) / 40UL, 1UL, 250UL);
    uint8_t arrivedNode = 0xFF;

    if (lastTurnWasBack && currentNode < nodeCount) {
      arrivedNode = currentNode;
      curX = nodes[arrivedNode].x;
      curY = nodes[arrivedNode].y;
      nodes[arrivedNode].exitsMask |= exitsMask;
    } else {
      curX += (int16_t)DIR_DX[inHead] * segCost;
      curY += (int16_t)DIR_DY[inHead] * segCost;
      arrivedNode = findOrCreateNode(curX, curY, exitsMask);
      if (currentNode < nodeCount && arrivedNode < nodeCount && arrivedNode != currentNode) {
        if (segCost < nodes[currentNode].edgeCost[lastExitDir]) {
          nodes[currentNode].neighbor[lastExitDir] = arrivedNode;
          nodes[currentNode].edgeCost[lastExitDir] = segCost;
        }
        if (segCost < nodes[arrivedNode].edgeCost[entryDir]) {
          nodes[arrivedNode].neighbor[entryDir] = currentNode;
          nodes[arrivedNode].edgeCost[entryDir] = segCost;
        }
      }
    }

    if (arrivedNode < nodeCount) {
      nodes[arrivedNode].visitedMask |= (1 << entryDir);
    }

    const char    candTurn[3]  = {'R', 'S', 'L'};
    const bool    candAvail[3] = {branchRight, straightAhead, branchLeft};
    const uint8_t candDir[3]   = {rightDir, inHead, leftDir};
    bool blockRight = (consecutiveRightTurns >= 4) && (straightAhead || branchLeft);

    char chosenTurn = 0;
    if (arrivedNode < nodeCount) {
      for (uint8_t k = 0; k < 3; k++) {
        if (candAvail[k] && !(k == 0 && blockRight)) {
          if ((nodes[arrivedNode].visitedMask & (1 << candDir[k])) == 0) {
            chosenTurn = candTurn[k];
            break;
          }
        }
      }
    }
    if (chosenTurn == 0) {
      for (uint8_t k = 0; k < 3; k++) {
        if (candAvail[k] && !(k == 0 && blockRight)) {
          chosenTurn = candTurn[k];
          break;
        }
      }
    }
    if (chosenTurn == 0) {
      chosenTurn = branchRight ? 'R' : (straightAhead ? 'S' : 'L');
    }

    uint8_t outDir = turnToDir(inHead, chosenTurn);
    if (arrivedNode < nodeCount) {
      nodes[arrivedNode].visitedMask |= (1 << outDir);
    }

    recordAndReduceTurn(arrivedNode, inHead, chosenTurn, outDir);
    currentNode = arrivedNode;
    lastExitDir = outDir;
    currentHeading = outDir;
    lastTurnWasBack = false;
    consecutiveRightTurns = (chosenTurn == 'R') ? (consecutiveRightTurns + 1) : 0;

    executeTurn(chosenTurn);
    segmentStartTime = millis();
    return;
  }

  if (tJunctionCount > 0 && (millis() - lastTJunctionTime > MAX_T_JUNCTION_GAP)) {
    tJunctionCount = 0;
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
