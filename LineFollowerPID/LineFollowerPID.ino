const uint8_t PIN_PWMA = 5;
const uint8_t PIN_AIN1 = 3;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 6;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 9;
const uint8_t PIN_STBY = 8;

const uint8_t PIN_LED = 13;

const uint8_t SENSOR_PINS[7] = {A0, A1, A2, A3, A4, A5, A6};
const int SENSOR_WEIGHTS[7]  = {-3000, -2000, -1000, 0, 1000, 2000, 3000};

const bool BLACK_LINE = true;
const int THRESHOLD = 500;

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

const int BASE_SPEED   = 110;
const int MAX_SPEED    = 170;
const int TURN_SPEED   = 105;
const int INCH_TIME    = 130;

float Kp = 0.06;
float Ki = 0.0001;
float Kd = 0.30;

const unsigned long MIN_JUNCTION_DEBOUNCE = 15; // Minimal 20ms debounce for tightly spaced junctions
bool junctionArmed = true;
uint8_t junctionClearCounter = 0;

int lastError = 0;
float integral = 0;
unsigned long lastJunctionTime = 0;

bool isOnLine(int raw) {
  return BLACK_LINE ? (raw > THRESHOLD) : (raw < THRESHOLD);
}

bool isSensorOnLine(uint8_t idx) {
  return isOnLine(analogRead(SENSOR_PINS[idx]));
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
  uint8_t assistSensor = turnToLeft ? 2 : 4;

  setMotors(leftSpeed, rightSpeed);
  delay(120);

  unsigned long t = millis();
  while (isSensorOnLine(3) && (millis() - t < 400)) {
  }

  t = millis();
  while (!isSensorOnLine(3) && !isSensorOnLine(assistSensor) && (millis() - t < 650)) {
  }

  setMotors(0, 0);
  delay(40);
  setMotors(BASE_SPEED, BASE_SPEED);
  delay(120);

  integral = 0;
  lastError = 0;
  lastJunctionTime = millis();
  junctionArmed = false;
  junctionClearCounter = 0;
}

void turnAround() {
  setMotors(-TURN_SPEED, TURN_SPEED);
  delay(180);

  unsigned long t = millis();
  while (!isSensorOnLine(3) && !isSensorOnLine(2) && !isSensorOnLine(4) && (millis() - t < 2500)) {
  }

  setMotors(0, 0);
  delay(40);

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
    s[i] = isOnLine(raw);
    if (s[i]) {
      int activeVal = BLACK_LINE ? (raw - THRESHOLD) : (THRESHOLD - raw);
      weightedSum += (long)activeVal * SENSOR_WEIGHTS[i];
      totalActive += activeVal;
      activeCount++;
    }
  }

  allWhite = (totalActive == 0);

  bool centerOnLine = (s[2] || s[3] || s[4]);

  hasCross = ((s[0] || s[1]) && (s[5] || s[6]) && centerOnLine) || (activeCount >= 5);
  hasLeft  = s[0] && s[1] && (s[3] || s[4]) && !hasCross;
  hasRight = s[6] && s[5] && (s[3] || s[2]) && !hasCross;

  if (totalActive > 0) {
    return (int)(weightedSum / totalActive);
  }

  return 0;
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
    setMotors(0, 0);
    delay(15);
    bool confL = false, confR = false, confCross = false, confWhite = false;
    readSensors(confL, confR, confCross, confWhite);

    if (confL || confR || confCross) {
      junctionArmed = false;
      junctionClearCounter = 0;
      lastJunctionTime = millis();

      pauseAndBlink();

      setMotors(BASE_SPEED, BASE_SPEED);
      delay(INCH_TIME);
      setMotors(0, 0);
      delay(40);

      bool straightAhead = isSensorOnLine(2) || isSensorOnLine(3) || isSensorOnLine(4);

      if (confR || confCross) {
        pivot(false);
      } else if (straightAhead) {
        setMotors(BASE_SPEED, BASE_SPEED);
        delay(120);
        integral = 0;
        lastError = 0;
        lastJunctionTime = millis();
        junctionArmed = false;
        junctionClearCounter = 0;
      } else if (confL) {
        pivot(true);
      } else {
        turnAround();
      }
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
