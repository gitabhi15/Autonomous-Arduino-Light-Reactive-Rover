#include "AFMotor_R4.h"
#include <Wire.h>

// where the sensors connect to the board
const uint8_t IR_LEFT_PIN = A2;
const uint8_t IR_RIGHT_PIN = A3;
const uint8_t LDR_LEFT_PIN = A0;
const uint8_t LDR_RIGHT_PIN = A1;
const uint8_t STATUS_LED_PIN = A4;
const uint8_t EDGE_PIN = A5;

// how the sensors report data
const bool IR_ACTIVE_LOW = true;
const bool EDGE_ACTIVE_LOW = true;

// setting up the four wheels
AF_DCMotor motorFL(1);
AF_DCMotor motorRL(2);
AF_DCMotor motorFR(3);
AF_DCMotor motorRR(4);

// fixing motor directions so forward is actually forward
#define FL_LOGICAL_FORWARD BACKWARD
#define FR_LOGICAL_FORWARD BACKWARD
#define RL_LOGICAL_FORWARD BACKWARD
#define RR_LOGICAL_FORWARD BACKWARD

// settings for speed and timing
const uint8_t BASE_SPEED = 255;
const uint8_t SPIN_PWM = 160;
const unsigned long SPIN_MS = 260;
const unsigned long BACKUP_MS = 220;
const unsigned long WANDER_MS = 250;

// variables for smoothing out light readings
float LDR_ALPHA = 0.13f;
float LDR_AMBIENT_ALPHA = 0.008f;

// thresholds for detecting the light beacon
int REL_DETECT_THRESHOLD = 180;
int GOAL_REL_THRESHOLD = 280;
int CENTER_DEADZONE_REL = 140;

// steering behavior settings
float STEER_K = 0.22f;
int BEACON_BASE_SPEED = 100;
int BEACON_PWM_MAX = 200;

int RAPID_RISE_THRESHOLD = 60;
unsigned long CALIB_MS = 2000UL;

// tracking current light levels
float emaL = 0, emaR = 0;
float ambL = 0, ambR = 0;
int lastTotalRel = -1;
int stallCount = 0;
unsigned long STALL_PUSH_MS = 220;

// logic to make sure individual motors spin the correct way
inline void runMotorLogical(AF_DCMotor &m, uint8_t logicalDirection) {
  uint8_t phys;
  if (&m == &motorFL)
    phys = (logicalDirection == FORWARD) ? FL_LOGICAL_FORWARD : (FL_LOGICAL_FORWARD == FORWARD ? BACKWARD : FORWARD);
  else if (&m == &motorFR)
    phys = (logicalDirection == FORWARD) ? FR_LOGICAL_FORWARD : (FR_LOGICAL_FORWARD == FORWARD ? BACKWARD : FORWARD);
  else if (&m == &motorRL)
    phys = (logicalDirection == FORWARD) ? RL_LOGICAL_FORWARD : (RL_LOGICAL_FORWARD == FORWARD ? BACKWARD : FORWARD);
  else
    phys = (logicalDirection == FORWARD) ? RR_LOGICAL_FORWARD : (RR_LOGICAL_FORWARD == FORWARD ? BACKWARD : FORWARD);

  m.run(phys);
}

// helper to move all wheels at once
inline void runAllLogical(uint8_t dir) {
  runMotorLogical(motorFL, dir);
  runMotorLogical(motorFR, dir);
  runMotorLogical(motorRL, dir);
  runMotorLogical(motorRR, dir);
}

// stops everything immediately
void stopAll() {
  motorFL.run(RELEASE);
  motorFR.run(RELEASE);
  motorRL.run(RELEASE);
  motorRR.run(RELEASE);
}

// just drive forward
void moveForward() {
  motorFL.setSpeed(BASE_SPEED);
  motorFR.setSpeed(BASE_SPEED);
  motorRL.setSpeed(BASE_SPEED);
  motorRR.setSpeed(BASE_SPEED);
  runAllLogical(FORWARD);
}

// drive forward but control the speed
void moveForwardDirectPWM(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorFL.setSpeed(pwm);
  motorFR.setSpeed(pwm);
  motorRL.setSpeed(pwm);
  motorRR.setSpeed(pwm);
  runAllLogical(FORWARD);
}

// spin the robot to the left
void spinLeft(unsigned long ms = SPIN_MS) {
  motorFL.setSpeed(SPIN_PWM);
  motorFR.setSpeed(SPIN_PWM);
  motorRL.setSpeed(SPIN_PWM);
  motorRR.setSpeed(SPIN_PWM);

  runMotorLogical(motorFL, BACKWARD);
  runMotorLogical(motorRL, BACKWARD);
  runMotorLogical(motorFR, FORWARD);
  runMotorLogical(motorRR, FORWARD);
  delay(ms);
  stopAll();
}

// spin the robot to the right
void spinRight(unsigned long ms = SPIN_MS) {
  motorFL.setSpeed(SPIN_PWM);
  motorFR.setSpeed(SPIN_PWM);
  motorRL.setSpeed(SPIN_PWM);
  motorRR.setSpeed(SPIN_PWM);

  runMotorLogical(motorFL, FORWARD);
  runMotorLogical(motorRL, FORWARD);
  runMotorLogical(motorFR, BACKWARD);
  runMotorLogical(motorRR, BACKWARD);
  delay(ms);
  stopAll();
}

// back up a bit then turn away
void backupAndSpin() {
  motorFL.setSpeed(SPIN_PWM);
  motorFR.setSpeed(SPIN_PWM);
  motorRL.setSpeed(SPIN_PWM);
  motorRR.setSpeed(SPIN_PWM);
  runAllLogical(BACKWARD);
  delay(BACKUP_MS);
  stopAll();

  spinLeft();
}

// wander around looking for things
void wanderStep() {
  unsigned long start = millis();
  moveForward();
  while (millis() - start < WANDER_MS) {
    int L = digitalRead(IR_LEFT_PIN);
    int R = digitalRead(IR_RIGHT_PIN);
    if (IR_ACTIVE_LOW) {
      L = !L;
      R = !R;
    }

    // if we hit something stop and turn
    if (L || R) {
      stopAll();
      backupAndSpin();
      return;
    }

    int edgeRaw = digitalRead(EDGE_PIN);
    int edgeDetected = EDGE_ACTIVE_LOW ? !edgeRaw : edgeRaw;

    // if we see a cliff stop immediately
    if (edgeDetected) {
      stopAll();
      backupAndSpin();
      return;
    }
    delay(15);
  }
  stopAll();
}

// calibration phase when it first turns on
void startupAmbientCalibration() {
  unsigned long start = millis();
  long sumL = 0, sumR = 0;
  int samples = 0;

  while (millis() - start < CALIB_MS) {
    sumL += (1023 - analogRead(LDR_LEFT_PIN));
    sumR += (1023 - analogRead(LDR_RIGHT_PIN));
    samples++;
    delay(15);
  }
  ambL = (float)sumL / samples;
  ambR = (float)sumR / samples;
  emaL = ambL;
  emaR = ambR;
}

// adaptive reading for the light sensors
void readLDRsAdaptive(int &bL, int &bR, int &relL, int &relR) {
  float rawL = analogRead(LDR_LEFT_PIN);
  float rawR = analogRead(LDR_RIGHT_PIN);

  emaL = LDR_ALPHA * (1023 - rawL) + (1 - LDR_ALPHA) * emaL;
  emaR = LDR_ALPHA * (1023 - rawR) + (1 - LDR_ALPHA) * emaR;

  bL = emaL;
  bR = emaR;

  relL = bL - ambL;
  relR = bR - ambR;

  // adjust ambient levels if it gets dark
  if ((relL + relR) < REL_DETECT_THRESHOLD) {
    ambL = LDR_AMBIENT_ALPHA * bL + (1 - LDR_AMBIENT_ALPHA) * ambL;
    ambR = LDR_AMBIENT_ALPHA * bR + (1 - LDR_AMBIENT_ALPHA) * ambR;
    relL = bL - ambL;
    relR = bR - ambR;
  }
}

// steers the robot toward the light source
void beaconSteer(int relL, int relR, int totalRel) {
  int error = relR - relL;

  // slow down if the light is really bright
  int base = BEACON_BASE_SPEED;
  if (totalRel > 450) base = 120;

  if (abs(error) <= CENTER_DEADZONE_REL) {
    moveForwardDirectPWM(base);
    return;
  }

  float steer = STEER_K * error;
  int leftPWM = constrain((int)(base - steer), 0, BEACON_PWM_MAX);
  int rightPWM = constrain((int)(base + steer), 0, BEACON_PWM_MAX);

  motorFL.setSpeed(leftPWM);
  motorRL.setSpeed(leftPWM);
  motorFR.setSpeed(rightPWM);
  motorRR.setSpeed(rightPWM);
  runAllLogical(FORWARD);
}

// gives a little push if it gets stuck trying to reach the light
void beaconApproachAssist(int totalRel) {
  if (lastTotalRel < 0) lastTotalRel = totalRel;
  if (totalRel <= lastTotalRel + 2)
    stallCount++;
  else
    stallCount = 0;

  lastTotalRel = totalRel;

  if (stallCount >= 3) {
    motorFL.setSpeed(BEACON_PWM_MAX);
    motorFR.setSpeed(BEACON_PWM_MAX);
    motorRL.setSpeed(BEACON_PWM_MAX);
    motorRR.setSpeed(BEACON_PWM_MAX);
    runAllLogical(FORWARD);
    delay(STALL_PUSH_MS);
    stopAll();
    stallCount = 0;
  }
}

// standard arduino setup function
void setup() {
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(EDGE_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  motorFL.setSpeed(BASE_SPEED);
  motorFR.setSpeed(BASE_SPEED);
  motorRL.setSpeed(BASE_SPEED);
  motorRR.setSpeed(BASE_SPEED);

  startupAmbientCalibration();
}

// main program loop
void loop() {
  // always check for cliffs first
  int edgeRaw = digitalRead(EDGE_PIN);
  int edgeDetected = EDGE_ACTIVE_LOW ? !edgeRaw : edgeRaw;
  if (edgeDetected) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    stopAll();
    backupAndSpin();
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
    return;
  }

  // read the obstacle sensors
  int L = digitalRead(IR_LEFT_PIN);
  int R = digitalRead(IR_RIGHT_PIN);
  if (IR_ACTIVE_LOW) {
    L = !L;
    R = !R;
  }

  // react if we see an obstacle
  if (L || R) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    stopAll();
    backupAndSpin();
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
    return;
  }

  // check light levels
  int bL, bR, relL, relR;
  readLDRsAdaptive(bL, bR, relL, relR);
  int totalRel = relL + relR;
  int diffRel = relR - relL;

  bool beaconDetected =
    (totalRel >= REL_DETECT_THRESHOLD) || (lastTotalRel >= 0 && (totalRel - lastTotalRel) >= RAPID_RISE_THRESHOLD);

  lastTotalRel = totalRel;

  // if we see the beacon follow it
  if (beaconDetected) {
    // blink fast so we know it sees the light
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(70);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(70);

    beaconSteer(relL, relR, totalRel);
    beaconApproachAssist(totalRel);

    // stop if we reached the goal
    if (totalRel >= GOAL_REL_THRESHOLD && abs(diffRel) <= CENTER_DEADZONE_REL) {
      stopAll();
      digitalWrite(STATUS_LED_PIN, HIGH);
      while (true) { delay(500); }
    }
    return;
  }

  // otherwise just wander around slowly
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(120);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(120);

  wanderStep();

  // occasional random spin just in case
  if (random(100) < 25) {
    if (random(2) == 0) spinLeft();
    else spinRight();
  }

  delay(20);
}
