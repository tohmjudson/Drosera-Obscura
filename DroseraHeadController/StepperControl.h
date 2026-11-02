#pragma once
#include <AccelStepper.h>

// ----------------------------
// Pin Definitions
// ----------------------------
#define PAN_STEP_PIN   17
#define PAN_DIR_PIN    18
#define TILT_STEP_PIN  14
#define TILT_DIR_PIN   12

// ----------------------------
// Microstepping Config
// MS0=HIGH, MS1=LOW, MS2=HIGH → 1/32 step (hardwired on DRV8825)
// NEMA-17: 200 steps/rev × 32 = 6400 microsteps/rev
// ----------------------------
const float STEPS_PER_REV  = 6400.0f;
const float STEPS_PER_DEG  = STEPS_PER_REV / 360.0f;  // 17.778 microsteps/degree

// ----------------------------
// Home Angle
// ----------------------------
const float STEPPER_HOME_ANGLE = 90.0f;

// ----------------------------
// Independent Pan/Tilt Angle Limits
// Default full range — expanded to ±360 for multi-turn support
// ----------------------------
static float _panMinAngle  = 0.0f;
static float _panMaxAngle  = 360.0f;
static float _tiltMinAngle = 0.0f;
static float _tiltMaxAngle = 360.0f;

// ----------------------------
// Speed & Acceleration
// 6400 microsteps/sec = 1 full rev/sec at 1/32
// ----------------------------
const float PAN_MAX_SPEED   = 6400.0f;
const float PAN_ACCEL       = 3200.0f;
const float TILT_MAX_SPEED  = 6400.0f;
const float TILT_ACCEL      = 3200.0f;

// ----------------------------
// AccelStepper Instances
// DRIVER mode: uses STEP + DIR pins only
// ----------------------------
static AccelStepper _panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
static AccelStepper _tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// ----------------------------
// Current tracked angles
// ----------------------------
static float _panAngle  = STEPPER_HOME_ANGLE;
static float _tiltAngle = STEPPER_HOME_ANGLE;

// ----------------------------
// Angle to Steps Converter
// ----------------------------
long angleToSteps(float angle) {
  return (long)((angle - STEPPER_HOME_ANGLE) * STEPS_PER_DEG);
}

// ----------------------------
// Init — call once in setup()
// ----------------------------
void stepperInit() {
  _panStepper.setMaxSpeed(PAN_MAX_SPEED);
  _panStepper.setAcceleration(PAN_ACCEL);
  _panStepper.setCurrentPosition(0);

  _tiltStepper.setMaxSpeed(TILT_MAX_SPEED);
  _tiltStepper.setAcceleration(TILT_ACCEL);
  _tiltStepper.setCurrentPosition(0);

  Serial.println("StepperControl: initialized");
  Serial.printf("StepperControl: %.2f microsteps/degree\n", STEPS_PER_DEG);
  Serial.printf("StepperControl: pan speed=%.0f accel=%.0f\n", PAN_MAX_SPEED, PAN_ACCEL);
  Serial.printf("StepperControl: tilt speed=%.0f accel=%.0f\n", TILT_MAX_SPEED, TILT_ACCEL);
  Serial.printf("StepperControl: pan limits=%.0f-%.0f  tilt limits=%.0f-%.0f\n",
    _panMinAngle, _panMaxAngle, _tiltMinAngle, _tiltMaxAngle);
}

// ----------------------------
// Move to Angle — absolute position
// Call ONLY when target changes — do NOT call every loop tick
// ----------------------------
void stepperMoveToAngle(float panAngle, float tiltAngle) {
  panAngle  = constrain(panAngle,  _panMinAngle,  _panMaxAngle);
  tiltAngle = constrain(tiltAngle, _tiltMinAngle, _tiltMaxAngle);

  _panAngle  = panAngle;
  _tiltAngle = tiltAngle;

  _panStepper.moveTo(angleToSteps(panAngle));
  _tiltStepper.moveTo(angleToSteps(tiltAngle));

  #ifdef DEBUG_VERBOSE
  Serial.printf("stepperMoveToAngle: pan=%.1f tilt=%.1f steps: pan=%ld tilt=%ld\n",
    panAngle, tiltAngle,
    angleToSteps(panAngle), angleToSteps(tiltAngle));
  #endif
}

// ----------------------------
// Update — call every loop tick AS FAST AS POSSIBLE
// ----------------------------
void stepperUpdate() {
  _panStepper.run();
  _tiltStepper.run();
}

// ----------------------------
// Getters
// ----------------------------
float stepperGetPanAngle()  { return _panAngle;  }
float stepperGetTiltAngle() { return _tiltAngle; }

float stepperGetPanMin()  { return _panMinAngle;  }
float stepperGetPanMax()  { return _panMaxAngle;  }
float stepperGetTiltMin() { return _tiltMinAngle; }
float stepperGetTiltMax() { return _tiltMaxAngle; }

long stepperGetPanSteps()  { return _panStepper.currentPosition();  }
long stepperGetTiltSteps() { return _tiltStepper.currentPosition(); }

bool stepperIsMoving() {
  return (_panStepper.distanceToGo() != 0 || _tiltStepper.distanceToGo() != 0);
}

// ----------------------------
// Set Speed — live tuning via OSC
// /robot/stepspeed  panSpeed  tiltSpeed
// ----------------------------
void stepperSetSpeed(float panSpeed, float tiltSpeed) {
  panSpeed  = constrain(panSpeed,  100.0f, 12800.0f);
  tiltSpeed = constrain(tiltSpeed, 100.0f, 12800.0f);
  _panStepper.setMaxSpeed(panSpeed);
  _tiltStepper.setMaxSpeed(tiltSpeed);
  Serial.printf("StepperControl: speed -> pan=%.0f tilt=%.0f\n", panSpeed, tiltSpeed);
}

// ----------------------------
// Set Acceleration — live tuning via OSC
// /robot/stepaccel  panAccel  tiltAccel
// ----------------------------
void stepperSetAccel(float panAccel, float tiltAccel) {
  panAccel  = constrain(panAccel,  100.0f, 6400.0f);
  tiltAccel = constrain(tiltAccel, 100.0f, 6400.0f);
  _panStepper.setAcceleration(panAccel);
  _tiltStepper.setAcceleration(tiltAccel);
  Serial.printf("StepperControl: accel -> pan=%.0f tilt=%.0f\n", panAccel, tiltAccel);
}

// ----------------------------
// Set Pan Limits — expanded to ±360 for multi-turn support
// /robot/steplimits/pan  minAngle  maxAngle
// ----------------------------
void stepperSetPanLimits(float minAngle, float maxAngle) {
  _panMinAngle = constrain(minAngle, -360.0f, 360.0f);
  _panMaxAngle = constrain(maxAngle, -360.0f, 360.0f);
  if (_panMinAngle > _panMaxAngle) {
    float tmp = _panMinAngle;
    _panMinAngle = _panMaxAngle;
    _panMaxAngle = tmp;
  }
  Serial.printf("StepperControl: pan limits -> %.0f-%.0f\n", _panMinAngle, _panMaxAngle);
}

// ----------------------------
// Set Tilt Limits — expanded to ±360 for multi-turn support
// /robot/steplimits/tilt  minAngle  maxAngle
// ----------------------------
void stepperSetTiltLimits(float minAngle, float maxAngle) {
  _tiltMinAngle = constrain(minAngle, -360.0f, 360.0f);
  _tiltMaxAngle = constrain(maxAngle, -360.0f, 360.0f);
  if (_tiltMinAngle > _tiltMaxAngle) {
    float tmp = _tiltMinAngle;
    _tiltMinAngle = _tiltMaxAngle;
    _tiltMaxAngle = tmp;
  }
  Serial.printf("StepperControl: tilt limits -> %.0f-%.0f\n", _tiltMinAngle, _tiltMaxAngle);
}

// ----------------------------
// Stop — immediate halt, holds position
// ----------------------------
void stepperStop() {
  _panStepper.stop();
  _tiltStepper.stop();
  Serial.println("StepperControl: stop");
}
