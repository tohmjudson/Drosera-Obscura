#pragma once
#include "FastNoiseLite.h"

// ----------------------------
// Auto Mode — Organic Motion
// Uses FastNoise Lite with Domain Warping
// Two independent noise tracks:
//   - Servo 1 (pan)
//   - Servo 2 (tilt)
// ----------------------------

// ----------------------------
// Servo Parameters
// ----------------------------
static float autoSpeed       = 0.03f;   // time advance (shared by all axes)
static float autoRange       = 35.0f;   // servo wander ±degrees
static float autoPanCenter   = 90.0f;   // servo pan center
static float autoTiltCenter  = 90.0f;   // servo tilt center
static float autoPanMin      = 20.0f;
static float autoPanMax      = 160.0f;
static float autoTiltMin     = 20.0f;
static float autoTiltMax     = 160.0f;
static float warpStrength    = 40.0f;   // domain warp strength (shared)

// ----------------------------
// Noise offset constants — keep axes independent
// ----------------------------
static const float TILT_OFFSET    = 100.0f;   // servo tilt offset

// ----------------------------
// Internal State
// ----------------------------
static FastNoiseLite _noise;
static FastNoiseLite _warpNoise;
static bool _autoInitialized = false;
static float _autoTime = 0.0f;

// ----------------------------
// Init — call once when entering AUTO mode
// ----------------------------
void autoModeInit() {
  if (!_autoInitialized) {
    _noise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
    _noise.SetFrequency(0.8f);
    _noise.SetSeed(42);

    _warpNoise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
    _warpNoise.SetFrequency(0.5f);
    _warpNoise.SetSeed(137);

    _autoInitialized = true;

    Serial.println("AutoMode: initialized (2 axes)");
    Serial.printf("AutoMode: speed=%.3f servoRange=%.1f warp=%.1f\n",
                  autoSpeed, autoRange, warpStrength);
  }
}

// ----------------------------
// Reset time — call when re-entering AUTO mode
// ----------------------------
void autoModeReset() {
  _autoTime = 0.0f;
  Serial.println("AutoMode: reset");
}

// ----------------------------
// Core Update — call every control loop tick
// Returns both servo axis targets via out params
// ----------------------------
void autoModeUpdate(float &outServoPan, float &outServoTilt) {

  _autoTime += autoSpeed;

  // --- Domain warp offsets for each axis ---
  float warpPan  = _warpNoise.GetNoise(_autoTime,               0.0f) * warpStrength;
  float warpTilt = _warpNoise.GetNoise(_autoTime + TILT_OFFSET, 0.0f) * warpStrength;

  // --- Primary noise samples ---
  float nServoPan  = _noise.GetNoise(_autoTime + warpPan,                0.0f);
  float nServoTilt = _noise.GetNoise(_autoTime + warpTilt + TILT_OFFSET, 0.0f);

  // --- Map servo outputs ---
  outServoPan  = autoPanCenter  + nServoPan  * autoRange;
  outServoTilt = autoTiltCenter + nServoTilt * autoRange;
  outServoPan  = constrain(outServoPan,  autoPanMin,  autoPanMax);
  outServoTilt = constrain(outServoTilt, autoTiltMin, autoTiltMax);
}

// ----------------------------
// OSC Parameter Setters — Servo
// ----------------------------
void autoSetSpeed(float speed) {
  autoSpeed = constrain(speed, 0.001f, 0.5f);
  Serial.printf("AutoMode: speed -> %.4f\n", autoSpeed);
}

void autoSetRange(float range) {
  autoRange = constrain(range, 5.0f, 70.0f);
  Serial.printf("AutoMode: servo range -> %.1f\n", autoRange);
}

void autoSetWarp(float warp) {
  warpStrength = constrain(warp, 0.0f, 100.0f);
  Serial.printf("AutoMode: warp -> %.1f\n", warpStrength);
}

void autoSetCenter(float pan, float tilt) {
  autoPanCenter  = constrain(pan,  0.0f, 180.0f);
  autoTiltCenter = constrain(tilt, 0.0f, 180.0f);
  Serial.printf("AutoMode: servo center -> pan=%.1f tilt=%.1f\n", autoPanCenter, autoTiltCenter);
}
