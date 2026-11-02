
#pragma once

#include <Arduino.h>
#include <Adafruit_DRV2605.h>

// ----------------------------
// Purr System
// ----------------------------
// Call purrInit() once in setup() after drv.begin().
// Call purrUpdate() every control loop tick (~10ms).
// Call purrSetPad() from handleTouch() on touch/release events.
// Call purrSetRootRate() / purrSetRootIntensity() from OSC handlers.

// Purr state machine
enum PurrState {
  PURR_IDLE,      // not touching — chip in INTTRIG, one-shots work normally
  PURR_ACTIVE,    // single pad held — chip in RTP, LFO running
  PURR_RELEASE    // pad released — LFO continues until natural exhale minimum
};

// Initialize the purr system (call after drv.begin())
void purrInit(Adafruit_DRV2605* drvPtr);

// Call every control loop tick (~10ms)
void purrUpdate();

// Call from handleTouch() with current touch bitmask
// Pass 0 when all pads released
void purrSetTouched(uint16_t touchMask);

// OSC tuning — shift root values at runtime
void purrSetRootRate(float rate);
void purrSetRootIntensity(float intensity);

// Query current state (useful for debug / OLED)
PurrState purrGetState();
int       purrGetActivePad();
