
#pragma once

// ----------------------------
// PurrConfig.h — Edit per bot
// ----------------------------
// Root values are the starting point for OSC tuning.
// Per-pad values scale relative to these roots, so tuning
// the root shifts all 12 pads together while preserving
// their individual characters.

// Root rate:      1.0 = default musical spacing (0.50–1.67 Hz)
//                 0.5 = everything twice as slow
//                 2.0 = everything twice as fast
static const float purrRootRate      = 1.0f;

// Root intensity: 1.0 = config values as-is (0–127 RTP range)
//                 0.5 = half amplitude across all pads
//                 2.0 = double amplitude (clamped to 127)
static const float purrRootIntensity = 1.0f;


// ----------------------------
// Per-Pad Config Table
// ----------------------------
// rateMultiplier : Hz when rootRate = 1.0
//                  Exponentially spaced: 0.50 Hz (slowest) → 1.67 Hz (fastest)
//                  Musical interval ratio across 12 pads (~1/11th octave steps)
//
// intensityMin   : RTP floor amplitude while pad is held (0–127)
// intensityMax   : RTP peak amplitude while pad is held (0–127)
//
// Both min and max are multiplied by rootIntensity at runtime.

struct PadPurrConfig {
  float   rateMultiplier;
  uint8_t intensityMin;
  uint8_t intensityMax;
};

static const PadPurrConfig padPurr[12] = {
  // pad   rate Hz   min  max   character
  { 0.50f,  15,  70 },   //  0  slowest, gentle baseline
  { 0.56f,  12,  75 },   //  1  slightly soft floor
  { 0.62f,  18,  80 },   //  2  medium floor, wider range
  { 0.69f,  10,  65 },   //  3  quietest, subtle
  { 0.77f,  20,  85 },   //  4  stronger floor, good presence
  { 0.86f,  15,  90 },   //  5  widest dynamic swing
  { 0.96f,  12,  72 },   //  6  near-middle, lighter top
  { 1.07f,  22,  88 },   //  7  higher floor, assertive
  { 1.19f,  18,  78 },   //  8  balanced mid-fast
  { 1.33f,  14,  82 },   //  9  brighter, longer swing
  { 1.48f,  20,  92 },   // 10  near-fastest, strong peak
  { 1.67f,  16,  85 },   // 11  fastest, bright and lively
};
