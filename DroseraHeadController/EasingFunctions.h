#pragma once
#include <math.h>

// ----------------------------
// Easing Function Selector
// ----------------------------
enum EasingType {
  EASE_LINEAR,
  EASE_IN_CUBIC,
  EASE_OUT_CUBIC,
  EASE_INOUT_CUBIC,
  EASE_INOUT_SINE,
  EASE_INOUT_QUAD,
  EASE_OUT_ELASTIC,
  EASE_INOUT_EXPO
};

// ----------------------------
// t is normalized time: 0.0 = start, 1.0 = end
// All functions return a mapped t in [0.0, 1.0]
// Apply to position: pos = start + (target - start) * ease(t)
// ----------------------------

inline float easeLinear(float t) {
  return t;
}

inline float easeInCubic(float t) {
  return t * t * t;
}

inline float easeOutCubic(float t) {
  float f = t - 1.0f;
  return f * f * f + 1.0f;
}

inline float easeInOutCubic(float t) {
  if (t < 0.5f) {
    return 4.0f * t * t * t;
  } else {
    float f = (2.0f * t) - 2.0f;
    return 0.5f * f * f * f + 1.0f;
  }
}

inline float easeInOutSine(float t) {
  return 0.5f * (1.0f - cosf(t * M_PI));
}

inline float easeInOutQuad(float t) {
  if (t < 0.5f) {
    return 2.0f * t * t;
  } else {
    return 1.0f - 0.5f * (2.0f - 2.0f * t) * (2.0f - 2.0f * t);
  }
}

inline float easeOutElastic(float t) {
  if (t == 0.0f || t == 1.0f) return t;
  float p = 0.3f;
  return powf(2.0f, -10.0f * t) * sinf((t - p / 4.0f) * (2.0f * M_PI) / p) + 1.0f;
}

inline float easeInOutExpo(float t) {
  if (t == 0.0f) return 0.0f;
  if (t == 1.0f) return 1.0f;
  if (t < 0.5f) {
    return 0.5f * powf(2.0f, (20.0f * t) - 10.0f);
  } else {
    return 1.0f - 0.5f * powf(2.0f, (-20.0f * t) + 10.0f);
  }
}

// ----------------------------
// Dispatcher — call with EasingType
// ----------------------------
inline float applyEasing(float t, EasingType type) {
  t = constrain(t, 0.0f, 1.0f);
  switch (type) {
    case EASE_LINEAR:       return easeLinear(t);
    case EASE_IN_CUBIC:     return easeInCubic(t);
    case EASE_OUT_CUBIC:    return easeOutCubic(t);
    case EASE_INOUT_CUBIC:  return easeInOutCubic(t);
    case EASE_INOUT_SINE:   return easeInOutSine(t);
    case EASE_INOUT_QUAD:   return easeInOutQuad(t);
    case EASE_OUT_ELASTIC:  return easeOutElastic(t);
    case EASE_INOUT_EXPO:   return easeInOutExpo(t);
    default:                return easeLinear(t);
  }
}
