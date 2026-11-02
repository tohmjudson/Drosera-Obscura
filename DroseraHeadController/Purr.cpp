
#include "Purr.h"
#include "PurrConfig.h"

// ----------------------------
// Internal State
// ----------------------------
static Adafruit_DRV2605* _drv       = nullptr;
static PurrState          _state     = PURR_IDLE;
static int                _activePad = -1;
static float              _phase     = 0.0f;   // radians, 0–2π
static unsigned long      _lastTick  = 0;

// Runtime-tunable roots (OSC can adjust these)
static float _rootRate      = purrRootRate;
static float _rootIntensity = purrRootIntensity;

// Previous touch mask — used to detect changes
static uint16_t _lastMask = 0;


// ----------------------------
// Helpers
// ----------------------------

// Count the number of set bits in a 16-bit mask
static int countBits(uint16_t mask) {
  int count = 0;
  while (mask) { count += (mask & 1); mask >>= 1; }
  return count;
}

// Return the index of the lowest set bit (-1 if none)
static int lowestBit(uint16_t mask) {
  for (int i = 0; i < 12; i++) {
    if (mask & (1 << i)) return i;
  }
  return -1;
}

// Switch DRV2605 to RTP mode
static void enterRTP() {
  _drv->setMode(DRV2605_MODE_REALTIME);
  _drv->setRealtimeValue(0);
}

// Switch DRV2605 back to internal trigger (one-shot) mode
static void enterINTTRIG() {
  _drv->setRealtimeValue(0);
  _drv->setMode(DRV2605_MODE_INTTRIG);
}

// Compute the current RTP amplitude from the LFO phase and pad config
static uint8_t computeAmplitude(int pad) {
  if (pad < 0 || pad > 11) return 0;

  float scaledMin = padPurr[pad].intensityMin * _rootIntensity;
  float scaledMax = padPurr[pad].intensityMax * _rootIntensity;

  scaledMin = constrain(scaledMin, 0.0f, 127.0f);
  scaledMax = constrain(scaledMax, 0.0f, 127.0f);

  // Sine ranges -1 to +1, remap to 0–1 for amplitude envelope
  float sine = (1.0f + sinf(_phase)) * 0.5f;

  float amp = scaledMin + (scaledMax - scaledMin) * sine;
  return (uint8_t)constrain(amp, 0.0f, 127.0f);
}

// Advance phase by elapsed time (dt in seconds) for the given pad
static void advancePhase(int pad, float dt) {
  if (pad < 0 || pad > 11) return;
  float hz = padPurr[pad].rateMultiplier * _rootRate;
  _phase += hz * 2.0f * (float)M_PI * dt;

  // Wrap to 0–2π
  while (_phase > 2.0f * (float)M_PI) {
    _phase -= 2.0f * (float)M_PI;
  }
}


// ----------------------------
// Public API
// ----------------------------

void purrInit(Adafruit_DRV2605* drvPtr) {
  _drv   = drvPtr;
  _state = PURR_IDLE;
  _phase = -(float)M_PI / 2.0f;  // start at sine minimum (natural resting point)
  _lastTick = millis();
}

void purrSetRootRate(float rate) {
  _rootRate = max(0.1f, rate);
}

void purrSetRootIntensity(float intensity) {
  _rootIntensity = max(0.0f, intensity);
}

PurrState purrGetState()     { return _state; }
int       purrGetActivePad() { return _activePad; }


void purrSetTouched(uint16_t touchMask) {
  if (touchMask == _lastMask) return;  // no change
  _lastMask = touchMask;

  int touchCount = countBits(touchMask);

  switch (_state) {

    case PURR_IDLE:
      if (touchCount == 1) {
        // Single pad touched — start purring
        _activePad = lowestBit(touchMask);
        _phase     = -(float)M_PI / 2.0f;  // begin at exhale (sine minimum)
        enterRTP();
        _state = PURR_ACTIVE;

        #ifdef DEBUG_VERBOSE
        Serial.printf("PURR: start pad %d\n", _activePad);
        #endif
      }
      // Multi-touch from idle → ignore
      break;

    case PURR_ACTIVE:
      if (touchCount == 0) {
        // Released — begin exhale-to-finish
        _state = PURR_RELEASE;

        #ifdef DEBUG_VERBOSE
        Serial.printf("PURR: release pad %d, finishing breath\n", _activePad);
        #endif
      }
      // Multi-touch while active → freeze, do nothing
      // Single pad swap while still held is also ignored (finger must fully lift)
      break;

    case PURR_RELEASE:
      if (touchCount == 1) {
        // Re-touched before exhale finished — interrupt release, start new pad
        _activePad = lowestBit(touchMask);
        _phase     = -(float)M_PI / 2.0f;
        // Already in RTP mode, no need to switch
        _state = PURR_ACTIVE;

        #ifdef DEBUG_VERBOSE
        Serial.printf("PURR: re-touch pad %d during release\n", _activePad);
        #endif
      }
      // Multi-touch during release → ignore
      break;
  }
}


void purrUpdate() {
  if (!_drv) return;

  unsigned long now = millis();
  float dt = (now - _lastTick) / 1000.0f;
  _lastTick = now;

  // Clamp dt to avoid large jumps after pauses
  if (dt > 0.1f) dt = 0.1f;

  switch (_state) {

    case PURR_IDLE:
      // Nothing to do
      break;

    case PURR_ACTIVE:
      advancePhase(_activePad, dt);
      _drv->setRealtimeValue(computeAmplitude(_activePad));
      break;

    case PURR_RELEASE:
      // Continue LFO until we naturally reach the sine minimum (~bottom of exhale)
      // Minimum is when sin(phase) ≤ -0.95, i.e. phase ≈ 3π/2
      advancePhase(_activePad, dt);
      {
        float sineVal = sinf(_phase);

        if (sineVal <= -0.95f) {
          // Exhale complete — cut and return to one-shot mode
          _drv->setRealtimeValue(0);
          enterINTTRIG();
          _state     = PURR_IDLE;
          _activePad = -1;

          #ifdef DEBUG_VERBOSE
          Serial.println("PURR: exhale done, IDLE");
          #endif
        } else {
          // Keep breathing out
          _drv->setRealtimeValue(computeAmplitude(_activePad));
        }
      }
      break;
  }
}
