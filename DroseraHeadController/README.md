# Drosera Head Controller

Firmware for the Drosera animatronic head: an ESP32-S3 (Unexpected Maker FeatherS3)
controller that drives 8 servos and 12 LEDs via two PCA9685 boards, reads 12-pad
capacitive touch (MPR121), plays "purring" haptic breath effects on 4 DRV2605
drivers behind an I2C mux, talks to a Daisy Seed audio processor over UART, and
is remote-controlled over OSC (UDP + Serial) with an SSD1306 status display.

---

## 1. Quick Start (first-time setup)

### 1.1 Install the toolchain
- **Arduino IDE** (2.x) or `arduino-cli`.
- **Board support**: ESP32 (Espressif) board package. This project targets an
  **Unexpected Maker FeatherS3** (ESP32-S3). If you don't have the UM board
  definitions installed, add their board manager URL, or select a generic
  **ESP32S3 Dev Module** — but you must still install the `UMS3` helper
  library (below) for the onboard NeoPixel/button helpers to compile.

### 1.2 Install required libraries
Install via Library Manager (or `arduino-cli lib install`):

| Library | Used for |
|---|---|
| `Adafruit PWM Servo Driver Library` | Both PCA9685 boards (servos + LEDs) |
| `Adafruit MPR121` | 12-pad capacitive touch |
| `Adafruit DRV2605 Library` | Haptic motor drivers |
| `Adafruit GFX Library` | OLED graphics primitives |
| `Adafruit SSD1306` | OLED display driver |
| `OSC` (by CNMAT) | OSC message parsing/building (`OSCMessage.h`) |
| `UMS3` | Unexpected Maker FeatherS3 helpers (onboard RGB LED, power) |

ESP32 core libraries used (no install needed, ship with the ESP32 board package):
`WiFi.h`, `WiFiUdp.h`, `ESPmDNS.h`, `Wire.h`.

### 1.3 Project file layout
Put every file below in **one sketch folder**, named to match the `.ino` file
exactly (Arduino requirement):

```
DroseraHeadController/
├── DroseraHeadController.ino   # main sketch (rename to your actual filename)
├── Secrets.h
├── Secrets.cpp
├── EasingFunctions.h
├── AutoMode.h
├── Purr.h
├── Purr.cpp
├── PurrConfig.h
└── FastNoiseLite.h             # third-party, MIT license (Auburn/FastNoiseLite)
```

> If your actual `.ino` has a different name, rename the folder to match —
> the Arduino IDE will refuse to open it otherwise.

### 1.4 Configure secrets
`Secrets.h` / `Secrets.cpp` are **not committed with real values** — fill in
your own before building:

```cpp
// Secrets.h
#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"

inline constexpr char ROBOT_NAME[] = "robot-name";  // mDNS name -> bot02.local
inline constexpr uint16_t LOCAL_PORT = 8888;        // this device listens here
inline constexpr uint16_t MAX_PORT   = 9999;        // status telemetry destination port

extern IPAddress maxIP;
```

```cpp
// Secrets.cpp
#include "Secrets.h"
IPAddress maxIP(0, 0, 0, 0);   // <-- set to your Max/Pd/controller IP
```

Keep `Secrets.*` out of version control (add to `.gitignore`) since it holds
Wi-Fi credentials.

### 1.5 Build & flash
1. Select the correct board + port in the IDE.
2. Upload at 115200 baud.
3. Open the Serial Monitor at **115200**. On boot you should see:
   - an I2C scan listing found addresses,
   - OLED init confirmation,
   - Wi-Fi connect result,
   - PCA9685 init + ALLCALL-disable confirmation for both `0x40` and `0x60`,
   - haptic channel health (`X/4 found`).
4. Once Wi-Fi connects, the OLED shows the device's IP and `<ROBOT_NAME>.local`.

### 1.6 Talk to it
- Send OSC over UDP to the device's IP (or `<ROBOT_NAME>.local` via mDNS) on
  `LOCAL_PORT` (8888 by default).
- Or send the same commands as plain text lines over USB Serial (115200) —
  see §4 for the line format.
- The device sends status/telemetry OSC back to `maxIP:MAX_PORT`.

---

## 2. Hardware Overview

| Subsystem | Device | Address / Pins |
|---|---|---|
| MCU | Unexpected Maker FeatherS3 (ESP32-S3) | — |
| I2C bus | Wire, 100 kHz | SDA = GPIO8, SCL = GPIO9 |
| OLED | SSD1306 128×64 | I2C `0x3C` |
| Servo driver | PCA9685 | I2C `0x40`, logical servos 0–7 → PCA channels 8–15 |
| LED driver | PCA9685 | I2C `0x60`, 12 channels, PWM freq 1 kHz, schematic-mapped |
| Touch | MPR121 (12 pads) | I2C `0x5C` |
| Haptic mux | TCA9546A | I2C `0x70`, 4 downstream channels |
| Haptics | 4× DRV2605 | I2C `0x5A` (behind mux, one per channel) |
| PWM global enable | GPIO7 | HIGH = outputs disabled, LOW = enabled (held HIGH until both PCA9685 boards are configured) |
| Button | GPIO0 | Active-low, debounced, cycles `/test` color on press |
| Daisy Seed UART | `HardwareSerial(2)` | RX = GPIO18, TX = GPIO17, 115200 baud |

Servo pulse range: 500–2400 µs, 50 Hz. LED PWM: 12-bit, 1 kHz, active-high
(`LED_OUTPUT_INVERTED = false`).

---

## 3. File Reference

| File | Role |
|---|---|
| **`DroseraHeadController.ino`** | Main sketch: setup/loop, Wi-Fi/UDP/mDNS, OSC + Serial command dispatch, servo/LED hardware control, mode state machine (DIRECT/EASE/AUTO), Daisy UART link, telemetry, OLED screens. |
| **`Secrets.h` / `Secrets.cpp`** | Wi-Fi credentials, robot name, UDP ports, and the `maxIP` destination for outbound telemetry. Not meant to be shared/committed with real values. |
| **`EasingFunctions.h`** | Header-only `EasingType` enum + easing curve math (`linear`, cubic, sine, quad, elastic, expo) and the `applyEasing(t, type)` dispatcher. Used by EASE mode and LED fades. No dependencies beyond `<math.h>`. |
| **`AutoMode.h`** | Header-only "organic motion" generator for AUTO mode. Uses `FastNoiseLite` (Perlin noise + domain warping) to drive 2 independent axes (servo pan, servo tilt) with musically-offset noise tracks. Exposes OSC-facing setters (`autoSetSpeed`, `autoSetRange`, `autoSetWarp`, `autoSetCenter`). Self-contained aside from `FastNoiseLite.h`. |
| **`FastNoiseLite.h`** | Third-party, single-header Perlin/Simplex/Cellular noise + domain-warp library (MIT, © Jordan Peck / Auburn). Unmodified vendor code — treat as a dependency, not project logic. |
| **`Purr.h` / `Purr.cpp`** | "Purr" haptic breathing state machine (`PURR_IDLE` → `PURR_ACTIVE` → `PURR_RELEASE`). Drives one DRV2605 in real-time-playback (RTP) mode with a sine LFO so a held touch pad "breathes," and hands control back to one-shot (`INTTRIG`) mode when idle. Per-pad character comes from `PurrConfig.h`; root rate/intensity are runtime-tunable via OSC. |
| **`PurrConfig.h`** | Per-robot tuning table: `purrRootRate`, `purrRootIntensity`, and a 12-entry `padPurr[]` array (`rateMultiplier`, `intensityMin`, `intensityMax`) giving each touch pad its own breathing character. Edit this per physical build. |

---

## 4. Command Interface (OSC + Serial)

Every address below works identically whether sent as a **UDP OSC message**
to `LOCAL_PORT`, or as a **plain Serial line** `"<address> <args...>"`
(e.g. `/robot/mode 2\n`) at 115200 baud. Serial lines are capped at 255 bytes
and silently discarded if longer; Daisy UART lines are capped at 63 bytes.
UDP packets over 512 bytes are rejected outright.

### 4.1 Servos & motion modes
| Address | Args | Notes | Example |
|---|---|---|---|
| `/robot/servos` | 8 floats | Target angle per channel (clamped to that channel's limits); only applied in DIRECT mode | `/robot/servos 90 90 45 120 90 90 90 90` |
| `/robot/servos/count` | int | Active servo count 0–8; rejected if it would *increase* during a mode-transition centering move | `/robot/servos/count 4` |
| `/robot/servolimits/0` … `/7` | min, max | Per-channel angle limits (0–180, auto-swapped if reversed) | `/robot/servolimits/2 20 160` |
| `/robot/mode` | int 0/1/2 | `0`=DIRECT, `1`=EASE, `2`=AUTO. Always transitions through a 2.5 s safe-pose centering move first | `/robot/mode 2` |
| `/robot/ease` | pan, tilt, durationMs | Eased move of servo channels 0/1; auto-switches into EASE mode first if needed (resend after centering) | `/robot/ease 60 120 1500` |
| `/robot/easemode` | int 0–7 | Selects the `EasingType` used for `/robot/ease` | `/robot/easemode 4` |
| `/speed` | float (0–100-ish) | Global servo speed scalar (`/100`, floor 0.1) | `/speed 75` |

### 4.2 Auto mode tuning
| Address | Args | Example |
|---|---|---|
| `/robot/auto/speed` | float — noise time-advance per tick | `/robot/auto/speed 0.05` |
| `/robot/auto/range` | float — servo wander ± degrees | `/robot/auto/range 40` |
| `/robot/auto/warp` | float — domain warp strength | `/robot/auto/warp 25` |
| `/robot/auto/center` | pan, tilt — servo auto-mode center point | `/robot/auto/center 90 100` |

### 4.3 LEDs
| Address | Args | Example |
|---|---|---|
| `/robot/led` | channel (0–11), level (0–4095) — instant | `/robot/led 3 4095` |
| `/robot/led/fade` | channel, target, durationMs | `/robot/led/fade 3 0 800` |
| `/robot/led/fadeall` | target, durationMs — all 12 channels | `/robot/led/fadeall 2000 1200` |
| `/robot/led/easemode` | int 0–7 — easing curve for fades | `/robot/led/easemode 2` |

### 4.4 Haptics / Purr
| Address | Args | Example |
|---|---|---|
| `/haptic` | effect (1–123) — plays on all available DRV2605 channels | `/haptic 14` |
| `/haptic` | channel (0–3), effect (1–123) — plays on one channel | `/haptic 1 47` |
| `/robot/purr/rate` | float — root breathing rate multiplier | `/robot/purr/rate 1.2` |
| `/robot/purr/intensity` | float — root breathing intensity multiplier | `/robot/purr/intensity 0.8` |

### 4.5 Display & misc
| Address | Args | Example |
|---|---|---|
| `/robot/display` | int 0–5 — OLED screen (System / Servos / Touch / Net Debug / Daisy CPU / Health) | `/robot/display 1` |
| `/test` | int 0–3 — flashes onboard RGB LED one of 4 test colors | `/test 2` |

### 4.6 Daisy Seed passthrough
| Address | Args | Example |
|---|---|---|
| `/daisy` | parameterId (0–99), value — forwarded as `"<id> <value>\n"` over UART, rate-limited to ~20 Hz per id (a few ids bypass the throttle) | `/daisy 17 1` |
| `/daisy/synth` | parameterId, value — same, with `+10` offset applied to the id | `/daisy/synth 3 64` |

### 4.7 Outbound telemetry (device → `maxIP:MAX_PORT`)
All prefixed with `/<ROBOT_NAME>/...`:

| Address | Interval | Payload |
|---|---|---|
| `/status/system` | 1 s | uptime (s), chip temp (°C), CPU MHz |
| `/status/memory` | 2 s | free heap, min free heap, max alloc |
| `/status/network` | 500 ms | RSSI, last OSC seq |
| `/status/servos` | 100 ms | 8 ints, rounded angles |
| `/status/daisy` | 200 ms | link ok, rx age (ms), cpu load %, rx packets, parse errors, last error code, audio level |
| `/status/haptics` | 1 s | health bitmask + per-channel found flags |
| `/status/mode` | on event | event name, current mode, pending mode, transition active, progress %, servo count |
| `/status/ease` | on completion | "done", final pan, final tilt |
| `/touch` | on change | 12-bit touch bitmask |

---

## 5. Behavior Notes & Known Quirks

### 5.1 Boot sequence safety
PCA9685 outputs are held fully off (`PWM_ENABLE_PIN` driven HIGH) until *both*
PCA9685 boards are initialized, their ALLCALL bit is cleared (to prevent the
two boards from responding to each other's broadcast address), and all 16
channels on each are explicitly set to full-off. Only then is the enable pin
driven LOW. Individual servo channels stay at full-off until
`/robot/servos/count` raises `activeServoCount` above 0.

### 5.2 Mode transitions always "center" first
Switching between DIRECT / EASE / AUTO never jumps directly — it always runs
a non-blocking 2.5 s eased move of all active servos to their configured
`servoSafePosition[]` first (`beginModeTransition` → `updateModeTransition` →
`finishModeTransition`), broadcasting `/status/mode` at each stage. Re-issuing
`/robot/mode` during a transition just retargets the pending mode.

### 5.3 Network resilience
Wi-Fi loss is detected every loop via `updateWiFiLink()`; UDP + mDNS are torn
down on disconnect and automatically restarted on reconnect, with reconnect
attempts throttled to once per `WIFI_RECONNECT_INTERVAL_MS` (5 s). OSC sends
are silently dropped (not queued) while Wi-Fi/UDP is down.

### 5.4 Strict input validation
All numeric OSC/Serial commands use `sscanf`-based "strict" parsers that
reject trailing garbage, and duration arguments are bounded to
`MAX_COMMAND_DURATION_MS` (10 minutes). Malformed lines/packets are logged
to Serial and dropped rather than partially applied.

### 5.5 Purr vs. one-shot haptics
Playing a one-shot haptic effect (`/haptic`) on the Purr channel
(`PURR_HAPTIC_CHANNEL`, default channel 0) temporarily suspends the purring
LFO for `HAPTIC_ONESHOT_GUARD_MS` (1.5 s) so the two don't fight over the
DRV2605's playback mode.

---

## 6. Debugging Tips
- `DEBUG_VERBOSE` is defined by default — prints detailed logs for LED,
  haptic, mode-transition, and Daisy RX/TX events. Comment it out to quiet
  Serial output.
- `DEBUG_OSC_RAW` exists as a commented-out flag for future raw-packet dumps.
- OLED screen 5 ("SYSTEM HEALTH") is the fastest way to see Wi-Fi / touch /
  haptic / display / Daisy status at a glance; screen 3 shows live OSC-RX and
  Daisy-TX activity blips.
- If a PCA9685 reports `ALLCALL conflict was not fully cleared` on boot, check
  for an I2C address collision or wiring fault — both boards sharing the
  ALLCALL address (`0x70` on the PCA9685 side, not to be confused with the
  TCA9546A mux's own `0x70`) can cause cross-talk if not disabled.
