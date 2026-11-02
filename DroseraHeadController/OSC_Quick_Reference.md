# Drosera OSC Message Reference

This document describes the OSC interface for the current **Drosera Head Controller** firmware.

## Network configuration

| Item | Current value |
|---|---|
| Robot name | `bot02` |
| Robot hostname | `bot02.local` |
| Robot receive port | `8888` |
| Status destination | `10.0.0.178` |
| Status destination port | `9999` |

- **Sends** below are messages sent **to the robot**.
- **Receives** below are messages sent **from the robot** to the configured status destination.
- Unless otherwise noted, channels are zero-based.
- Numeric control arguments may generally be sent as OSC integers or floats.
- Every address below also works over the USB Serial console using the same address and space-separated arguments, with no leading slash-type formatting beyond the address itself (e.g. typing `/robot/mode 2` into the Serial Monitor behaves the same as sending it over UDP).

---

# Sends — OSC messages sent to the robot

## Operating mode and servo motion

### `/robot/mode <mode>`

Requests an operating mode.

| Mode | Meaning |
|---:|---|
| `0` | DIRECT |
| `1` | EASE |
| `2` | AUTO |

Example:

```text
/robot/mode 2
```

When changing modes, enabled servos first move to their configured safe positions over approximately 2.5 seconds. The requested mode becomes active after centering completes.

A new mode request received during centering changes the pending destination without restarting the centering timer.

---

### `/robot/servos <s0> <s1> <s2> <s3> <s4> <s5> <s6> <s7>`

Sets the eight stored DIRECT-mode servo targets in degrees.

```text
/robot/servos 90 90 90 90 90 90 90 90
```

```text
/robot/servos 120 60 90 90 90 90 90 90
```

| Argument | Meaning | Range |
|---|---|---:|
| `s0`–`s7` | Logical servo target angle | `0–180°`, further limited by each servo's configured limits |

Notes:

- The packet must contain all eight values.
- Targets are stored even when a servo is disabled.
- Only logical channels below `activeServoCount` can drive hardware.
- During a mode transition, incoming targets may be stored but are not applied.
- When a transition completes into DIRECT, active targets are reset to the safe pose. A subsequent servo packet is required to move away from it.

---

### `/robot/servos/count <count>`

Sets the number of enabled logical servo channels.

```text
/robot/servos/count 2
```

| Argument | Range | Meaning |
|---|---:|---|
| `count` | `0–8` | Enables logical servos `0` through `count - 1` |

Safety behavior:

- The controller boots with `count = 0`.
- Decreasing the count immediately sets removed PCA9685 outputs to full-off.
- Increasing the count energizes newly enabled servos at their configured safe positions.
- Count increases are rejected while mode centering is active.
- Count reductions remain allowed during centering.

---

### `/robot/ease <pan> <tilt> <durationMs>`

Starts an eased move for logical servos 0 and 1.

```text
/robot/ease 120 70 2000
```

| Argument | Meaning |
|---|---|
| `pan` | Target angle for logical servo 0 |
| `tilt` | Target angle for logical servo 1 |
| `durationMs` | Move duration in milliseconds |

The duration must be non-negative and no greater than the firmware's command-duration limit, currently 600,000 ms.

Important mode behavior:

- If EASE is already active, the move starts immediately.
- If another mode is active, the robot begins its safe transition into EASE but deliberately discards the requested target.
- Resend `/robot/ease` after the centering transition completes.

---

### `/robot/easemode <easing>`

Selects the easing curve used by `/robot/ease`.

| Value | Curve |
|---:|---|
| `0` | Linear |
| `1` | In cubic |
| `2` | Out cubic |
| `3` | In/out cubic |
| `4` | In/out sine |
| `5` | In/out quadratic |
| `6` | Out elastic |
| `7` | In/out exponential |

Example:

```text
/robot/easemode 4
```

---

### `/robot/servolimits/<channel> <minimum> <maximum>`

Sets the limits for one logical servo. Channels are zero-based from `0` through `7`.

```text
/robot/servolimits/0 30 150
/robot/servolimits/1 45 135
/robot/servolimits/7 20 160
```

| Argument | Range | Meaning |
|---|---:|---|
| Address `channel` | `0–7` | Logical servo channel |
| `minimum` | `0–180°` | Minimum commanded angle |
| `maximum` | `0–180°` | Maximum commanded angle |

If minimum and maximum are reversed, the firmware swaps them. The stored DIRECT target for the channel is also clamped to the new range.

---

### `/speed <speed>`

Retained for protocol compatibility.

```text
/speed 100
```

The current firmware stores this value in `servoSpeed`, but DIRECT servo motion does not presently use it for rate limiting.

---

## AUTO-mode servo settings

AUTO mode drives logical servos 0 (pan) and 1 (tilt) using two independent, domain-warped Perlin noise tracks for organic, non-repeating wander.

### `/robot/auto/speed <value>`

Sets the AUTO-mode time-advance rate (how quickly the noise is sampled).

```text
/robot/auto/speed 0.03
```

Accepted range: `0.001–0.5`.

### `/robot/auto/range <value>`

Sets how far (in degrees, ±) each servo wanders from its AUTO-mode center.

```text
/robot/auto/range 35
```

Accepted range: `5–70`.

### `/robot/auto/warp <value>`

Sets the domain-warp strength, which adds organic irregularity to the base noise pattern. Higher values produce less predictable, less cyclical motion.

```text
/robot/auto/warp 40
```

Accepted range: `0–100`.

### `/robot/auto/center <pan> <tilt>`

Sets the AUTO-mode center angle for logical servos 0 and 1.

```text
/robot/auto/center 90 90
```

Each value is clamped to `0–180°`. Final output is additionally constrained by that servo's configured `/robot/servolimits` range.

---

## LEDs

### `/robot/led <channel> <level>`

Immediately sets one LED channel and cancels any fade running on it.

```text
/robot/led 0 4095
```

| Argument | Range |
|---|---:|
| `channel` | `0–11` |
| `level` | `0–4095` |

---

### `/robot/led/fade <channel> <target> <durationMs>`

Fades one LED channel from its current level to a target level.

```text
/robot/led/fade 3 4095 2000
```

| Argument | Range |
|---|---:|
| `channel` | `0–11` |
| `target` | `0–4095` |
| `durationMs` | `0–600000` ms |

A zero duration performs an immediate set.

---

### `/robot/led/fadeall <target> <durationMs>`

Starts synchronized fades on all 12 logical LED channels.

```text
/robot/led/fadeall 2048 3000
```

---

### `/robot/led/easemode <easing>`

Selects the LED fade easing curve independently of servo easing.

```text
/robot/led/easemode 4
```

The easing values are the same `0–7` values listed for `/robot/easemode`.

---

## Haptics and Purr

### `/haptic <effect>`

Triggers the selected DRV2605 effect on every available haptic motor.

```text
/haptic 47
```

| Argument | Range |
|---|---:|
| `effect` | `1–123` |

---

### `/haptic <channel> <effect>`

Triggers one haptic motor.

```text
/haptic 2 47
```

| Argument | Range |
|---|---:|
| `channel` | `0–3` |
| `effect` | `1–123` |

Channel 0 is also used by Purr. A one-shot effect on channel 0 temporarily suspends Purr (for ~1.5 seconds) so it does not immediately overwrite the effect.

---

### `/robot/purr/rate <value>`

Sets the Purr root rate — a multiplier applied to every pad's individual rate in `PurrConfig.h`. `1.0` is the configured default; `0.5` halves all pad rates, `2.0` doubles them.

```text
/robot/purr/rate 0.8
```

Minimum accepted value: `0.1`.

### `/robot/purr/intensity <value>`

Sets the Purr root intensity — a multiplier applied to every pad's min/max RTP amplitude in `PurrConfig.h`.

```text
/robot/purr/intensity 0.6
```

Minimum accepted value: `0.0`. Resulting amplitudes are clamped to the DRV2605's `0–127` RTP range regardless of multiplier.

---

## Daisy Seed audio

### `/daisy <parameterID> <value>`

Sends a raw parameter ID and integer value to the Daisy Seed.

```text
/daisy 17 1
```

Common Daisy IDs:

| ID | Function | Typical value |
|---:|---|---|
| `11` | Bell/chime amplitude | `0–127` |
| `12` | Bell/chime MIDI note | `0–127` |
| `13` | Attack time | milliseconds |
| `14` | Decay time | milliseconds |
| `15` | Bell/chime preset | `0–5` |
| `16` | Pitch drop | `0` or `1` |
| `17` | Bell/chime trigger | `1` |
| `18` | Detune | cents/range value |
| `19` | Stop both engines | `1` |
| `20` | Master volume | `0–127` |
| `21` | Chirp duration | milliseconds |
| `22` | Chirp count | `1–8` |
| `23` | Chirp sequence duration | milliseconds |
| `24` | Chirp filter MIDI note | `0–127` |
| `25` | Chirp amplitude | `0–127` |
| `26` | Chirp trigger | `1` |
| `27` | Chirp resonance/Q | engine-specific value |
| `97` | Soft-reset Daisy audio engines | `1` |

Examples:

```text
/daisy 20 127
/daisy 11 100
/daisy 12 69
/daisy 17 1
```

Recovery/reset:

```text
/daisy 97 1
```

Do not send ID 99 manually as a reset. ID 99 is reserved for the Feather-to-Daisy link heartbeat.

Parameter IDs `17`, `19`, `26`, and `97` bypass the Feather's 50ms per-ID send throttle; all others are throttled to one send per 50ms.

---

### `/daisy/synth <parameterID> <value>`

Convenience form for bell/chime parameters. The Feather adds 10 to `parameterID` before forwarding it to Daisy.

```text
/daisy/synth 7 1
```

This example becomes Daisy command `17 1`, which triggers the bell/chime engine.

| OSC parameter ID | Forwarded Daisy ID | Function |
|---:|---:|---|
| `1` | `11` | Amplitude |
| `2` | `12` | MIDI note |
| `3` | `13` | Attack |
| `4` | `14` | Decay |
| `5` | `15` | Preset |
| `6` | `16` | Pitch drop |
| `7` | `17` | Trigger |
| `8` | `18` | Detune |
| `9` | `19` | Stop |

---

## Display and test controls

### `/robot/display <page>`

Selects the OLED page.

| Page | Display |
|---:|---|
| `0` | Main system/network page |
| `1` | Servo positions and motion-mode state |
| `2` | Touch sensor state |
| `3` | Network and I/O diagnostics |
| `4` | Daisy CPU load |
| `5` | System health |

Example:

```text
/robot/display 1
```

---

### `/test <colorIndex>`

Sets the Feather's onboard RGB status pixel to one of four test colors.

| Value | Color |
|---:|---|
| `0` | Red |
| `1` | Green |
| `2` | Blue |
| `3` | Yellow |

The onboard pushbutton also sends `/test <colorIndex>` to the configured status destination.

---

# Receives — OSC messages sent from the robot

## System status

### `/bot02/status/system <uptimeSeconds> <temperatureC> <cpuMHz>`

Sent approximately once per second.

```text
/bot02/status/system 1432 46.8 240
```

| Argument | Meaning |
|---|---|
| `uptimeSeconds` | Time since boot in seconds |
| `temperatureC` | ESP32 internal temperature reading |
| `cpuMHz` | CPU clock frequency |

---

## Memory status

### `/bot02/status/memory <freeHeap> <minimumFreeHeap> <largestBlock>`

Sent approximately every two seconds.

```text
/bot02/status/memory 201432 187520 114688
```

Values are bytes.

---

## Network status

### `/bot02/status/network <rssi> <lastSequence>`

Sent approximately twice per second.

```text
/bot02/status/network -52 -1
```

| Argument | Meaning |
|---|---|
| `rssi` | Wi-Fi received signal strength |
| `lastSequence` | Reserved sequence-number field; nothing in this firmware currently sets it, so it remains `-1` |

---

## Servo status

### `/bot02/status/servos <servo0> <servo1> <servo2> <servo3> <servo4> <servo5> <servo6> <servo7>`

Reports the last positions actually written to all eight logical servo outputs.

```text
/bot02/status/servos 90 90 90 90 90 90 90 90
```

| Argument | Meaning |
|---|---|
| `servo0`–`servo7` | Last angle command written while that logical channel was enabled |

These are commanded-output positions, not measured mechanical feedback. A disabled channel retains its last applied value in telemetry; this does not imply that an unpowered servo remains mechanically at that angle.

---

## Touch status

### `/bot02/touch <touchMask>`

Sent when the MPR121 touch state changes.

```text
/bot02/touch 5
```

The argument is a 12-bit bitmask. Bit `n` is 1 when touch electrode `n` is active.

Example: decimal `5` is binary `000000000101`, meaning electrodes 0 and 2 are touched.

---

## Daisy status

### `/bot02/status/daisy <online> <rxAgeMs> <cpuLoad> <rxPackets> <parseErrors> <lastError> <audioLevel>`

Sent approximately five times per second.

```text
/bot02/status/daisy 1 143 22 581 0 0 37
```

| Argument | Meaning |
|---|---|
| `online` | `1` when recognized Daisy messages are being received; otherwise `0` |
| `rxAgeMs` | Milliseconds since the last recognized Daisy message, or `-1` before the first message |
| `cpuLoad` | Daisy CPU load percentage |
| `rxPackets` | Number of recognized Daisy protocol messages received |
| `parseErrors` | Number of malformed or overlong Daisy UART lines |
| `lastError` | Most recent Daisy error/status value |
| `audioLevel` | Most recent Daisy audio-level value, if implemented |

---

## Haptic status

### `/bot02/status/haptics <healthMask> <ch0> <ch1> <ch2> <ch3>`

Sent approximately once per second.

```text
/bot02/status/haptics 15 1 1 1 1
```

`healthMask` is a four-bit mask:

| Mask | Meaning |
|---:|---|
| `0` | No haptic drivers detected |
| `1` | Channel 0 detected |
| `2` | Channel 1 detected |
| `4` | Channel 2 detected |
| `8` | Channel 3 detected |
| `15` / `0xF` | All four channels detected |

The four following values report the individual channel states explicitly.

---

## Mode transition status

### `/bot02/status/mode <event> <currentMode> <pendingMode> <active> <progressPercent> <servoCount>`

Sent when a safe mode transition starts, is retargeted, or completes.

Examples:

```text
/bot02/status/mode centering 0 2 1 0 4
/bot02/status/mode retargeted 0 1 1 37 4
/bot02/status/mode complete 1 1 0 100 4
```

| Argument | Meaning |
|---|---|
| `event` | `centering`, `retargeted`, or `complete` |
| `currentMode` | Mode active before or after the transition |
| `pendingMode` | Destination mode |
| `active` | `1` while centering, otherwise `0` |
| `progressPercent` | `0–100` |
| `servoCount` | Number of servos participating in the transition |

Mode numbers are DIRECT `0`, EASE `1`, and AUTO `2`.

---

## Ease completion

### `/bot02/status/ease <event> <servo0> <servo1>`

Sent when an EASE move finishes.

```text
/bot02/status/ease done 120 70
```

---

## Button/test message

### `/test <colorIndex>`

Sent when the onboard test button is pressed.

```text
/test 2
```

The destination is the configured status IP and port.

---

# Not currently emitted

The firmware stores fields for Daisy audio level and error state, but those values depend on the Daisy firmware sending protocol IDs `54` and `53`.

There is currently no separate OSC acknowledgement for each accepted command. Command acceptance or rejection is primarily reported over the USB Serial log.

---

# Practical tips

- **Zero-based everything**: servo channels, servo limit addresses, and LED channels are all zero-based (`0`–`7` for servos, `0`–`11` for LEDs). There is no legacy one-based numbering anywhere in this firmware.
- **Testing without Max/OSC tooling**: every address in this document can be tested directly from the Arduino IDE's Serial Monitor at 115200 baud — type the address and arguments exactly as shown in the examples (no leading `/` changes, just a plain text line) and press Enter.
- **Packet size limit**: UDP OSC packets larger than 512 bytes are rejected outright; this only matters if you're hand-building raw OSC blobs rather than using a standard OSC library.
- **Command duration cap**: any address accepting a `durationMs` argument (`/robot/ease`, `/robot/led/fade`, `/robot/led/fadeall`) rejects values above 600,000 ms (10 minutes).
