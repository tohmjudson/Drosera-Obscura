# Drosera Obscura

Drosera Obscura is a deeply immersive XR project that dissolves the
boundaries between the organic and the digital through a multisensory
environment shaped by sight, sound, scent, and touch. Set within a
speculative post-human bogscape, the experience invites participants to
explore a living world of animatronic flora, reactive soundscapes, and
tactile interfaces that blur the line between virtual and physical space.

Inspired by evolved carnivorous plants and their symbiotic relationships
with sound and synthetic biology, the project transforms passive
observation into active participation. Participants influence the
environment in real time: disturbing digital foliage releases subtle
fragrances, alters lighting patterns, and reshapes spatialized sound.

These interactions expand VR outward, turning what is often an isolated
medium into a performative, theatrical ecosystem that responds to human
presence with both eerie intimacy and strange beauty.

At its core, Drosera Obscura examines ecological interdependence,
embodiment, and the evolving relationship between technology and living
systems — offering an experience that feels at once alien, responsive,
and alive.

This repository holds the firmware for one physical piece of that
ecosystem: an animatronic carnivorous-plant "head" that senses touch,
moves, and makes sound in response to participants.

---

## What's in this repo

The head is built from two independent microcontroller projects that
talk to each other over a simple serial link:

| System | Hardware | What it does |
|---|---|---|
| **Head Controller** | Adafruit ESP32 Feather | Touch sensing, servo motion, LEDs, haptics ("purr"), Wi-Fi/OSC control, and an OLED status display. Sends simple commands to the Daisy over UART to trigger sound. |
| **Audio Engine** | Electrosmith Daisy Seed | Generates all sound in real time based on commands received from the Feather. Built around a swappable synth engine so a sound-design team can write fully custom audio without touching the robot's code. |

Both boards mount together on a custom carrier board with labeled
connectors — physical assembly is covered by a separate tutorial video,
not this README.

---

## Repo layout

```
/
├── DroseraHeadController/   # ESP32 Feather project (Arduino)
│   ├── README.md            # setup, hardware, modes, file reference
│   ├── OSC_Reference.md     # full OSC / Serial command reference
│   └── *.ino / *.h / *.cpp
└── Drosera2026/             # Daisy Seed project (C++ / libDaisy)
    ├── README.md            # toolchain setup, build/flash, engine system
    └── *.cpp / *.h / Makefile
```

**Folder names are intentional, not arbitrary:**

- `DroseraHeadController/` must keep that exact name — the Arduino IDE
  requires a sketch's folder name to match its `.ino` filename.
- `Drosera2026/` can technically be named anything (Daisy projects have
  no such restriction), but it matches the project's `TARGET` name in
  its `Makefile`, so we're keeping it as-is.

Neither toolchain's libraries (Arduino board/library files, or
`libDaisy`/`DaisySP`) are stored in this repo — each is installed once
per machine by that platform's own setup process, as described in the
relevant sub-README.

---

## Quick start

1. **Clone this repo.**
2. **Decide which system you're working on** — robot behavior/motion/touch,
   or sound — using the table above. Many contributors will eventually
   touch both.
3. **Copy the relevant project folder into place:**
   - `DroseraHeadController/` → your Arduino sketches folder.
   - `Drosera2026/` → `DaisyExamples/MyProjects/` (sitting next to
     `libDaisy/` and `DaisySP/`).
4. **Follow that project's own README** for toolchain installation,
   build, and flashing steps — each is written to be followed from a
   completely clean machine.
5. **If you're writing sound** for the Daisy, you do not need the
   Arduino side at all, and you do not need to understand the UART
   protocol — see `Drosera2026/README.md` §2 for the two-engine system
   that keeps sound design isolated from robot firmware.

### Local secrets and example config

The ESP32 head controller uses `Secrets.h` and `Secrets.cpp` for Wi-Fi and
network values. These files are intentionally local-only and should not be
committed to version control, even if they contain placeholder or dummy data.

Use the example template files in `DroseraHeadController/` as your starting
point:

- `Secrets.example.h`
- `Secrets.example.cpp`

Copy them to `Secrets.h` and `Secrets.cpp`, fill in your own values, and keep
those real files out of Git. The repo's `.gitignore` already excludes the
actual `Secrets.*` files.

---

## Which doc do I need?

| I want to... | Go to |
|---|---|
| Set up and flash the Feather (robot motion/touch/LEDs/haptics) | `DroseraHeadController/README.md` |
| Look up a specific OSC or Serial command | `DroseraHeadController/OSC_Reference.md` |
| Set up and flash the Daisy Seed (audio) | `Drosera2026/README.md` |
| Write my own synth engine without touching robot code | `Drosera2026/README.md` §2 |

---

## Credits

*Placeholder — team, course/installation context, and acknowledgments
to be added.*

## License

*Placeholder — to be determined.*
