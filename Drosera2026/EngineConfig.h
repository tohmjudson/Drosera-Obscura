#pragma once

// ============================================================
// Drosera2026 — Synth Engine Selector
// ============================================================
// Set to 0 to run the demo engine (HybridEngine + ChirpEngine,
// defined in synth_engine.h).
//
// Set to 1 to run your own engine instead. Your engine lives in
// student_engine.h and must implement the StudentEngine interface
// described there (Init, Process, HandleParam, and optionally
// Update). You are free to redefine what every parameter ID means,
// except for a handful of reserved link-protocol IDs -- see the
// "Reserved IDs" note at the top of student_engine.h before you
// start.
// ============================================================
#define USE_STUDENT_ENGINE 0
