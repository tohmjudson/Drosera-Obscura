#pragma once
#include "daisysp.h"
#include <cmath>
#include <cstdlib>

// ============================================================
// StudentEngine — starter template
// ============================================================
// This file is yours. Build whatever synthesis/sequencing you
// want here. Drosera2026.cpp only ever calls three functions on
// this class: Init(), Update(), and Process(). Everything else
// is internal to your design.
//
// --------------------------------------------------------------
// RESERVED IDs -- DO NOT REUSE, DO NOT IGNORE
// --------------------------------------------------------------
// These parameter IDs are the link protocol between the Feather
// (robot controller) and this Daisy board. They are handled in
// Drosera2026.cpp BEFORE your engine ever sees a message, so you
// cannot break them by accident -- but you must not design your
// own parameter map to assume these numbers are free:
//
//   97   Soft reset request        (Feather -> Daisy)
//   99   Link health ping          (Feather -> Daisy)
//   50   CPU load report           (Daisy -> Feather)
//   51   Engine ready report       (Daisy -> Feather)
//   52   Heartbeat / ping response (Daisy -> Feather)
//   53   Status / unknown-ID error (Daisy -> Feather)
//
// Every other ID (0-49 and 54-96) is completely open. You can
// keep the demo's layout (11-27, see synth_engine.h) or throw it
// out and number your own parameters however makes sense to you.
//
// Whatever you choose, fill in the table below and keep it
// current -- the robotics team will read this table when they
// wire up touch/servo events to trigger your sounds.
//
// This template ships with a tiny working example already wired
// up (a plain sine beep with pitch/volume/duration/trigger) so you
// hear sound the first time you build, instead of silence. Rip it
// out whenever you're ready -- it's just a starting point, not a
// requirement.
// --------------------------------------------------------------
// YOUR PARAMETER ID TABLE (fill this in as you build)
// --------------------------------------------------------------
//   ID   Name              Range        Notes
//   --   ----------------  -----------  -----------------------
//    1   Pitch             0-127        MIDI note (A4 = 69 = 440Hz)
//    2   Volume            0-127        Overall output level
//    3   Duration          int ms       Note length before auto-off
//    4   Trigger           1            Fires a new note
//
// ============================================================

class StudentEngine {
public:
    // --------------------------------------------------------
    // Init — called once at startup with the audio sample rate.
    // Set up oscillators, filters, envelopes, etc. here.
    // --------------------------------------------------------
    void Init(float sampleRate) {
        sample_rate = sampleRate;

        osc.Init(sampleRate);
        osc.SetWaveform(daisysp::Oscillator::WAVE_SIN);
        osc.SetFreq(440.0f);
        osc.SetAmp(1.0f);

        pitch_note    = 69;     // A4
        volume        = 0.8f;
        duration_ms   = 300.0f;

        note_active   = false;
        samples_left  = 0;
    }

    // --------------------------------------------------------
    // HandleParam — called whenever a recognized (non-reserved)
    // "<id> <value>" message arrives over UART from the Feather,
    // or from /daisy or /daisy/synth over OSC on the robot side.
    //
    // id and value arrive exactly as sent -- integers. Scale,
    // clamp, and interpret them however your design needs.
    // --------------------------------------------------------
    void HandleParam(int id, int value) {
        if (id == 1) {
            // Pitch: MIDI note 0-127 -> Hz, standard tuning (A4 = 440Hz)
            pitch_note = (value < 0) ? 0 : (value > 127) ? 127 : value;
            osc.SetFreq(440.0f * powf(2.0f, (float)(pitch_note - 69) / 12.0f));
        } else if (id == 2) {
            // Volume: 0-127 -> 0.0-1.0
            int v = (value < 0) ? 0 : (value > 127) ? 127 : value;
            volume = (float)v / 127.0f;
        } else if (id == 3) {
            // Duration: milliseconds the note stays on before auto-off
            duration_ms = (value < 1) ? 1.0f : (float)value;
        } else if (id == 4) {
            // Trigger: start a new note immediately
            if (value == 1) {
                samples_left = (int)(duration_ms / 1000.0f * sample_rate);
                note_active  = true;
            }
        }
    }

    // --------------------------------------------------------
    // Update — called once per main-loop iteration (not per
    // audio sample). Use this for sequencing/timing logic that
    // depends on wall-clock time, similar to ChirpEngine::Update
    // in synth_engine.h. Optional -- leave empty if you don't
    // need it.
    // --------------------------------------------------------
    void Update(uint32_t now) {
        // Not needed for this simple example -- Process() tracks
        // note length directly in samples. Use this function if
        // you build something sequenced (e.g. multi-step patterns)
        // that needs wall-clock timing like ChirpEngine does.
    }

    // --------------------------------------------------------
    // Process — called once per audio sample from AudioCallback.
    // Return your next output sample here.
    // --------------------------------------------------------
    inline float Process() {
        if (!note_active) return 0.0f;

        if (samples_left <= 0) {
            note_active = false;
            return 0.0f;
        }
        samples_left--;

        return osc.Process() * volume;
    }

private:
    float sample_rate = 0.0f;

    daisysp::Oscillator osc;

    int   pitch_note;
    float volume;
    float duration_ms;

    bool note_active;
    int  samples_left;

    // TODO: add more oscillators, envelopes, filters, and state
    // here as your design grows.
};
