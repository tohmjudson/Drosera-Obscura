#pragma once
#include "daisysp.h"
#include <cmath>
#include <cstdlib>

// ---------------------------------------------------------------------------
// HybridEngine — One-shot FM synthesizer
//
// Triggering is ignored while a note is playing (no retrigger).
// Preset and all parameters are latched at the moment of Trigger().
//
// Parameter map (set via ProcessSerial in Drosera2026.cpp):
//   ID 11  Amp         0–127  → 0.0–1.0
//   ID 12  MIDI Note   0–127  → Hz (A4=440)
//   ID 13  Attack      int ms → sec (no clamp)
//   ID 14  Decay       int ms → sec (no clamp)
//   ID 15  Preset      0–5    (see PresetParams table)
//   ID 16  Pitch Drop  0=off, 1=on
//   ID 17  Trigger     1 = fire
//   ID 18  Detune      0–N cents ±range per trigger
//   ID 19  Stop        1 = fast fade to silence (~0.5 sec), stops both engines
//   ID 50  CPU Load    outbound only
// ---------------------------------------------------------------------------

class HybridEngine {
public:

    // ------------------------------------------------------------------
    // Preset table
    // Each preset defines the FM ratio, modulation index, and feedback.
    // Changes take effect on the next Trigger() call.
    // ------------------------------------------------------------------
    struct PresetParams {
        float ratio;      // modulator freq = carrier freq * ratio
        float index;      // FM depth (scales with envelope for natural decay)
        float feedback;   // modulator self-feedback (0 = none)
        const char* name;
    };

    static const PresetParams& GetPreset(int idx) {
        static const PresetParams table[6] = {
            { 1.000f, 0.30f, 0.00f, "Whistle"      },
            { 2.756f, 0.80f, 0.00f, "Soft Bell"    },
            { 1.500f, 1.20f, 0.05f, "Warm Bell"    },
            { 3.000f, 1.80f, 0.10f, "Bright Bell"  },
            { 1.414f, 2.50f, 0.20f, "Metallic Bell"},
            { 3.500f, 1.00f, 0.00f, "Wood Tone"    },
        };
        int i = (idx < 0) ? 0 : (idx > 5) ? 5 : idx;
        return table[i];
    }

    // ------------------------------------------------------------------
    // Init — call once at startup
    // ------------------------------------------------------------------
    void Init(float sr) {
        sample_rate = sr;

        // Seed random for detune — varies with time elapsed since boot
        srand((unsigned int)daisy::System::GetNow());

        carrier.Init(sr);
        carrier.SetWaveform(daisysp::Oscillator::WAVE_SIN);
        modulator.Init(sr);
        modulator.SetWaveform(daisysp::Oscillator::WAVE_SIN);

        // Default parameter values
        amp           = 0.8f;
        freq_hz       = 440.0f;
        attack_time   = 0.01f;
        decay_time    = 1.5f;
        preset_idx    = 2;       // Warm Bell
        pitch_drop    = false;
        detune_cents  = 0.0f;

        // Envelope state
        env   = 0.0f;
        state = IDLE;

        // FM state
        last_mod        = 0.0f;
        active_freq     = 440.0f;
        active_ratio    = 1.5f;
        active_index    = 1.2f;
        active_feedback = 0.05f;

        // Pitch drop — fixed ~80ms exponential decay
        pd_env   = 0.0f;
        pd_coeff = expf(-4.605f / (0.08f * sr));   // ~1% after 80ms

        // Envelope coefficients (recalculated on each Trigger)
        attack_coeff = 1.0f - expf(-2.996f / (0.01f * sr));
        decay_coeff  = expf(-4.605f / (1.5f  * sr));

        // Stop fade — fixed 0.5 sec (speaker-safe)
        stop_coeff = expf(-4.605f / (0.5f * sr));
    }

    // ------------------------------------------------------------------
    // Parameter setters — safe to call at any time; latched on Trigger()
    // ------------------------------------------------------------------
    void SetAmp(float a)        { amp = a; }

    void SetMidiNote(int note) {
        // Standard equal temperament, A4 = 440 Hz
        freq_hz = 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
    }

    // Attack time in seconds (no clamp — controlled upstream)
    void SetAttack(float t)     { attack_time = t; }

    // Decay time in seconds (no clamp — controlled upstream)
    void SetDecay(float t)      { decay_time  = t; }

    // Preset index 0–5
    void SetPreset(int p)       { preset_idx  = (p < 0) ? 0 : (p > 5) ? 5 : p; }

    // Pitch drop: 0 = off, 1 = on
    void SetPitchDrop(bool on)  { pitch_drop  = on; }

    // Detune range in cents — random ±range applied on each Trigger()
    // 0 = no detune, 100 = ±100 cents (±1 semitone)
    void SetDetune(float cents) { detune_cents = cents; }

    // ------------------------------------------------------------------
    // Trigger — starts a new note. Ignored if a note is already playing.
    // ------------------------------------------------------------------
    void Trigger() {
        if (state != IDLE) return;

        // Apply random detune within ±detune_cents
        active_freq = freq_hz;
        if (detune_cents > 0.0f) {
            float r = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
            float offset_cents = r * detune_cents;
            active_freq = freq_hz * powf(2.0f, offset_cents / 1200.0f);
        }

        // Latch preset
        const PresetParams& p = GetPreset(preset_idx);
        active_ratio    = p.ratio;
        active_index    = p.index;
        active_feedback = p.feedback;

        // Compute per-note envelope coefficients
        // Attack: reaches ~95% in attack_time seconds
        attack_coeff = 1.0f - expf(-2.996f / (attack_time * sample_rate));
        // Decay: falls to ~1% in decay_time seconds
        decay_coeff  = expf(-4.605f / (decay_time  * sample_rate));

        // Reset state
        env      = 0.0f;
        last_mod = 0.0f;
        state    = ATTACK;

        // Pitch drop envelope (only active if enabled)
        pd_env = pitch_drop ? 1.0f : 0.0f;
    }

    // ------------------------------------------------------------------
    // Stop — initiates a fast 0.5 sec fade to silence (speaker-safe)
    // ------------------------------------------------------------------
    void Stop() {
        if (state == IDLE) return;
        state = STOPPING;
    }

    // ------------------------------------------------------------------
    // Process — call once per sample from AudioCallback
    // ------------------------------------------------------------------
    inline float Process() {
        if (state == IDLE) return 0.0f;

        // --- Amplitude envelope ---
        if (state == ATTACK) {
            env += (1.0f - env) * attack_coeff;
            if (env >= 0.999f) {
                env   = 1.0f;
                state = DECAY;
            }
        } else if (state == DECAY) {
            env *= decay_coeff;
            if (env < 0.0001f) {
                env   = 0.0f;
                state = IDLE;
                return 0.0f;
            }
        } else if (state == STOPPING) {
            env *= stop_coeff;
            if (env < 0.0001f) {
                env   = 0.0f;
                state = IDLE;
                return 0.0f;
            }
        }

        // --- Pitch drop ---
        // Short downward glide on the transient (~15% drop, ~2.5 semitones, ~80ms)
        float pd_offset = 0.0f;
        if (pitch_drop && pd_env > 0.0001f) {
            pd_env   *= pd_coeff;
            pd_offset = -active_freq * 0.15f * pd_env;
        }
        float current_freq = active_freq + pd_offset;

        // --- FM synthesis ---
        // Modulator with self-feedback (approximated via frequency offset)
        float mod_base = current_freq * active_ratio;
        modulator.SetFreq(mod_base + last_mod * active_feedback * mod_base);
        float mod_out = modulator.Process();   // -1.0 to 1.0
        last_mod = mod_out;

        // FM index scales with envelope: bright transient, mellows as it decays
        float carrier_freq = current_freq + mod_out * active_index * mod_base * env;
        carrier.SetFreq(carrier_freq);

        return carrier.Process() * env * amp;
    }

private:
    enum State { IDLE, ATTACK, DECAY, STOPPING } state;

    daisysp::Oscillator carrier, modulator;
    float sample_rate;

    float amp, freq_hz, attack_time, decay_time;
    int   preset_idx;
    bool  pitch_drop;
    float detune_cents;

    float env;
    float attack_coeff, decay_coeff, stop_coeff;

    float last_mod;
    float active_freq, active_ratio, active_index, active_feedback;

    float pd_env, pd_coeff;
};


// ---------------------------------------------------------------------------
// ChirpEngine — Bandpass-filtered noise burst sequencer
//
// A sequence of evenly-spaced noise chirps triggered by a single OSC message.
// Triggering is ignored while a sequence is active.
// Timing is driven by Update() in the main loop; audio is generated in
// Process() from the AudioCallback.
//
// Parameter map (set via ProcessSerial in Drosera2026.cpp):
//   ID 21  Chirp Duration   int ms (length of each individual chirp)
//   ID 22  Chirp Count      1–8
//   ID 23  Sequence Duration int ms (total window; chirps evenly spaced)
//   ID 24  Filter Center    MIDI note 0–127 → Hz (A4=440)
//   ID 25  Amp              0–127 → 0.0–1.0
//   ID 26  Trigger          1 = start sequence (ignored if already active)
//   ID 27  Q                raw int from Max → passed directly to SVF
// ---------------------------------------------------------------------------

class ChirpEngine {
public:

    // ------------------------------------------------------------------
    // Init — call once at startup
    // ------------------------------------------------------------------
    void Init(float sr) {
        sample_rate = sr;

        noise.Init();
        noise.SetAmp(1.0f);

        filter.Init(sr);
        filter.SetFreq(880.0f);
        filter.SetRes(1.0f);

        amp                  = 0.8f;
        chirp_duration_ms    = 50.0f;
        chirp_count          = 3;
        sequence_duration_ms = 1000.0f;
        filter_freq          = 880.0f;
        q_val                = 1.0f;

        chirp_state         = IDLE;
        env_pos             = 0;
        attack_samples      = 0;
        total_chirp_samples = 0;
        attack_recip        = 1.0f;
        decay_recip         = 1.0f;

        sequence_active = false;
        chirps_fired    = 0;
        last_chirp_time = 0;
        gap_ms          = 0.0f;
        pending_chirp   = false;
    }

    // ------------------------------------------------------------------
    // Parameter setters
    // ------------------------------------------------------------------
    void SetAmp(float a)             { amp = a; }
    void SetChirpDuration(int ms)    { chirp_duration_ms = (float)ms; }
    void SetChirpCount(int c)        { chirp_count = (c < 1) ? 1 : (c > 8) ? 8 : c; }
    void SetSequenceDuration(int ms) { sequence_duration_ms = (float)ms; }

    void SetFilterNote(int note) {
        filter_freq = 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
        filter.SetFreq(filter_freq);
    }

    // Raw Q value from Max — range controlled upstream
    void SetQ(float q) {
        q_val = q;
        filter.SetRes(q_val);
    }

    // ------------------------------------------------------------------
    // TriggerSequence — call from main loop (e.g. ProcessSerial)
    // Ignored if a sequence is already active.
    // ------------------------------------------------------------------
    void TriggerSequence() {
        if (sequence_active) return;

        // Evenly space chirps across the sequence window
        gap_ms = (chirp_count > 1)
                 ? (sequence_duration_ms / (float)chirp_count)
                 : sequence_duration_ms;

        chirps_fired    = 0;
        sequence_active = true;
        last_chirp_time = 0;
        pending_chirp   = false;
    }

    // ------------------------------------------------------------------
    // Update — call from main loop every iteration
    // Advances the sequence timer and sets pending_chirp flag when it is
    // time for the next chirp. Actual audio is generated in Process().
    // ------------------------------------------------------------------
    void Update(uint32_t now) {
        if (!sequence_active) return;

        // All chirps fired — wait for last one to finish, then close out
        if (chirps_fired >= chirp_count) {
            if (chirp_state == IDLE) sequence_active = false;
            return;
        }

        if (pending_chirp)      return;  // waiting for AudioCallback to pick up
        if (chirp_state != IDLE) return; // current chirp still sounding

        bool should_fire = false;
        if (chirps_fired == 0) {
            should_fire = true;          // first chirp fires immediately
        } else {
            uint32_t elapsed = now - last_chirp_time;
            should_fire = (elapsed >= (uint32_t)gap_ms);
        }

        if (should_fire) {
            pending_chirp   = true;
            last_chirp_time = now;
            chirps_fired++;
        }
    }

    // ------------------------------------------------------------------
    // Stop — immediately silences and resets the engine
    // ------------------------------------------------------------------
    void Stop() {
        chirp_state     = IDLE;
        sequence_active = false;
        pending_chirp   = false;
        env_pos         = 0;
    }

    // ------------------------------------------------------------------
    // Process — call once per sample from AudioCallback
    // ------------------------------------------------------------------
    inline float Process() {
        // Pick up a pending chirp trigger from the main loop
        if (pending_chirp && chirp_state == IDLE) {
            pending_chirp = false;
            FireChirp();
        }

        if (chirp_state == IDLE) return 0.0f;

        // --- Linear envelope ---
        float env_val;
        if (chirp_state == ATTACK) {
            env_val = (float)env_pos * attack_recip;
            env_pos++;
            if (env_pos > attack_samples) chirp_state = DECAY;
        } else {
            // DECAY
            int decay_pos = env_pos - attack_samples;
            env_val = 1.0f - (float)decay_pos * decay_recip;
            env_pos++;
            if (env_pos >= total_chirp_samples) {
                chirp_state = IDLE;
                return 0.0f;
            }
        }

        // --- Bandpass-filtered white noise ---
        float n = noise.Process();
        filter.Process(n);
        return filter.Band() * env_val * amp;
    }

private:
    // ------------------------------------------------------------------
    // FireChirp — called from Process() when a pending chirp is ready
    // ------------------------------------------------------------------
    void FireChirp() {
        total_chirp_samples = (int)(chirp_duration_ms / 1000.0f * sample_rate);
        if (total_chirp_samples < 2) total_chirp_samples = 2;

        // 10% attack, 90% decay
        attack_samples = (int)(total_chirp_samples * 0.1f);
        if (attack_samples < 1) attack_samples = 1;

        int decay_samples = total_chirp_samples - attack_samples;
        if (decay_samples < 1) decay_samples = 1;

        attack_recip = 1.0f / (float)attack_samples;
        decay_recip  = 1.0f / (float)decay_samples;

        // Latch current filter settings
        filter.SetFreq(filter_freq);
        filter.SetRes(q_val);

        env_pos     = 0;
        chirp_state = ATTACK;
    }

    enum ChirpState { IDLE, ATTACK, DECAY } chirp_state;

    float sample_rate;
    daisysp::WhiteNoise noise;
    daisysp::Svf        filter;

    float amp;
    float chirp_duration_ms;
    int   chirp_count;
    float sequence_duration_ms;
    float filter_freq;
    float q_val;

    // Per-chirp envelope (linear)
    int   env_pos;
    int   attack_samples;
    int   total_chirp_samples;
    float attack_recip;
    float decay_recip;

    // Sequencer state (main loop side)
    bool          sequence_active;
    int           chirps_fired;
    uint32_t      last_chirp_time;
    float         gap_ms;

    // Handoff flag between main loop and AudioCallback
    volatile bool pending_chirp;
};
