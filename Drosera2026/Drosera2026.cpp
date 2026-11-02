#include <stdio.h>
#include <string.h>
#include "daisy_seed.h"
#include "daisysp.h"

#include "EngineConfig.h"

#if USE_STUDENT_ENGINE
#include "student_engine.h"
#else
#include "synth_engine.h"
#endif

using namespace daisy;
using namespace daisysp;

DaisySeed    hw;
UartHandler  uart;
CpuLoadMeter cpu;

// --- Engine(s) ---
// When USE_STUDENT_ENGINE is 0 (default), the demo engines run.
// When USE_STUDENT_ENGINE is 1, your StudentEngine runs instead.
// See EngineConfig.h to flip this, and student_engine.h for the
// interface your engine must implement.
#if USE_STUDENT_ENGINE
StudentEngine myEngine;
#else
HybridEngine mySynth;
ChirpEngine  myChirp;
#endif

// --- Master Volume (0.0-1.0) ---
float masterVolume = 0.8f;

// --- UART receive state ---
static char msgBuffer[32];
static int  bufferPos = 0;
static bool bufferOverflow = false;

// --- Runtime state ---
static float    sampleRate = 0.0f;
static uint32_t heartbeatCounter = 0;

// Feather -> Daisy
static constexpr int ID_SOFT_RESET = 97;
static constexpr int ID_PING       = 99;

// Daisy -> Feather
static constexpr int ID_CPU_LOAD   = 50;
static constexpr int ID_READY      = 51;
static constexpr int ID_HEARTBEAT  = 52;
static constexpr int ID_STATUS     = 53;

void InitUart();
void ProcessSerial();
void ProcessMessage(int id, int value);
void SoftReset();
void SendMessage(int id, int value);

int ClampInt(int value, int minimum, int maximum)
{
    if(value < minimum) return minimum;
    if(value > maximum) return maximum;
    return value;
}

void SendMessage(int id, int value)
{
    char buffer[32];
    const int length = snprintf(buffer, sizeof(buffer), "%d %d\n", id, value);

    if(length > 0)
    {
        uart.BlockingTransmit(reinterpret_cast<uint8_t*>(buffer),
                              static_cast<size_t>(length));
    }
}

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    cpu.OnBlockStart();

    for(size_t i = 0; i < size; i++)
    {
#if USE_STUDENT_ENGINE
        const float mix = myEngine.Process() * masterVolume;
#else
        const float bell  = mySynth.Process();
        const float chirp = myChirp.Process();
        const float mix   = (bell + chirp) * masterVolume;
#endif

        out[0][i] = mix;
        out[1][i] = mix;
    }

    cpu.OnBlockEnd();
}

int main(void)
{
    hw.Init();
    sampleRate = hw.AudioSampleRate();

#if USE_STUDENT_ENGINE
    myEngine.Init(sampleRate);
#else
    mySynth.Init(sampleRate);
    myChirp.Init(sampleRate);
#endif
    cpu.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

    InitUart();
    hw.StartAudio(AudioCallback);

    // Announces a successful boot. If the Feather is not ready yet, its
    // periodic ID 99 ping will still establish the link afterward.
    SendMessage(ID_READY, 1);

    uint32_t lastCpuSend = System::GetNow();

    for(;;)
    {
        ProcessSerial();

#if USE_STUDENT_ENGINE
        // Advance any main-loop-timed sequencing in the student engine.
        myEngine.Update(System::GetNow());
#else
        // Advance the chirp sequencer using the main-loop clock.
        myChirp.Update(System::GetNow());
#endif

        const uint32_t now = System::GetNow();
        if(now - lastCpuSend >= 200)
        {
            lastCpuSend = now;

            const float load = cpu.GetAvgCpuLoad();
            const int scaled = ClampInt(static_cast<int>(load * 100.0f), 0, 100);
            SendMessage(ID_CPU_LOAD, scaled);
        }
    }
}

// Reinitializes the active engine. The protocol reset command is ID 97 so it
// does not conflict with the Feather's recurring ID 99 link ping. This
// function is part of the protocol core and runs the same way regardless of
// which engine is active.
void SoftReset()
{
#if USE_STUDENT_ENGINE
    myEngine.Init(sampleRate);
#else
    mySynth.Init(sampleRate);
    myChirp.Init(sampleRate);
#endif
    masterVolume = 0.8f;

    memset(msgBuffer, 0, sizeof(msgBuffer));
    bufferPos = 0;
    bufferOverflow = false;

    // Report that the audio engine is ready again.
    SendMessage(ID_READY, 1);
}

// ----------------------------------------------------------------------
// ProcessMessage — protocol core.
//
// Reserved IDs (97, 99, 50-53) are handled here directly and are NEVER
// forwarded to either engine. Every other ID (0-49, 54-96) is forwarded
// to the active engine's HandleParam(id, value) unchanged -- the demo
// engine and a student engine are both free to interpret those IDs
// however they want. See student_engine.h for the full reserved-ID list
// and a blank parameter table template.
// ----------------------------------------------------------------------
void ProcessMessage(int id, int val)
{
    // System protocol -- always handled first, regardless of engine.
    if(id == ID_SOFT_RESET)
    {
        if(val == 1) SoftReset();
        return;
    }
    else if(id == ID_PING)
    {
        // ID 99 is a link-health ping, not a reset. The Feather sends this
        // periodically and expects any recognized Daisy response before its
        // receive timeout expires.
        if(val == 1)
        {
            ++heartbeatCounter;
            SendMessage(ID_HEARTBEAT, static_cast<int>(heartbeatCounter));
        }
        return;
    }

    // Everything else belongs to the active synth engine.
#if USE_STUDENT_ENGINE
    myEngine.HandleParam(id, val);
#else
    // Bell / Chime Engine (IDs 11-19)
    if(id == 11)
    {
        mySynth.SetAmp(static_cast<float>(ClampInt(val, 0, 127)) / 127.0f);
    }
    else if(id == 12)
    {
        mySynth.SetMidiNote(ClampInt(val, 0, 127));
    }
    else if(id == 13)
    {
        mySynth.SetAttack(static_cast<float>(val) / 1000.0f);
    }
    else if(id == 14)
    {
        mySynth.SetDecay(static_cast<float>(val) / 1000.0f);
    }
    else if(id == 15)
    {
        mySynth.SetPreset(ClampInt(val, 0, 5));
    }
    else if(id == 16)
    {
        mySynth.SetPitchDrop(val != 0);
    }
    else if(id == 17)
    {
        if(val == 1) mySynth.Trigger();
    }
    else if(id == 18)
    {
        mySynth.SetDetune(static_cast<float>(val));
    }
    else if(id == 19)
    {
        if(val == 1)
        {
            mySynth.Stop();
            myChirp.Stop();
        }
    }

    // Master Volume (ID 20)
    else if(id == 20)
    {
        masterVolume = static_cast<float>(ClampInt(val, 0, 127)) / 127.0f;
    }

    // Chirp Engine (IDs 21-27)
    else if(id == 21)
    {
        myChirp.SetChirpDuration(val);
    }
    else if(id == 22)
    {
        myChirp.SetChirpCount(ClampInt(val, 1, 8));
    }
    else if(id == 23)
    {
        myChirp.SetSequenceDuration(val);
    }
    else if(id == 24)
    {
        myChirp.SetFilterNote(ClampInt(val, 0, 127));
    }
    else if(id == 25)
    {
        myChirp.SetAmp(static_cast<float>(ClampInt(val, 0, 127)) / 127.0f);
    }
    else if(id == 26)
    {
        if(val == 1) myChirp.TriggerSequence();
    }
    else if(id == 27)
    {
        myChirp.SetQ(static_cast<float>(val));
    }
    else
    {
        // Unknown command. ID 53 is interpreted by the Feather as status/error.
        SendMessage(ID_STATUS, id);
    }
#endif
}

void ProcessSerial()
{
    uint8_t incomingByte = 0;

    if(uart.BlockingReceive(&incomingByte, 1, 0) != UartHandler::Result::OK)
        return;

    if(incomingByte == '\n' || incomingByte == '\r')
    {
        if(bufferPos > 0 && !bufferOverflow)
        {
            msgBuffer[bufferPos] = '\0';

            int id = 0;
            int value = 0;
            if(sscanf(msgBuffer, "%d %d", &id, &value) == 2)
                ProcessMessage(id, value);
            else
                SendMessage(ID_STATUS, -1);
        }

        memset(msgBuffer, 0, sizeof(msgBuffer));
        bufferPos = 0;
        bufferOverflow = false;
        return;
    }

    if(incomingByte < 32)
        return;

    if(bufferPos < static_cast<int>(sizeof(msgBuffer)) - 1)
    {
        msgBuffer[bufferPos++] = static_cast<char>(incomingByte);
    }
    else
    {
        // Discard the entire overlong line at the next delimiter.
        bufferOverflow = true;
    }
}

void InitUart()
{
    UartHandler::Config config;
    config.baudrate      = 115200;
    config.periph        = UartHandler::Config::Peripheral::USART_1;
    config.mode          = UartHandler::Config::Mode::TX_RX;
    config.pin_config.rx = {DSY_GPIOB, 7}; // Daisy pin 15, receives Feather TX
    config.pin_config.tx = {DSY_GPIOB, 6}; // Daisy pin 14, sends to Feather RX
    uart.Init(config);
}
