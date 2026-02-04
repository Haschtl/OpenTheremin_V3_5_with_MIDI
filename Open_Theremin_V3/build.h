// Build definitions

#ifndef _BUILD_H
#define _BUILD_H



// Set to build with control voltage output (experimental)
#define CV_ENABLED 0

// DAC SPI clock (MCP49xx max is typically 20MHz, keep board/wiring quality in mind).
#define OT_SPI_CLOCK_HZ 20000000

// Optional SPI DMA backend (UNO R4 FSP SPI + DTC path).
#define OT_USE_DMA 0

// Audio ISR tick rate in Hz (31.25kHz keeps legacy behavior).
// Original: 31250
#define OT_AUDIO_TICK_HZ 48000 
#define OT_AUDIO_TICK_HZ_MIN 20000
#define OT_AUDIO_TICK_HZ_MAX 96000

// Audio rate preset helper in application.cpp:
// 0 = 31.25kHz, 1 = 40kHz, 2 = 48kHz, 3 = custom OT_AUDIO_TICK_HZ.
#define OT_AUDIO_RATE_PRESET 3

// Median filtering for analog pots (odd number, currently supported: 3).
#define OT_POT_MEDIAN_SAMPLES 3

// Audio timbre shaping controls.
// Morph speed between adjacent wavetables in Q8 domain per audio tick (1 = slow/smooth).
#define OT_WAVEMORPH_STEP_Q8 1
// Runtime-togglable defaults for advanced audio shaping features.
#define OT_WAVEMORPH_ENABLE_DEFAULT 1
#define OT_TILT_ENABLE_DEFAULT 1
#define OT_SOFTCLIP_ENABLE_DEFAULT 1
#define OT_VIBRATO_ENABLE_DEFAULT 1
// Start pitch-dependent darkening when absolute phase increment exceeds this value.
#define OT_TILT_START_INCREMENT 512
// Max wet amount (0..255) for pitch-dependent darkening blend.
#define OT_TILT_WET_MAX 160
// 2-pole tilt filter tuning.
#define OT_TILT_BIQUAD_Q_X1000 707
#define OT_TILT_CUTOFF_MAX_HZ 8000
#define OT_TILT_CUTOFF_MIN_HZ 1800
// Soft-clip enable and drive (lower shift => stronger saturation).
#define OT_SOFTCLIP_CUBIC_SHIFT 24
// Subtle vibrato/jitter tuning (applied to oscillator phase increment).
#define OT_VIBRATO_HZ_X100 500
#define OT_VIBRATO_DEPTH_PPM 2500
#define OT_JITTER_DEPTH_PPM 700

// MIDI IN control channel (0..15 => MIDI channels 1..16).
#define OT_MIDI_IN_CHANNEL 0

// MIDI transport:
// 0 = USB serial MIDI byte stream (default, stable)
// 1 = native USB MIDI class (requires UNO R4 core with TinyUSB MIDI descriptor support)
#ifndef OT_MIDI_NATIVE_USB
#define OT_MIDI_NATIVE_USB 0
#endif

// Baud for USB serial MIDI byte stream when OT_MIDI_NATIVE_USB == 0.
#ifndef OT_MIDI_SERIAL_BAUD
#define OT_MIDI_SERIAL_BAUD 115200
#endif

#endif // _BUILD_H
