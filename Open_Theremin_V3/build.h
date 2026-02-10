// Build definitions

#ifndef _BUILD_H
#define _BUILD_H



// Set to build with control voltage output (experimental)
#define CV_ENABLED 0

// Split controls:
// - OT_DEBUG_LOG_ENABLE: enables text debug logs/macros.
// - OT_SERIAL_MIDI_ENABLE: enables MIDI byte stream over USB CDC Serial.
// Defaults preserve previous behavior (DEBUG build => logs on, serial MIDI off).
#ifndef OT_DEBUG_LOG_ENABLE
  #define OT_DEBUG_LOG_ENABLE 0
#endif

#ifndef OT_SERIAL_MIDI_ENABLE
  #define OT_SERIAL_MIDI_ENABLE 0
#endif

// DAC SPI clock (MCP49xx max is typically 20MHz, but board/shield wiring is often
// more stable at 12MHz).
#define OT_SPI_CLOCK_HZ 20000000

// Optional SPI DMA backend (UNO R4 FSP SPI + DTC path).
#define OT_USE_DMA 0
// DMA transfer timeout budget per 16-bit SPI frame.
#define OT_DMA_TRANSFER_TIMEOUT_US 200
// 0 = keep running on DMA frame errors (recommended for live), 1 = hard-fail.
#define OT_DMA_HARDFAIL 0

// Audio ISR tick rate in Hz (31.25kHz keeps legacy behavior).
// Original: 31250
#define OT_AUDIO_TICK_HZ 48000
#define OT_AUDIO_TICK_HZ_MIN 20000
#define OT_AUDIO_TICK_HZ_MAX 96000
// Safety cap for runtime preset switching to avoid unstable high-rate timer restarts.
// Set to 0 to disable capping.
#define OT_AUDIO_TICK_SAFE_MAX_HZ 96000

// Timer selection:
// 0 = prefer GPT (default core behavior), 1 = prefer AGT (often more robust on some R4 setups).
#define OT_TIMER_PREFER_AGT 0

// Audio rate preset helper in application.cpp:
// 0 = 31.25kHz, 1 = 40kHz, 2 = 48kHz, 3 = custom OT_AUDIO_TICK_HZ.
#define OT_AUDIO_RATE_PRESET 0

// Median filtering for analog pots (odd number, currently supported: 3).
#define OT_POT_MEDIAN_SAMPLES 3

// Gate time (ms) for capture edge counting during calibration/measurement.
#define OT_CAPTURE_GATE_MS 1000

// Antenna response gain (higher = more sensitive hand movement response).
#define OT_PITCH_RESPONSE_GAIN 42
// Pitch response curve (0 = linear raw-delta mapping, 1 = nonlinear compression at larger deltas).
#define OT_PITCH_CURVE_ENABLE 1
// Curve strength for OT_PITCH_CURVE_ENABLE (higher = stronger compression near antenna).
#define OT_PITCH_CURVE_K 10000
// Average this many capture periods before publishing one pitch sample (1 = off).
#define OT_PITCH_CAPTURE_AVG_EDGES 8
// Preserve fractional resolution when averaging capture periods (Q bits in raw value).
#define OT_PITCH_CAPTURE_Q_BITS 4
// Capture on both rising and falling edges for higher effective resolution.
#define OT_PITCH_CAPTURE_BOTH_EDGES 1
// 3-sample median prefilter on pitch capture path (1 = enabled, 0 = disabled).
#define OT_PITCH_MEDIAN3_ENABLE 1
// Pitch low-pass strength: filtered += (raw - filtered) >> shift (0 = off, 2 = legacy behavior).
#define OT_PITCH_FILTER_SHIFT 6
// Ignore tiny control-path pitch delta movement (in pitchDelta units, post gain/curve).
#define OT_PITCH_CONTROL_DEADBAND 0
// Limit sample-increment change per control update (0 = disabled).
#define OT_PITCH_DELTA_LIMIT_PER_UPDATE 0
// Use reciprocal-domain pitch mapping around calibration base (1 = on, 0 = linear delta).
#define OT_PITCH_RECIPROCAL_MAP_ENABLE 1
// Output scaling for reciprocal mapping (higher = less sensitive).
#define OT_PITCH_RECIPROCAL_SHIFT 2


#define OT_VOLUME_RESPONSE_GAIN 12
// Average this many capture periods before publishing one volume sample (1 = off).
#define OT_VOLUME_CAPTURE_AVG_EDGES 8
// Preserve fractional resolution when averaging capture periods (Q bits in raw value).
#define OT_VOLUME_CAPTURE_Q_BITS 4
// Capture on both rising and falling edges for higher effective resolution.
#define OT_VOLUME_CAPTURE_BOTH_EDGES 1
// Volume raw capture sanitize guard (1 = enabled, 0 = disabled / Rev3-closer).
#define OT_VOLUME_SANITIZE_ENABLE 1
// 3-sample median prefilter on volume capture path (1 = enabled, 0 = disabled).
#define OT_VOLUME_MEDIAN3_ENABLE 1
// Volume low-pass strength: filtered += (raw - filtered) >> shift (0 = off, 1 = 50/50 mix, 2..4 smoother).
#define OT_VOLUME_FILTER_SHIFT 3
// Volume mapping scale shift after response gain multiply.
#define OT_VOLUME_MAP_SHIFT 1
// Ignore tiny control-path volume movement (in loop_hand_pos units, 0..255).
#define OT_VOLUME_CONTROL_DEADBAND 2
// Limit vScaledVolume step per control update (0 = disabled).
#define OT_VOLUME_DELTA_LIMIT_PER_UPDATE 512
// Audio-rate amplitude slew for anti-zipper noise: y += (target - y) >> shift (0 = off).
#define OT_AUDIO_VOLUME_SLEW_SHIFT 9

// Output fade gate (Q8 0..256) for smooth pause/unpause and startup transitions.
// Every OT_OUTPUT_FADE_STEP_MS the gate moves by the configured Q8 step.
#define OT_OUTPUT_FADE_STEP_MS 2
#define OT_OUTPUT_FADE_UP_STEP_Q8 4
#define OT_OUTPUT_FADE_DOWN_STEP_Q8 8

// Clean-sine diagnostics mode:
// 1 = lock to plain sine voice and fixed clean engine settings (ignores parameter/value pot changes).
// 0 = normal instrument behavior.
#ifndef OT_TESTMODE_CLEAN_SINE
#define OT_TESTMODE_CLEAN_SINE 1
#endif

// Audio timbre shaping controls.
// Morph speed between adjacent wavetables in Q8 domain per audio tick (1 = slow/smooth).
#define OT_WAVEMORPH_STEP_Q8 1
// Runtime-togglable defaults for advanced audio shaping features.
#define OT_WAVEMORPH_ENABLE_DEFAULT 1
#define OT_TILT_ENABLE_DEFAULT 1
#define OT_SOFTCLIP_ENABLE_DEFAULT 1
#define OT_VIBRATO_ENABLE_DEFAULT 0
// Start pitch-dependent darkening when absolute phase increment exceeds this value.
#define OT_TILT_START_INCREMENT 512
// Max wet amount (0..255) for pitch-dependent darkening blend.
#define OT_TILT_WET_MAX 160
// 2-pole tilt filter tuning.
#define OT_TILT_BIQUAD_Q_X1000 707
#define OT_TILT_CUTOFF_MAX_HZ 8000
#define OT_TILT_CUTOFF_MIN_HZ 1800
// Perceived loudness compensation for high pitch (based on absolute phase increment).
// 1 = enable, 0 = disable.
#define OT_PITCH_LOUDNESS_COMP_ENABLE 1
// Start applying attenuation above this absolute increment.
#define OT_PITCH_LOUDNESS_COMP_START_INCREMENT 2600
// Attenuation growth: attenQ8 = (absIncrement - start) >> slopeShift.
#define OT_PITCH_LOUDNESS_COMP_SLOPE_SHIFT 5
// Max attenuation in Q8 (0..255). Example 96 ~= -37.5%.
#define OT_PITCH_LOUDNESS_COMP_MAX_ATTEN_Q8 127
// Soft-clip enable and drive (lower shift => stronger saturation).
#define OT_SOFTCLIP_CUBIC_SHIFT 24
// Subtle vibrato/jitter tuning (applied to oscillator phase increment).
#define OT_VIBRATO_HZ_X100 500
// #define OT_VIBRATO_DEPTH_PPM 2500
#define OT_VIBRATO_DEPTH_PPM 25000
#define OT_JITTER_DEPTH_PPM 0

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
