#include "Arduino.h"

#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "pins.h"
#include "wavetables.h"

#include "build.h"
#include <math.h>

#if __has_include(<FspTimer.h>)
  #include <FspTimer.h>
#else
  #error "FspTimer.h is required. This firmware only supports the UNO R4 hardware-timer path."
#endif

#if __has_include(<r_timer_api.h>)
  #include <r_timer_api.h>
  #define OT_TIMER_CB_HAS_ARGS 1
#else
  #define OT_TIMER_CB_HAS_ARGS 0
#endif

static const uint32_t MCP_DAC_BASE = 2048;
static const uint8_t BIQUAD_COEF_SHIFT = 14;
static const uint8_t BIQUAD_TABLE_SIZE = 32;

const int16_t* const wavetables[] = {
  sine_table,
  sine_table2,
  sine_table3,
  sine_table4,
  sine_table5,
  sine_table6,
  sine_table7,
  sine_table8
};

volatile uint16_t vScaledVolume = 0;
volatile uint16_t vPointerIncrement = 0;

volatile uint16_t pitch = 0;
volatile uint16_t pitch_counter = 0;
volatile uint16_t pitch_counter_l = 0;

volatile bool volumeValueAvailable = 0;
volatile bool pitchValueAvailable = 0;
volatile bool reenableInt1 = 0;

volatile uint16_t vol;
volatile uint16_t vol_counter = 0;
volatile uint16_t vol_counter_i = 0;
volatile uint16_t vol_counter_l;

volatile uint8_t vWavetableSelector = 0;

static volatile uint16_t pointer = 0;
static volatile uint8_t debounce_p = 0;
static volatile uint8_t debounce_v = 0;
static volatile uint16_t wavetableMorphQ8 = 0;
static volatile bool waveMorphEnabled = (OT_WAVEMORPH_ENABLE_DEFAULT != 0);
static volatile bool toneTiltEnabled = (OT_TILT_ENABLE_DEFAULT != 0);
static volatile bool softClipEnabled = (OT_SOFTCLIP_ENABLE_DEFAULT != 0);
static volatile bool vibratoJitterEnabled = (OT_VIBRATO_ENABLE_DEFAULT != 0);
static volatile uint8_t waveMorphStepQ8 = (OT_WAVEMORPH_STEP_Q8 == 0) ? 1 : OT_WAVEMORPH_STEP_Q8;
static volatile uint8_t toneTiltWetMax = (OT_TILT_WET_MAX > 255) ? 255 : OT_TILT_WET_MAX;
static volatile uint8_t softClipCubicShift = OT_SOFTCLIP_CUBIC_SHIFT;
static volatile uint32_t vibratoPhaseQ32 = 0;
static volatile uint32_t vibratoRateQ32 = 0;
static volatile uint32_t jitterState = 0x1234ABCDUL;
static volatile int32_t biquadZ1 = 0;
static volatile int32_t biquadZ2 = 0;

struct BiquadCoeff {
  int32_t b0;
  int32_t b1;
  int32_t b2;
  int32_t a1;
  int32_t a2;
};

static BiquadCoeff biquadTable[BIQUAD_TABLE_SIZE];

static volatile bool int1Enabled = false;
static volatile uint16_t pitch_capture_counter_i = 0;

static FspTimer waveTimer;
static bool waveTimerStarted = false;
static uint8_t waveTimerType = GPT_TIMER;
static int8_t waveTimerChannel = -1;
static volatile uint32_t currentAudioTickHz = OT_AUDIO_TICK_HZ;

static inline uint16_t readTimerCounter16() {
  // 1MHz monotonic source converted to ~16MHz virtual ticks for legacy math.
  return (uint16_t)((micros() * 16UL) & 0xFFFF);
}

static inline void updateVibratoRate() {
  const uint32_t hz = currentAudioTickHz;
  if (hz == 0 || OT_VIBRATO_HZ_X100 == 0) {
    vibratoRateQ32 = 0;
    return;
  }
  const uint64_t num = (uint64_t)OT_VIBRATO_HZ_X100 * 4294967296ULL;
  const uint64_t den = (uint64_t)hz * 100ULL;
  vibratoRateQ32 = (uint32_t)(num / den);
}

static inline int16_t nextJitterSample() {
  jitterState = jitterState * 1664525UL + 1013904223UL;
  return (int16_t)(jitterState >> 16);
}

static inline void buildLowpassBiquad(float cutoffHz, float sampleHz, float q, BiquadCoeff &out) {
  if (cutoffHz < 40.0f) {
    cutoffHz = 40.0f;
  }
  const float nyquist = sampleHz * 0.5f;
  if (cutoffHz > nyquist * 0.45f) {
    cutoffHz = nyquist * 0.45f;
  }

  const float w0 = 2.0f * 3.14159265358979323846f * cutoffHz / sampleHz;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);

  const float a0 = 1.0f + alpha;
  const float b0 = ((1.0f - cw) * 0.5f) / a0;
  const float b1 = (1.0f - cw) / a0;
  const float b2 = ((1.0f - cw) * 0.5f) / a0;
  const float a1 = (-2.0f * cw) / a0;
  const float a2 = (1.0f - alpha) / a0;

  const float scale = (float)(1 << BIQUAD_COEF_SHIFT);
  out.b0 = (int32_t)(b0 * scale);
  out.b1 = (int32_t)(b1 * scale);
  out.b2 = (int32_t)(b2 * scale);
  out.a1 = (int32_t)(a1 * scale);
  out.a2 = (int32_t)(a2 * scale);
}

static inline void rebuildBiquadTable() {
  const float fs = (float)currentAudioTickHz;
  if (fs < 1000.0f) {
    for (uint8_t i = 0; i < BIQUAD_TABLE_SIZE; ++i) {
      biquadTable[i].b0 = (1 << BIQUAD_COEF_SHIFT);
      biquadTable[i].b1 = 0;
      biquadTable[i].b2 = 0;
      biquadTable[i].a1 = 0;
      biquadTable[i].a2 = 0;
    }
    return;
  }
  const float q = ((OT_TILT_BIQUAD_Q_X1000 <= 0) ? 707.0f : (float)OT_TILT_BIQUAD_Q_X1000) / 1000.0f;
  const float cutoffMax = (OT_TILT_CUTOFF_MAX_HZ < 200) ? 200.0f : (float)OT_TILT_CUTOFF_MAX_HZ;
  const float cutoffMinRaw = (OT_TILT_CUTOFF_MIN_HZ < 80) ? 80.0f : (float)OT_TILT_CUTOFF_MIN_HZ;
  const float cutoffMin = (cutoffMinRaw > cutoffMax) ? cutoffMax : cutoffMinRaw;

  for (uint8_t i = 0; i < BIQUAD_TABLE_SIZE; ++i) {
    const float t = (float)i / (float)(BIQUAD_TABLE_SIZE - 1);
    // Higher pitch index => lower cutoff for more classic theremin darkening.
    const float cutoff = cutoffMax - ((cutoffMax - cutoffMin) * t);
    buildLowpassBiquad(cutoff, fs, q, biquadTable[i]);
  }
}

static inline int32_t processTiltBiquad(int32_t x, const BiquadCoeff &c) {
  const int32_t yQ = c.b0 * x + biquadZ1;
  int32_t y = yQ >> BIQUAD_COEF_SHIFT;
  biquadZ1 = c.b1 * x - c.a1 * y + biquadZ2;
  biquadZ2 = c.b2 * x - c.a2 * y;
  if (y > 4095) {
    y = 4095;
  } else if (y < -4096) {
    y = -4096;
  }
  return y;
}

static inline int16_t readInterpolatedSample(const int16_t *table, uint16_t index, uint8_t frac) {
  const uint16_t nextIndex = (index + 1U) & 0x3FFU;
  const int32_t a = table[index];
  const int32_t b = table[nextIndex];
  return (int16_t)(a + (((b - a) * (int32_t)frac) >> 6));
}

static inline int16_t softClip12Bit(int32_t x) {
  if (x > 2047) {
    x = 2047;
  } else if (x < -2048) {
    x = -2048;
  }

  int32_t y = x;
  if (softClipEnabled) {
    const int64_t cubic = (int64_t)x * (int64_t)x * (int64_t)x;
    y = x - (int32_t)(cubic >> softClipCubicShift);  // gentle cubic soft clip
  }

  if (y > 2047) {
    y = 2047;
  } else if (y < -2048) {
    y = -2048;
  }
  return (int16_t)y;
}

static inline void runWaveTick() {
  if (!int1Enabled) {
    return;
  }

  SPImcpDAClatch();

  const uint16_t offset = (uint16_t)(pointer >> 6) & 0x3FFU;
  const uint8_t frac = (uint8_t)(pointer & 0x3FU);

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#else
  const uint16_t targetMorphQ8 = ((uint16_t)(vWavetableSelector & 0x07U)) << 8;
  if (waveMorphEnabled) {
    const uint16_t morphStepQ8 = (waveMorphStepQ8 == 0) ? 1U : (uint16_t)waveMorphStepQ8;
    if (wavetableMorphQ8 < targetMorphQ8) {
      const uint16_t delta = targetMorphQ8 - wavetableMorphQ8;
      wavetableMorphQ8 += (delta > morphStepQ8) ? morphStepQ8 : delta;
    } else if (wavetableMorphQ8 > targetMorphQ8) {
      const uint16_t delta = wavetableMorphQ8 - targetMorphQ8;
      wavetableMorphQ8 -= (delta > morphStepQ8) ? morphStepQ8 : delta;
    }
  } else {
    wavetableMorphQ8 = targetMorphQ8;
  }

  const uint8_t tableA = (uint8_t)(wavetableMorphQ8 >> 8);
  const uint8_t tableBlend = (uint8_t)(wavetableMorphQ8 & 0xFFU);
  const uint8_t tableB = (tableA < 7U) ? (tableA + 1U) : 7U;

  int32_t waveSample = readInterpolatedSample(wavetables[tableA], offset, frac);
  if (tableBlend > 0U) {
    const int32_t waveSampleB = readInterpolatedSample(wavetables[tableB], offset, frac);
    waveSample += ((waveSampleB - waveSample) * (int32_t)tableBlend) >> 8;
  }

  const int16_t pointerIncrementSigned = (int16_t)vPointerIncrement;
  const uint16_t absIncrement = (pointerIncrementSigned < 0)
                                  ? (uint16_t)(-pointerIncrementSigned)
                                  : (uint16_t)pointerIncrementSigned;

  if (toneTiltEnabled) {
    uint8_t biquadIdx = (uint8_t)(absIncrement >> 7);
    if (biquadIdx >= BIQUAD_TABLE_SIZE) {
      biquadIdx = BIQUAD_TABLE_SIZE - 1;
    }
    const int32_t filtered = processTiltBiquad(waveSample, biquadTable[biquadIdx]);

    uint8_t wet = 0;
    if (absIncrement > OT_TILT_START_INCREMENT) {
      const uint16_t wetRaw = (uint16_t)((absIncrement - OT_TILT_START_INCREMENT) >> 3);
      const uint16_t wetMax = (uint16_t)toneTiltWetMax;
      wet = (wetRaw > wetMax) ? (uint8_t)wetMax : (uint8_t)wetRaw;
    }
    waveSample += ((filtered - waveSample) * (int32_t)wet) >> 8;
  } else {
    biquadZ1 = 0;
    biquadZ2 = 0;
  }

  int32_t scaledSample = (waveSample * (int32_t)vScaledVolume) >> 16;
  scaledSample = softClip12Bit(scaledSample);
  int32_t dacValue = scaledSample + (int32_t)MCP_DAC_BASE;
  if (dacValue < 0) {
    dacValue = 0;
  } else if (dacValue > 4095) {
    dacValue = 4095;
  }
#if OT_USE_DMA
  interrupts();
#endif
  SPImcpDACsendPrepared(SPImcpDACformatA((uint16_t)dacValue));
#if OT_USE_DMA
  noInterrupts();
#endif

  uint16_t phaseIncrement = vPointerIncrement;
  if (vibratoJitterEnabled) {
    vibratoPhaseQ32 += vibratoRateQ32;

    const uint16_t saw = (uint16_t)(vibratoPhaseQ32 >> 16);
    const uint16_t tri = (saw < 32768U) ? (uint16_t)(saw << 1) : (uint16_t)((65535U - saw) << 1);
    const int32_t triSigned = (int32_t)tri - 32768;
    const int32_t vibPpm = (triSigned * (int32_t)OT_VIBRATO_DEPTH_PPM) / 32768;

    const int32_t jitPpm = ((int32_t)nextJitterSample() * (int32_t)OT_JITTER_DEPTH_PPM) / 32768;
    const int32_t modPpm = vibPpm + jitPpm;

    const int32_t base = (int16_t)vPointerIncrement;
    int32_t modulated = base + (int32_t)(((int64_t)base * (int64_t)modPpm) / 1000000LL);
    if (modulated > 32767) {
      modulated = 32767;
    } else if (modulated < -32768) {
      modulated = -32768;
    }
    phaseIncrement = (uint16_t)((int16_t)modulated);
  }

  pointer += phaseIncrement;
#endif

  incrementTimer();
  incrementMidiTimer();

  debounce_p++;
  if (debounce_p == 3) {
    pitch_counter = pitch_capture_counter_i;
    pitch = (pitch_counter - pitch_counter_l);
    pitch_counter_l = pitch_counter;
  }
  if (debounce_p == 5) {
    pitchValueAvailable = true;
  }

  debounce_v++;
  if (debounce_v == 3) {
    vol_counter = vol_counter_i;
    vol = (vol_counter - vol_counter_l);
    vol_counter_l = vol_counter;
  }
  if (debounce_v == 5) {
    volumeValueAvailable = true;
  }
}

#if OT_TIMER_CB_HAS_ARGS
static void onWaveTimerTick(timer_callback_args_t *p_args) {
  (void)p_args;
  runWaveTick();
}
#else
static void onWaveTimerTick() {
  runWaveTick();
}
#endif

static void onPitchCapture() {
  pitch_capture_counter_i = readTimerCounter16();
  debounce_p = 0;
}

static void onVolumeCapture() {
  vol_counter_i = readTimerCounter16();
  debounce_v = 0;
}

void ihDisableInt1() {
  int1Enabled = false;
}

void ihEnableInt1() {
  if (reenableInt1) {
    int1Enabled = true;
  }
}

static void attachCaptureInterrupts() {
  const pin_size_t volumeInterrupt = digitalPinToInterrupt(OT_VOLUME_CAPTURE_PIN);
  const pin_size_t pitchInterrupt = digitalPinToInterrupt(OT_PITCH_CAPTURE_PIN);
  attachInterrupt(volumeInterrupt, onVolumeCapture, RISING);
  attachInterrupt(pitchInterrupt, onPitchCapture, RISING);
}

static void detachCaptureInterrupts() {
  const pin_size_t volumeInterrupt = digitalPinToInterrupt(OT_VOLUME_CAPTURE_PIN);
  const pin_size_t pitchInterrupt = digitalPinToInterrupt(OT_PITCH_CAPTURE_PIN);
  detachInterrupt(volumeInterrupt);
  detachInterrupt(pitchInterrupt);
}

static void startWaveTimer() {
  if (waveTimerStarted) {
    return;
  }

  if (waveTimerChannel < 0) {
    waveTimerChannel = FspTimer::get_available_timer(waveTimerType);
  }

  if (waveTimerChannel < 0) {
    // Hard fail by design: no fallback timer source allowed.
    for (;;) { }
  }

  waveTimer.begin(TIMER_MODE_PERIODIC, waveTimerType, waveTimerChannel, (float)currentAudioTickHz, 0.0f, onWaveTimerTick);
  waveTimer.setup_overflow_irq();
  waveTimer.open();
  waveTimer.start();
  waveTimerStarted = true;
}

static void stopWaveTimer() {
  if (!waveTimerStarted) {
    return;
  }

  waveTimer.stop();
  waveTimer.close();
  waveTimerStarted = false;
}

void ihInitialiseTimer() {
  // Timer resources are managed by startWaveTimer/stopWaveTimer.
}

void ihInitialiseInterrupts() {
  reenableInt1 = true;
  int1Enabled = true;
  wavetableMorphQ8 = ((uint16_t)(vWavetableSelector & 0x07U)) << 8;
  biquadZ1 = 0;
  biquadZ2 = 0;
  rebuildBiquadTable();
  updateVibratoRate();
  vibratoPhaseQ32 = 0;
  attachCaptureInterrupts();
  startWaveTimer();
}

void ihInitialisePitchMeasurement() {
  reenableInt1 = false;
  int1Enabled = false;
  stopWaveTimer();
  detachCaptureInterrupts();
}

void ihInitialiseVolumeMeasurement() {
  ihInitialisePitchMeasurement();
}

uint32_t ihGetAudioTickHz() {
  return currentAudioTickHz;
}

bool ihSetAudioTickHz(uint32_t hz) {
  if (hz < OT_AUDIO_TICK_HZ_MIN || hz > OT_AUDIO_TICK_HZ_MAX) {
    return false;
  }

  const bool restart = waveTimerStarted;
  if (restart) {
    stopWaveTimer();
  }

  noInterrupts();
  currentAudioTickHz = hz;
  timer = 0;
  midi_timer = 0;
  interrupts();
  rebuildBiquadTable();
  updateVibratoRate();

  if (restart) {
    startWaveTimer();
  }

  return true;
}

void ihSetWaveMorphEnabled(bool enabled) {
  noInterrupts();
  waveMorphEnabled = enabled;
  interrupts();
}

void ihSetToneTiltEnabled(bool enabled) {
  noInterrupts();
  toneTiltEnabled = enabled;
  interrupts();
}

void ihSetSoftClipEnabled(bool enabled) {
  noInterrupts();
  softClipEnabled = enabled;
  interrupts();
}

void ihSetWaveMorphStepQ8(uint8_t stepQ8) {
  noInterrupts();
  waveMorphStepQ8 = (stepQ8 == 0) ? 1 : stepQ8;
  interrupts();
}

void ihSetToneTiltWetMax(uint8_t wetMax) {
  noInterrupts();
  toneTiltWetMax = wetMax;
  interrupts();
}

void ihSetSoftClipCubicShift(uint8_t cubicShift) {
  if (cubicShift < 20) {
    cubicShift = 20;
  } else if (cubicShift > 30) {
    cubicShift = 30;
  }
  noInterrupts();
  softClipCubicShift = cubicShift;
  interrupts();
}

void ihSetVibratoJitterEnabled(bool enabled) {
  noInterrupts();
  vibratoJitterEnabled = enabled;
  interrupts();
}

bool ihGetWaveMorphEnabled() {
  return waveMorphEnabled;
}

bool ihGetToneTiltEnabled() {
  return toneTiltEnabled;
}

bool ihGetSoftClipEnabled() {
  return softClipEnabled;
}

bool ihGetVibratoJitterEnabled() {
  return vibratoJitterEnabled;
}

uint8_t ihGetWaveMorphStepQ8() {
  return waveMorphStepQ8;
}

uint8_t ihGetToneTiltWetMax() {
  return toneTiltWetMax;
}

uint8_t ihGetSoftClipCubicShift() {
  return softClipCubicShift;
}
