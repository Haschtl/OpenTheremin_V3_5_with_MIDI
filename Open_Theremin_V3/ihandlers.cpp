#include "Arduino.h"

#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "pins.h"
#include "wavetables.h"
#include "error_indicator.h"
#include "debug_log.h"

#include "build.h"
#include <math.h>
#include <stdint.h>

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
  // Ordered roughly from warm/dark to bright/edgy.
  sine_table,
  sine_table6,
  sine_table5,
  sine_table12,  // Cello
  sine_table9,   // Triangle-Vocal
  sine_table3,
  sine_table4,
  sine_table11,  // Clarinet-like
  sine_table8,
  sine_table7,
  sine_table10,
  sine_table2    // Phoenix (brightest)
};

volatile uint16_t vScaledVolume = 0;
volatile uint16_t vPointerIncrement = 0;

volatile uint32_t pitch = 0;
volatile uint32_t pitch_counter = 0;
volatile uint32_t pitch_counter_l = 0;

volatile bool volumeValueAvailable = 0;
volatile bool pitchValueAvailable = 0;
volatile bool reenableInt1 = 0;

volatile uint32_t vol;
volatile uint32_t vol_counter = 0;
volatile uint32_t vol_counter_i = 0;
volatile uint32_t vol_counter_l;

volatile uint8_t vWavetableSelector = 0;

static volatile uint16_t pointer = 0;
static volatile uint8_t debounce_p = 0;
static volatile uint8_t debounce_v = 0;
static volatile uint16_t wavetableMorphQ8 = 0;
static volatile uint16_t wavetableMorphTargetQ8 = 0;
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
static volatile uint16_t masterOutGainQ8 = 256;
static volatile uint16_t outputFadeGateQ8 = 0;
static volatile uint32_t smoothedScaledVolumeQ8 = 0;
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
static volatile uint32_t pitch_capture_counter_i = 0;
static volatile uint32_t wave_tick_count = 0;
static volatile uint32_t pitch_capture_count = 0;
static volatile uint32_t volume_capture_count = 0;
static volatile uint32_t pitch_capture_raw_last = 0;
static volatile uint32_t volume_capture_raw_last = 0;
static volatile bool pitch_capture_seen = false;
static volatile bool volume_capture_seen = false;

static FspTimer waveTimer;
static bool waveTimerStarted = false;
static uint8_t waveTimerType = (OT_TIMER_PREFER_AGT != 0) ? AGT_TIMER : GPT_TIMER;
static int8_t waveTimerChannel = -1;
static volatile uint32_t currentAudioTickHz = OT_AUDIO_TICK_HZ;
static FspTimer pitchCaptureTimer;
static FspTimer volumeCaptureTimer;
static bool pitchCaptureStarted = false;
static bool volumeCaptureStarted = false;
static uint8_t pitchCaptureChannel = 0xFF;
static uint8_t volumeCaptureChannel = 0xFF;
static bool pitchCaptureUseA = true;
static bool volumeCaptureUseA = true;
static uint32_t pitchCaptureDeltaMask = 0xFFFFFFFFUL;
static uint32_t volumeCaptureDeltaMask = 0xFFFFFFFFUL;

#if OT_TIMER_CB_HAS_ARGS
static void onPitchCaptureTimer(timer_callback_args_t *p_args);
static void onVolumeCaptureTimer(timer_callback_args_t *p_args);
#endif

static void releaseWaveTimerResource() {
  // `close()` alone does not mark FspTimer channel as free in the core.
  // `end()` releases the channel allocation bookkeeping.
  waveTimer.end();
  waveTimerStarted = false;
}

static inline bool isGpt16BitChannel(uint8_t channel) {
  return channel >= GTP32_HOWMANY;
}

static inline uint32_t capturePeriodCountsForChannel(uint8_t channel) {
  return isGpt16BitChannel(channel) ? 65535UL : 0xFFFFFFFFUL;
}

static inline uint32_t normalizeCaptureDelta(uint32_t now, uint32_t last, uint32_t deltaMask) {
  return (now - last) & deltaMask;
}

static bool setupCapturePinAndChannel(uint8_t pin, uint8_t &channel, bool &useA) {
  const auto cfgs = getPinCfgs(pin, PIN_CFG_REQ_PWM);
  if (cfgs[0] == 0U) {
    return false;
  }
  channel = (uint8_t)GET_CHANNEL(cfgs[0]);
  useA = IS_PWM_ON_A(cfgs[0]);
  const uint32_t pinCfg = (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1);
  return (R_IOPORT_PinCfg(&g_ioport_ctrl, digitalPinToBspPin(pin), pinCfg) == FSP_SUCCESS);
}

static void configureCapturePinsAsGpioInput() {
  pinMode(OT_VOLUME_CAPTURE_PIN, INPUT);
  pinMode(OT_PITCH_CAPTURE_PIN, INPUT);
}

static bool configureCaptureSource(FspTimer &timer, bool useA) {
  if (useA) {
    const gpt_source_t src = (gpt_source_t)(
      GPT_SOURCE_GTIOCA_RISING_WHILE_GTIOCB_LOW |
      GPT_SOURCE_GTIOCA_RISING_WHILE_GTIOCB_HIGH
    );
    return timer.set_source_capture_a(src) && timer.setup_capture_a_irq();
  }
  const gpt_source_t src = (gpt_source_t)(
    GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_LOW |
    GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_HIGH
  );
  return timer.set_source_capture_b(src) && timer.setup_capture_b_irq();
}

static bool startCaptureTimer(FspTimer &timer,
                              bool &started,
                              uint8_t pin,
                              uint8_t &channel,
                              bool &useA,
                              uint32_t &deltaMask,
                              GPTimerCbk_f callback,
                              const char *label) {
  if (started) {
    return true;
  }

  if (!setupCapturePinAndChannel(pin, channel, useA)) {
    OT_DEBUG_PRINT("[DBG] capture ");
    OT_DEBUG_PRINT(label);
    OT_DEBUG_PRINTLN_F(": pin cfg failed");
    setErrorIndicator(OT_ERR_TIMER);
    return false;
  }

  deltaMask = isGpt16BitChannel(channel) ? 0x0000FFFFUL : 0xFFFFFFFFUL;
  const uint32_t periodCounts = capturePeriodCountsForChannel(channel);

  OT_DEBUG_PRINT("[DBG] capture ");
  OT_DEBUG_PRINT(label);
  OT_DEBUG_PRINT(" timer: GPT ch=");
  OT_DEBUG_PRINT((int)channel);
  OT_DEBUG_PRINT(" line=");
  OT_DEBUG_PRINTLN(useA ? "A" : "B");

  // Capture pins can be on channels pre-marked as PWM-reserved by the core (e.g. D8 on GPT7).
  // We intentionally claim that channel for input-capture in this project.
  FspTimer::force_use_of_pwm_reserved_timer();
  if (!timer.begin(TIMER_MODE_PERIODIC,
                   GPT_TIMER,
                   channel,
                   periodCounts,
                   0UL,
                   TIMER_SOURCE_DIV_1,
                   callback,
                   nullptr)) {
    OT_DEBUG_PRINT("[DBG] capture ");
    OT_DEBUG_PRINT(label);
    OT_DEBUG_PRINTLN_F(": begin failed");
    setErrorIndicator(OT_ERR_TIMER);
    timer.end();
    return false;
  }

  if (!configureCaptureSource(timer, useA)) {
    OT_DEBUG_PRINT("[DBG] capture ");
    OT_DEBUG_PRINT(label);
    OT_DEBUG_PRINTLN_F(": capture source/irq failed");
    setErrorIndicator(OT_ERR_TIMER);
    timer.end();
    return false;
  }

  if (!timer.open()) {
    OT_DEBUG_PRINT("[DBG] capture ");
    OT_DEBUG_PRINT(label);
    OT_DEBUG_PRINTLN_F(": open failed");
    setErrorIndicator(OT_ERR_TIMER);
    timer.end();
    return false;
  }

  if (!timer.start()) {
    OT_DEBUG_PRINT("[DBG] capture ");
    OT_DEBUG_PRINT(label);
    OT_DEBUG_PRINTLN_F(": start failed");
    setErrorIndicator(OT_ERR_TIMER);
    timer.end();
    return false;
  }

  started = true;
  return true;
}

static void stopCaptureTimer(FspTimer &timer, bool &started) {
  if (started) {
    (void)timer.stop();
  }
  timer.end();
  started = false;
}

static void stopCaptureTimers() {
  stopCaptureTimer(pitchCaptureTimer, pitchCaptureStarted);
  stopCaptureTimer(volumeCaptureTimer, volumeCaptureStarted);
}

static bool startCaptureTimers() {
#if !OT_TIMER_CB_HAS_ARGS
  #error "GPT input capture requires timer_callback_args_t support on UNO R4."
#endif
  if (!startCaptureTimer(pitchCaptureTimer,
                         pitchCaptureStarted,
                         OT_PITCH_CAPTURE_PIN,
                         pitchCaptureChannel,
                         pitchCaptureUseA,
                         pitchCaptureDeltaMask,
                         onPitchCaptureTimer,
                         "pitch")) {
    return false;
  }

  if (!startCaptureTimer(volumeCaptureTimer,
                         volumeCaptureStarted,
                         OT_VOLUME_CAPTURE_PIN,
                         volumeCaptureChannel,
                         volumeCaptureUseA,
                         volumeCaptureDeltaMask,
                         onVolumeCaptureTimer,
                         "volume")) {
    stopCaptureTimer(pitchCaptureTimer, pitchCaptureStarted);
    return false;
  }

  return true;
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

static inline float polyBlep(float t, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }
  if (t < dt) {
    t /= dt;
    return (t + t) - (t * t) - 1.0f;
  }
  if (t > (1.0f - dt)) {
    t = (t - 1.0f) / dt;
    return (t * t) + (t + t) + 1.0f;
  }
  return 0.0f;
}

static inline int16_t readPolyBlepSample(uint8_t mode, float phase, float dt) {
  float y = 0.0f;
  if (mode == OT_WAVETABLE_TABLE_COUNT) {
    // PolyBLEP saw
    y = (2.0f * phase) - 1.0f;
    y -= polyBlep(phase, dt);
  } else {
    // PolyBLEP pulse (50% duty)
    y = (phase < 0.5f) ? 1.0f : -1.0f;
    y += polyBlep(phase, dt);
    float t2 = phase + 0.5f;
    if (t2 >= 1.0f) {
      t2 -= 1.0f;
    }
    y -= polyBlep(t2, dt);
  }

  int32_t s = (int32_t)(y * 1900.0f);
  if (s > 2047) s = 2047;
  if (s < -2048) s = -2048;
  return (int16_t)s;
}

static inline int16_t readWaveSampleByIndex(uint8_t idx, uint16_t offset, uint8_t frac, float phase, float dt) {
  if (idx < OT_WAVETABLE_TABLE_COUNT) {
    return readInterpolatedSample(wavetables[idx], offset, frac);
  }
  return readPolyBlepSample(idx, phase, dt);
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

static inline uint16_t pitchLoudnessGainQ8(uint16_t absIncrement) {
#if OT_PITCH_LOUDNESS_COMP_ENABLE
  if (absIncrement <= OT_PITCH_LOUDNESS_COMP_START_INCREMENT) {
    return 256U;
  }
  uint16_t atten = (uint16_t)((absIncrement - OT_PITCH_LOUDNESS_COMP_START_INCREMENT) >> OT_PITCH_LOUDNESS_COMP_SLOPE_SHIFT);
  uint16_t maxAtten = OT_PITCH_LOUDNESS_COMP_MAX_ATTEN_Q8;
  if (maxAtten > 255U) {
    maxAtten = 255U;
  }
  if (atten > maxAtten) {
    atten = maxAtten;
  }
  return (uint16_t)(256U - atten);
#else
  (void)absIncrement;
  return 256U;
#endif
}

static inline void runWaveTick() {
  if (!int1Enabled) {
    return;
  }
  wave_tick_count++;

  SPImcpDAClatch();

  const uint16_t offset = (uint16_t)(pointer >> 6) & 0x3FFU;
  const uint8_t frac = (uint8_t)(pointer & 0x3FU);
  const float phase = (float)pointer / 65536.0f;

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#else
  const uint8_t tableMax = (uint8_t)(OT_WAVETABLE_COUNT - 1U);
  const uint16_t targetMaxQ8 = ((uint16_t)tableMax) << 8;
  uint16_t targetMorphQ8 = wavetableMorphTargetQ8;
  if (targetMorphQ8 > targetMaxQ8) {
    targetMorphQ8 = targetMaxQ8;
  }
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
  const uint8_t tableB = (tableA < tableMax) ? (tableA + 1U) : tableMax;

  const int16_t pointerIncrementSignedForDt = (int16_t)vPointerIncrement;
  uint16_t absIncrementForDt = (pointerIncrementSignedForDt < 0)
                                 ? (uint16_t)(-pointerIncrementSignedForDt)
                                 : (uint16_t)pointerIncrementSignedForDt;
  if (absIncrementForDt == 0) {
    absIncrementForDt = 1;
  }
  float dt = (float)absIncrementForDt / 65536.0f;
  if (dt > 0.45f) {
    dt = 0.45f;
  }

  int32_t waveSample = readWaveSampleByIndex(tableA, offset, frac, phase, dt);
  if (tableBlend > 0U) {
    const int32_t waveSampleB = readWaveSampleByIndex(tableB, offset, frac, phase, dt);
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

  uint32_t scaledVolume = ((uint32_t)vScaledVolume * (uint32_t)masterOutGainQ8) >> 8;
  scaledVolume = (scaledVolume * (uint32_t)outputFadeGateQ8) >> 8;
  scaledVolume = (scaledVolume * (uint32_t)pitchLoudnessGainQ8(absIncrement)) >> 8;
#if OT_AUDIO_VOLUME_SLEW_SHIFT > 0
  {
    uint32_t targetQ8 = scaledVolume << 8;
    int32_t diff = (int32_t)targetQ8 - (int32_t)smoothedScaledVolumeQ8;
    smoothedScaledVolumeQ8 = (uint32_t)((int32_t)smoothedScaledVolumeQ8 + (diff >> OT_AUDIO_VOLUME_SLEW_SHIFT));
    scaledVolume = smoothedScaledVolumeQ8 >> 8;
  }
#endif
  if (scaledVolume > 65535UL) {
    scaledVolume = 65535UL;
  }
  int32_t scaledSample = (waveSample * (int32_t)scaledVolume) >> 16;
  scaledSample = softClip12Bit(scaledSample);
  int32_t dacValue = scaledSample + (int32_t)MCP_DAC_BASE;
  if (dacValue < 0) {
    dacValue = 0;
  } else if (dacValue > 4095) {
    dacValue = 4095;
  }
  SPImcpDACsendPrepared(SPImcpDACformatA((uint16_t)dacValue));

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

}

#if OT_TIMER_CB_HAS_ARGS
static void onWaveTimerTick(timer_callback_args_t *p_args) {
  (void)p_args;
  runWaveTick();
}

static void onPitchCaptureTimer(timer_callback_args_t *p_args) {
  if (p_args == nullptr) {
    return;
  }
  if (p_args->event != TIMER_EVENT_CAPTURE_A && p_args->event != TIMER_EVENT_CAPTURE_B) {
    return;
  }

  pitch_capture_count++;
  const uint32_t captureNow = p_args->capture;
  const uint32_t prev = pitch_capture_raw_last;
  pitch_capture_raw_last = captureNow;

  if (!pitch_capture_seen) {
    pitch_capture_seen = true;
    return;
  }

  const uint32_t delta = normalizeCaptureDelta(captureNow, prev, pitchCaptureDeltaMask);
  if (delta > 0U) {
    pitch_capture_counter_i = delta;
    pitch_counter = delta;
    pitch = delta;
    pitchValueAvailable = true;
  }
}

static void onVolumeCaptureTimer(timer_callback_args_t *p_args) {
  if (p_args == nullptr) {
    return;
  }
  if (p_args->event != TIMER_EVENT_CAPTURE_A && p_args->event != TIMER_EVENT_CAPTURE_B) {
    return;
  }

  volume_capture_count++;
  const uint32_t captureNow = p_args->capture;
  const uint32_t prev = volume_capture_raw_last;
  volume_capture_raw_last = captureNow;

  if (!volume_capture_seen) {
    volume_capture_seen = true;
    return;
  }

  const uint32_t delta = normalizeCaptureDelta(captureNow, prev, volumeCaptureDeltaMask);
  if (delta > 0U) {
    vol_counter_i = delta;
    vol_counter = delta;
    vol = delta;
    volumeValueAvailable = true;
  }
}
#else
static void onWaveTimerTick() {
  runWaveTick();
}
#endif

void ihDisableInt1() {
  int1Enabled = false;
}

void ihEnableInt1() {
  if (reenableInt1) {
    int1Enabled = true;
  }
}

static bool startWaveTimer() {
  if (waveTimerStarted) {
    return true;
  }

  if (waveTimerChannel < 0) {
    // Pick one channel/type once and keep it fixed for all later restarts.
    waveTimerChannel = FspTimer::get_available_timer(waveTimerType);
  }

  if (waveTimerChannel < 0) {
    // No timer resource available.
    OT_DEBUG_PRINTLN_F("[DBG] timer: no channel available");
    setErrorIndicator(OT_ERR_TIMER);
    return false;
  }

  OT_DEBUG_PRINT("[DBG] timer: type=");
  OT_DEBUG_PRINT((waveTimerType == GPT_TIMER) ? "GPT" : "AGT");
  OT_DEBUG_PRINT(" ch=");
  OT_DEBUG_PRINT((int)waveTimerChannel);
  OT_DEBUG_PRINT(" hz=");
  OT_DEBUG_PRINTLN((unsigned long)currentAudioTickHz);

  OT_DEBUG_PRINTLN_F("[DBG] timer: begin");
  if (!waveTimer.begin(TIMER_MODE_PERIODIC, waveTimerType, waveTimerChannel, (float)currentAudioTickHz, 0.0f, onWaveTimerTick)) {
    OT_DEBUG_PRINTLN_F("[DBG] timer: begin failed");
    setErrorIndicator(OT_ERR_TIMER);
    releaseWaveTimerResource();
    return false;
  }

  OT_DEBUG_PRINTLN_F("[DBG] timer: setup irq");
  if (!waveTimer.setup_overflow_irq()) {
    OT_DEBUG_PRINTLN_F("[DBG] timer: setup irq failed");
    setErrorIndicator(OT_ERR_TIMER);
    releaseWaveTimerResource();
    return false;
  }

  OT_DEBUG_PRINTLN_F("[DBG] timer: open");
  if (!waveTimer.open()) {
    OT_DEBUG_PRINTLN_F("[DBG] timer: open failed");
    setErrorIndicator(OT_ERR_TIMER);
    releaseWaveTimerResource();
    return false;
  }

  OT_DEBUG_PRINTLN_F("[DBG] timer: start");
  if (!waveTimer.start()) {
    OT_DEBUG_PRINTLN_F("[DBG] timer: start failed");
    setErrorIndicator(OT_ERR_TIMER);
    releaseWaveTimerResource();
    return false;
  }

  waveTimerStarted = true;
  if (getErrorIndicator() == OT_ERR_TIMER) {
    clearErrorIndicator();
  }
  OT_DEBUG_PRINTLN_F("[DBG] timer: started");
  return true;
}

static void stopWaveTimer() {
  if (waveTimerStarted) {
    (void)waveTimer.stop();
  }
  releaseWaveTimerResource();
}

void ihInitialiseTimer() {
  // Timer resources are managed by startWaveTimer/stopWaveTimer.
}

void ihInitialiseInterrupts() {
  reenableInt1 = true;
  int1Enabled = true;
  {
    const uint8_t tableMax = (uint8_t)(OT_WAVETABLE_COUNT - 1U);
    const uint8_t startTable = (vWavetableSelector > tableMax) ? tableMax : vWavetableSelector;
    wavetableMorphQ8 = ((uint16_t)startTable) << 8;
    wavetableMorphTargetQ8 = wavetableMorphQ8;
  }
  biquadZ1 = 0;
  biquadZ2 = 0;
  rebuildBiquadTable();
  updateVibratoRate();
  vibratoPhaseQ32 = 0;
  noInterrupts();
  wave_tick_count = 0;
  pitch_capture_count = 0;
  volume_capture_count = 0;
  pitch_capture_raw_last = 0;
  volume_capture_raw_last = 0;
  pitch_capture_seen = false;
  volume_capture_seen = false;
  pitchValueAvailable = false;
  volumeValueAvailable = false;
  interrupts();
  OT_DEBUG_PRINTLN_F("[DBG] capture source=GPT input capture");
  if (!startCaptureTimers()) {
    return;
  }
  (void)startWaveTimer();
}

void ihInitialisePitchMeasurement() {
  reenableInt1 = false;
  int1Enabled = false;
  stopWaveTimer();
  stopCaptureTimers();
  configureCapturePinsAsGpioInput();
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
  if (hz == currentAudioTickHz) {
    return true;
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
    if (!startWaveTimer()) {
      return false;
    }
  }

  return true;
}

void ihRecoverTimer() {
  static uint32_t lastAttemptMs = 0;
  static uint32_t lastTickCount = 0;
  static uint32_t lastPitchCaptureCount = 0;
  static uint32_t lastVolumeCaptureCount = 0;
  static uint32_t lastProgressCheckMs = 0;
  if (!reenableInt1) {
    return;
  }

  const uint32_t now = millis();

  if (!waveTimerStarted) {
    if ((uint32_t)(now - lastAttemptMs) < 200U) {
      return;
    }
    lastAttemptMs = now;
    OT_DEBUG_PRINTLN_F("[DBG] timer: recover attempt");
    (void)startWaveTimer();
    return;
  }

  // Detect stalled capture IRQs while timer still runs and auto-recover capture timers.
  if ((uint32_t)(now - lastProgressCheckMs) < 600U) {
    return;
  }
  lastProgressCheckMs = now;

  noInterrupts();
  const uint32_t ticksNow = wave_tick_count;
  const uint32_t pitchCapNow = pitch_capture_count;
  const uint32_t volCapNow = volume_capture_count;
  interrupts();

  const bool waveAdvancing = (ticksNow != lastTickCount);
  const bool pitchAdvancing = (pitchCapNow != lastPitchCaptureCount);
  const bool volAdvancing = (volCapNow != lastVolumeCaptureCount);

  if (!waveAdvancing) {
    OT_DEBUG_PRINTLN_F("[DBG] timer: stalled, restart");
    stopWaveTimer();
    (void)startWaveTimer();

    // Re-sample after forced restart.
    noInterrupts();
    lastTickCount = wave_tick_count;
    lastPitchCaptureCount = pitch_capture_count;
    lastVolumeCaptureCount = volume_capture_count;
    interrupts();
    return;
  }

  if (waveAdvancing && (!pitchAdvancing || !volAdvancing)) {
    OT_DEBUG_PRINT("[DBG] capture: restart pitch=");
    OT_DEBUG_PRINT((unsigned long)pitchCapNow);
    OT_DEBUG_PRINT(" vol=");
    OT_DEBUG_PRINTLN((unsigned long)volCapNow);
    stopCaptureTimers();
    noInterrupts();
    pitch_capture_raw_last = 0;
    volume_capture_raw_last = 0;
    pitch_capture_seen = false;
    volume_capture_seen = false;
    interrupts();
    (void)startCaptureTimers();
  }

  lastTickCount = ticksNow;
  lastPitchCaptureCount = pitchCapNow;
  lastVolumeCaptureCount = volCapNow;
}

uint32_t ihGetWaveTickCount() {
  noInterrupts();
  const uint32_t v = wave_tick_count;
  interrupts();
  return v;
}

uint32_t ihGetPitchCaptureCount() {
  noInterrupts();
  const uint32_t v = pitch_capture_count;
  interrupts();
  return v;
}

uint32_t ihGetVolumeCaptureCount() {
  noInterrupts();
  const uint32_t v = volume_capture_count;
  interrupts();
  return v;
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

void ihSetWaveMorphTargetQ8(uint16_t targetQ8) {
  const uint8_t tableMax = (uint8_t)(OT_WAVETABLE_COUNT - 1U);
  const uint16_t targetMaxQ8 = ((uint16_t)tableMax) << 8;
  if (targetQ8 > targetMaxQ8) {
    targetQ8 = targetMaxQ8;
  }
  noInterrupts();
  wavetableMorphTargetQ8 = targetQ8;
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

void ihSetMasterOutGainQ8(uint16_t gainQ8) {
  if (gainQ8 < 26U) {
    gainQ8 = 26U;
  } else if (gainQ8 > 256U) {
    gainQ8 = 256U;
  }
  noInterrupts();
  masterOutGainQ8 = gainQ8;
  interrupts();
}

uint16_t ihGetMasterOutGainQ8() {
  return masterOutGainQ8;
}

void ihSetOutputFadeGateQ8(uint16_t gateQ8) {
  if (gateQ8 > 256U) {
    gateQ8 = 256U;
  }
  noInterrupts();
  outputFadeGateQ8 = gateQ8;
  interrupts();
}

uint16_t ihGetOutputFadeGateQ8() {
  return outputFadeGateQ8;
}
