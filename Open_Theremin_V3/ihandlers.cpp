#include "Arduino.h"

#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "pins.h"
#include "wavetables.h"

#include "build.h"

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
static volatile int32_t toneFilterState = 0;
static volatile bool waveMorphEnabled = (OT_WAVEMORPH_ENABLE_DEFAULT != 0);
static volatile bool toneTiltEnabled = (OT_TILT_ENABLE_DEFAULT != 0);
static volatile bool softClipEnabled = (OT_SOFTCLIP_ENABLE_DEFAULT != 0);
static volatile uint8_t waveMorphStepQ8 = (OT_WAVEMORPH_STEP_Q8 == 0) ? 1 : OT_WAVEMORPH_STEP_Q8;
static volatile uint8_t toneTiltWetMax = (OT_TILT_WET_MAX > 255) ? 255 : OT_TILT_WET_MAX;
static volatile uint8_t softClipCubicShift = OT_SOFTCLIP_CUBIC_SHIFT;

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
    // Darken very high notes a bit to get closer to classic theremin timbre.
    uint8_t filterShift = 2;
    if (absIncrement > 3072U) {
      filterShift = 5;
    } else if (absIncrement > 1536U) {
      filterShift = 4;
    } else if (absIncrement > 768U) {
      filterShift = 3;
    }
    toneFilterState += (waveSample - toneFilterState) >> filterShift;

    uint8_t wet = 0;
    if (absIncrement > OT_TILT_START_INCREMENT) {
      const uint16_t wetRaw = (uint16_t)((absIncrement - OT_TILT_START_INCREMENT) >> 3);
      const uint16_t wetMax = (uint16_t)toneTiltWetMax;
      wet = (wetRaw > wetMax) ? (uint8_t)wetMax : (uint8_t)wetRaw;
    }
    waveSample += ((toneFilterState - waveSample) * (int32_t)wet) >> 8;
  } else {
    toneFilterState = waveSample;
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
  pointer += vPointerIncrement;
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
  toneFilterState = 0;
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

bool ihGetWaveMorphEnabled() {
  return waveMorphEnabled;
}

bool ihGetToneTiltEnabled() {
  return toneTiltEnabled;
}

bool ihGetSoftClipEnabled() {
  return softClipEnabled;
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
