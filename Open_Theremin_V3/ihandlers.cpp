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

static inline void runWaveTick() {
  if (!int1Enabled) {
    return;
  }

  SPImcpDAClatch();

  const uint16_t offset = (uint16_t)(pointer >> 6) & 0x3ff;

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#else
  const int16_t waveSample = wavetables[vWavetableSelector][offset];
  const uint32_t scaledSample = ((int32_t)waveSample * (uint32_t)vScaledVolume) >> 16;
#if OT_USE_DMA
  interrupts();
#endif
  SPImcpDACsendPrepared(SPImcpDACformatA(scaledSample + MCP_DAC_BASE));
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
