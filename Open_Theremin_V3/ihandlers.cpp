#include "Arduino.h"

#include "ihandlers.h"
#include "SPImcpDAC.h"
#include "timer.h"
#include "pins.h"

#include "build.h"

#include "theremin_sintable.c"
#include "theremin_sintable2.c"
#include "theremin_sintable3.c"
#include "theremin_sintable4.c"
#include "theremin_sintable5.c"
#include "theremin_sintable6.c"
#include "theremin_sintable7.c"
#include "theremin_sintable8.c"

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

static const uint32_t MCP_DAC_BASE = 2048;

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

volatile uint16_t timer_overflow_counter;

volatile uint8_t vWavetableSelector = 0;

static volatile uint16_t pointer = 0;
static volatile uint8_t debounce_p = 0;
static volatile uint8_t debounce_v = 0;

static volatile bool int1Enabled = false;
static volatile uint16_t pitch_capture_counter_i = 0;

static inline uint16_t readTimerCounter16() {
  return (uint16_t)((micros() * 16UL) & 0xFFFF);
}

void ihDisableInt1() {
  int1Enabled = false;
}

void ihEnableInt1() {
  if (reenableInt1) {
    int1Enabled = true;
  }
}

static void onPitchCapture() {
  pitch_capture_counter_i = readTimerCounter16();
  debounce_p = 0;
}

static void onVolumeCapture() {
  vol_counter_i = readTimerCounter16();
  debounce_v = 0;
}

static void onWaveTick() {
  if (!int1Enabled) {
    return;
  }

  SPImcpDAClatch();

  uint32_t scaledSample = 0;
  uint16_t offset = (uint16_t)(pointer >> 6) & 0x3ff;

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#else
  const int16_t waveSample = wavetables[vWavetableSelector][offset];
  scaledSample = ((int32_t)waveSample * (uint32_t)vScaledVolume) >> 16;
  SPImcpDACsend(scaledSample + MCP_DAC_BASE);
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

void ihInitialiseTimer() {
  // Nothing to configure on UNO R4.
}

void ihInitialiseInterrupts() {
  reenableInt1 = true;
  int1Enabled = true;
  const int waveInterrupt = digitalPinToInterrupt(OT_WAVE_TICK_PIN);
  const int volumeInterrupt = digitalPinToInterrupt(OT_VOLUME_CAPTURE_PIN);
  const int pitchInterrupt = digitalPinToInterrupt(OT_PITCH_CAPTURE_PIN);

  if (waveInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(waveInterrupt, onWaveTick, RISING);
  }
  if (volumeInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(volumeInterrupt, onVolumeCapture, RISING);
  }
  if (pitchInterrupt != NOT_AN_INTERRUPT) {
    attachInterrupt(pitchInterrupt, onPitchCapture, RISING);
  }
}

void ihInitialisePitchMeasurement() {
  reenableInt1 = false;
  int1Enabled = false;
  const int waveInterrupt = digitalPinToInterrupt(OT_WAVE_TICK_PIN);
  const int volumeInterrupt = digitalPinToInterrupt(OT_VOLUME_CAPTURE_PIN);
  const int pitchInterrupt = digitalPinToInterrupt(OT_PITCH_CAPTURE_PIN);
  if (waveInterrupt != NOT_AN_INTERRUPT) {
    detachInterrupt(waveInterrupt);
  }
  if (volumeInterrupt != NOT_AN_INTERRUPT) {
    detachInterrupt(volumeInterrupt);
  }
  if (pitchInterrupt != NOT_AN_INTERRUPT) {
    detachInterrupt(pitchInterrupt);
  }
}

void ihInitialiseVolumeMeasurement() {
  ihInitialisePitchMeasurement();
}
