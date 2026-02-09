#include "Arduino.h"

#include "application.h"

#include "hw.h"
#include "SPImcpDAC.h"
#include "ihandlers.h"
#include "timer.h"
#include "wavetables.h"
#include "error_indicator.h"
#include "debug_log.h"
#include <EEPROM.h>

#if OT_MIDI_NATIVE_USB && defined(OT_MIDI_BACKEND_TINYUSB_RAW)
#if !defined(CFG_TUD_MIDI) || (CFG_TUD_MIDI != 1)
#error "OT_MIDI_BACKEND_TINYUSB_RAW needs CFG_TUD_MIDI=1."
#endif
extern "C" {
#include "tusb.h"
#include "class/midi/midi_device.h"
}
#define OT_HAS_NATIVE_USB_MIDI 1
#else
#define OT_HAS_NATIVE_USB_MIDI 0
#endif

#if OT_MIDI_NATIVE_USB && !OT_HAS_NATIVE_USB_MIDI
#error "OT_MIDI_NATIVE_USB=1 but no native USB MIDI backend is active. Define OT_MIDI_BACKEND_TINYUSB_RAW=1 (and provide core/linker support), or disable OT_MIDI_NATIVE_USB."
#endif

const AppMode AppModeValues[] = {MUTE,NORMAL};
const int16_t PitchCalibrationTolerance = 15;
const int16_t VolumeCalibrationTolerance = 21;
const int8_t HYST_VAL = 40;
const long PitchCalTarget = 14925L;
const long VolumeCalTarget = 13698L;

static int32_t pitchCalibrationBase = 0;
static int16_t pitchDAC = 0;
static int16_t volumeDAC = 0;

static int32_t volCalibrationBase   = 0;

static uint8_t new_midi_note =0;
static uint8_t old_midi_note =0;

static uint8_t new_midi_loop_cc_val =0;
static uint8_t old_midi_loop_cc_val =0;

static uint8_t midi_velocity = 0;

static uint8_t loop_hand_pos = 0; 
static uint16_t outputFadeGateQ8 = 0;
static uint32_t outputFadeLastStepMs = 0;

static uint16_t new_midi_rod_cc_val =0;
static uint16_t old_midi_rod_cc_val =0;

static uint16_t new_midi_bend =0;
static uint16_t old_midi_bend = 0;
static uint8_t midi_bend_low; 
static uint8_t midi_bend_high;

static uint32_t long_log_note = 0;
static uint32_t midi_key_follow = 2048; ;
      
// Configuration parameters
static uint8_t registerValue = 2;
  // wavetable selector is defined and initialized in ihandlers.cpp
static uint8_t midi_channel = OT_MIDI_IN_CHANNEL;
static uint8_t midi_in_channel = OT_MIDI_IN_CHANNEL;
static uint8_t midi_bend_range = 2;
static uint8_t midi_volume_trigger = 0;
static uint8_t flag_legato_on = 1;
static uint8_t flag_pitch_bend_on = 1;
static uint8_t loop_midi_cc = 7;
static uint8_t rod_midi_cc = 255; 
static uint8_t rod_midi_cc_lo = 255; 
static uint32_t rod_cc_scale = 128;


// tweakable paramameters
#define VELOCITY_SENS  9 // How easy it is to reach highest velocity (127). Something betwen 5 and 12.
#define PLAYER_ACCURACY  819 // between 0 (very accurate players) and 2048 (not accurate at all)

static uint16_t data_pot_value = 0; 
static uint16_t old_data_pot_value = 0; 

static uint16_t param_pot_value = 0; 
static uint16_t old_param_pot_value = 0; 

static const uint8_t OT_ADC_READ_BITS = 14;
static const uint8_t OT_ADC_LEGACY_BITS = 10;
static const uint8_t OT_ADC_DOWNSHIFT = OT_ADC_READ_BITS - OT_ADC_LEGACY_BITS;
static const uint16_t OT_MODE_TOGGLE_PRESS_MS = 48;
static const uint16_t OT_CALIBRATION_PRESS_MS = 480;
static const uint16_t OT_LED_RESTORE_MS = 2080;
static const uint16_t OT_MIDI_UPDATE_MS = 3;
static const uint16_t OT_MIDI_CAL_ARM_TIMEOUT_MS = 2000;

static const uint8_t OT_CC_MUTE_TOGGLE = 20;
static const uint8_t OT_CC_PANIC = 21;
static const uint8_t OT_CC_LEGATO = 22;
static const uint8_t OT_CC_PITCH_BEND_ENABLE = 23;
static const uint8_t OT_CC_PITCH_BEND_RANGE = 24;
static const uint8_t OT_CC_VOLUME_TRIGGER = 25;
static const uint8_t OT_CC_WAVETABLE = 26;
static const uint8_t OT_CC_AUDIO_RATE_PRESET = 27;
static const uint8_t OT_CC_WAVEMORPH_ENABLE = 28;
static const uint8_t OT_CC_TONETILT_ENABLE = 29;
static const uint8_t OT_CC_SOFTCLIP_ENABLE = 30;
static const uint8_t OT_CC_WAVEMORPH_SPEED = 31;
static const uint8_t OT_CC_TONETILT_AMOUNT = 32;
static const uint8_t OT_CC_SOFTCLIP_DRIVE = 33;
static const uint8_t OT_CC_VIBRATO_JITTER_ENABLE = 34;
static const uint8_t OT_CC_MIDI_CHANNEL_SET = 36;
static const uint8_t OT_CC_CAL_ARM = 102;
static const uint8_t OT_CC_CAL_CONFIRM = 103;

static const uint8_t OT_CAL_ARM_KEY = 42;
static const uint8_t OT_CAL_CONFIRM_KEY = 99;

static bool calibrationArmed = false;
static uint32_t calibrationArmMillis = 0;
static bool midiCalibrationRequested = false;
static const int OT_EEPROM_ADDR_MIDI_CHANNEL = 12;
static uint8_t activeTonePreset = 0xFF;

struct TonePreset {
  uint8_t wavetable;
  uint8_t transpose;
  uint8_t audioRatePreset;
  uint8_t bendRange;
  uint8_t volumeTrigger;
  uint8_t legatoOn;
  uint8_t pitchBendOn;
  uint8_t morphOn;
  uint8_t morphStepQ8;
  uint8_t tiltOn;
  uint8_t tiltWetMax;
  uint8_t softClipOn;
  uint8_t softClipShift;
  uint8_t vibratoOn;
};

static const TonePreset kTonePresets[] = {
  // warm/clean -> bright/modern
  {0, 2, 2, 2, 4,  1, 1, 1,  2, 1, 170, 1, 26, 1},  // Classic Sing
  {3, 2, 2, 2, 6,  1, 1, 1,  2, 1, 190, 1, 26, 1},  // Cello Air
  {4, 2, 2, 4, 8,  1, 1, 1,  3, 1, 165, 1, 25, 1},  // Vocal Lead
  {5, 2, 2, 7, 10, 1, 1, 1,  4, 1, 145, 1, 24, 1},  // Soft Expressive
  {6, 2, 2, 12,12, 1, 1, 1,  4, 1, 125, 1, 24, 1},  // Clarinet Glide
  {7, 2, 2, 12,14, 1, 1, 1,  5, 1, 110, 1, 23, 1},  // Bright Classic
  {8, 2, 2, 24,16, 1, 1, 1,  6, 1, 95,  1, 23, 1},  // Soft-Saw Warm
  {9, 2, 2, 24,18, 1, 1, 1,  7, 1, 80,  1, 22, 1},  // Phoenix Edge
  {10,2, 1, 12,20, 0, 1, 1,  8, 1, 75,  1, 22, 0},  // Modern Mono
  {11,2, 1, 7, 12, 1, 1, 1, 10, 1, 65,  1, 21, 0},  // Razor Solo
  {12,2, 2, 12,22, 1, 1, 1, 10, 1, 60,  1, 22, 0},  // PolyBLEP Saw
  {13,1, 2, 24,24, 0, 1, 1, 12, 1, 55,  1, 21, 0},  // PolyBLEP Pulse
};

static bool setAudioRatePreset(uint8_t preset) {
  uint32_t targetHz;
  switch (preset) {
    case 0: targetHz = 31250U; break;
    case 1: targetHz = 40000U; break;
    case 2: targetHz = 48000U; break;
    default: targetHz = (uint32_t)OT_AUDIO_TICK_HZ; break;
  }
#if defined(OT_AUDIO_TICK_SAFE_MAX_HZ) && (OT_AUDIO_TICK_SAFE_MAX_HZ > 0)
  if (targetHz > (uint32_t)OT_AUDIO_TICK_SAFE_MAX_HZ) {
    OT_DEBUG_PRINT("[DBG] audio-rate clamp ");
    OT_DEBUG_PRINT((unsigned long)targetHz);
    OT_DEBUG_PRINT(" -> ");
    OT_DEBUG_PRINTLN((unsigned long)OT_AUDIO_TICK_SAFE_MAX_HZ);
    targetHz = (uint32_t)OT_AUDIO_TICK_SAFE_MAX_HZ;
  }
#endif
  if (ihGetAudioTickHz() == targetHz) {
    return true;
  }
  return ihSetAudioTickHz(targetHz);
}

static void resetAudioFeatureDefaults() {
  ihSetWaveMorphEnabled(OT_WAVEMORPH_ENABLE_DEFAULT != 0);
  ihSetToneTiltEnabled(OT_TILT_ENABLE_DEFAULT != 0);
  ihSetSoftClipEnabled(OT_SOFTCLIP_ENABLE_DEFAULT != 0);
  ihSetVibratoJitterEnabled(OT_VIBRATO_ENABLE_DEFAULT != 0);
  ihSetWaveMorphStepQ8((OT_WAVEMORPH_STEP_Q8 == 0) ? 1 : OT_WAVEMORPH_STEP_Q8);
  ihSetToneTiltWetMax((OT_TILT_WET_MAX > 255) ? 255 : OT_TILT_WET_MAX);
  ihSetSoftClipCubicShift(OT_SOFTCLIP_CUBIC_SHIFT);
}

static void applyCleanSineTestMode() {
#if OT_TESTMODE_CLEAN_SINE
  // Freeze to clean baseline signal path for diagnostics.
  vWavetableSelector = 0;
  ihSetWaveMorphTargetQ8(0);
  registerValue = 2;
  activeTonePreset = 0xFF;
  (void)setAudioRatePreset(0);
  ihSetWaveMorphEnabled(false);
  ihSetToneTiltEnabled(false);
  ihSetSoftClipEnabled(false);
  ihSetVibratoJitterEnabled(false);
  ihSetWaveMorphStepQ8(1);
  ihSetToneTiltWetMax(0);
  ihSetMasterOutGainQ8(220);
#endif
}

static void loadMidiChannelPersistent() {
  uint8_t stored = 0xFF;
  EEPROM.get(OT_EEPROM_ADDR_MIDI_CHANNEL, stored);
  if (stored <= 15U) {
    midi_channel = stored;
    midi_in_channel = stored;
  } else {
    midi_channel = OT_MIDI_IN_CHANNEL;
    midi_in_channel = OT_MIDI_IN_CHANNEL;
  }
}

static void saveMidiChannelPersistent() {
  EEPROM.put(OT_EEPROM_ADDR_MIDI_CHANNEL, midi_channel);
}

static void applyTonePreset(uint8_t preset) {
  const uint8_t count = (uint8_t)(sizeof(kTonePresets) / sizeof(kTonePresets[0]));
  if (preset >= count) {
    return;
  }
  if (activeTonePreset == preset) {
    return;
  }

  const TonePreset &cfg = kTonePresets[preset];
  vWavetableSelector = min((uint8_t)(OT_WAVETABLE_COUNT - 1U), cfg.wavetable);
  ihSetWaveMorphTargetQ8(((uint16_t)vWavetableSelector) << 8);
  registerValue = cfg.transpose;
  setAudioRatePreset(cfg.audioRatePreset);
  midi_bend_range = cfg.bendRange;
  midi_volume_trigger = cfg.volumeTrigger;
  flag_legato_on = cfg.legatoOn ? 1 : 0;
  flag_pitch_bend_on = cfg.pitchBendOn ? 1 : 0;

  ihSetWaveMorphEnabled(cfg.morphOn != 0);
  ihSetWaveMorphStepQ8((cfg.morphStepQ8 == 0) ? 1 : cfg.morphStepQ8);
  ihSetToneTiltEnabled(cfg.tiltOn != 0);
  ihSetToneTiltWetMax(cfg.tiltWetMax);
  ihSetSoftClipEnabled(cfg.softClipOn != 0);
  ihSetSoftClipCubicShift(cfg.softClipShift);
  ihSetVibratoJitterEnabled(cfg.vibratoOn != 0);
  activeTonePreset = preset;

  resetTimer();
  HW_LED1_TOGGLE;
  HW_LED2_TOGGLE;
}

static inline uint16_t median3U16(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { const uint16_t t = a; a = b; b = t; }
  if (b > c) { const uint16_t t = b; b = c; c = t; }
  if (a > b) { const uint16_t t = a; a = b; b = t; }
  return b;
}

static inline uint16_t readPotLegacy(uint8_t pin) {
  uint16_t raw;
#if OT_POT_MEDIAN_SAMPLES == 3
  const uint16_t r0 = analogRead(pin);
  const uint16_t r1 = analogRead(pin);
  const uint16_t r2 = analogRead(pin);
  raw = median3U16(r0, r1, r2);
#elif OT_POT_MEDIAN_SAMPLES == 1
  raw = analogRead(pin);
#else
  #error "Unsupported OT_POT_MEDIAN_SAMPLES. Use 1 or 3."
#endif
  return (OT_ADC_DOWNSHIFT > 0) ? (raw >> OT_ADC_DOWNSHIFT) : raw;
}

static inline void serviceOutputFadeGate(AppMode mode, AppState state) {
  const bool mutedWhilePlaying = (mode == MUTE) && (state == PLAYING);
  const uint16_t targetGateQ8 = mutedWhilePlaying ? 0U : 256U;
  const uint16_t upStepQ8 = (OT_OUTPUT_FADE_UP_STEP_Q8 == 0) ? 1U : (uint16_t)OT_OUTPUT_FADE_UP_STEP_Q8;
  const uint16_t downStepQ8 = (OT_OUTPUT_FADE_DOWN_STEP_Q8 == 0) ? 1U : (uint16_t)OT_OUTPUT_FADE_DOWN_STEP_Q8;

  const uint32_t nowMs = millis();
  if (OT_OUTPUT_FADE_STEP_MS > 0U &&
      (uint32_t)(nowMs - outputFadeLastStepMs) < (uint32_t)OT_OUTPUT_FADE_STEP_MS) {
    return;
  }
  outputFadeLastStepMs = nowMs;

  if (outputFadeGateQ8 < targetGateQ8) {
    uint16_t next = (uint16_t)(outputFadeGateQ8 + upStepQ8);
    if (next > targetGateQ8) {
      next = targetGateQ8;
    }
    outputFadeGateQ8 = next;
  } else if (outputFadeGateQ8 > targetGateQ8) {
    uint16_t next = (outputFadeGateQ8 > downStepQ8) ? (uint16_t)(outputFadeGateQ8 - downStepQ8) : 0U;
    if (next < targetGateQ8) {
      next = targetGateQ8;
    }
    outputFadeGateQ8 = next;
  }

  ihSetOutputFadeGateQ8(outputFadeGateQ8);
}

static uint8_t midi_in_running_status = 0;
static uint8_t midi_in_data1 = 0;
static uint8_t midi_in_expected = 0;
static bool midi_in_has_data1 = false;
static uint32_t calibrationErrorSetMs = 0;
static const uint16_t OT_CALIB_ERROR_SHOW_MS = 4000;

#if OT_HAS_NATIVE_USB_MIDI
static uint8_t midi_usb_payload[3] = {0, 0, 0};
static uint8_t midi_usb_payload_size = 0;
static uint8_t midi_usb_payload_pos = 0;
#endif

static inline void midiTransportBegin() {
#if OT_HAS_NATIVE_USB_MIDI
  // USB stack is started by core init.
#else
  #if !OT_SERIAL_MIDI_ENABLE
  OT_DEBUG_PRINTLN_F("[DBG] midi serial transport disabled");
  #else
  Serial.begin(OT_MIDI_SERIAL_BAUD);
  #endif
#endif
}

static inline void midiTransportWrite3(uint8_t status, uint8_t data1, uint8_t data2) {
#if OT_HAS_NATIVE_USB_MIDI
  const uint8_t msg[3] = {status, data1, data2};
  if (tud_midi_mounted()) {
    (void)tud_midi_stream_write(0, msg, sizeof(msg));
  }
#else
  #if !OT_SERIAL_MIDI_ENABLE
  (void)status;
  (void)data1;
  (void)data2;
  #else
  // Keep audio/control loop responsive: drop MIDI bytes if USB CDC TX buffer is busy.
  if (!Serial || Serial.availableForWrite() < 3) {
    return;
  }
  Serial.write(status);
  Serial.write(data1);
  Serial.write(data2);
  #endif
#endif
}

static inline bool midiTransportReadByte(uint8_t *out) {
#if OT_HAS_NATIVE_USB_MIDI
  while (true) {
    if (midi_usb_payload_pos < midi_usb_payload_size) {
      *out = midi_usb_payload[midi_usb_payload_pos++];
      return true;
    }

    uint8_t packet[4];
    if (!tud_midi_packet_read(packet)) {
      return false;
    }

    const uint8_t cin = packet[0] & 0x0F;
    if (cin == 0x5 || cin == 0xF) {
      midi_usb_payload_size = 1;
    } else if (cin == 0x2 || cin == 0x6 || cin == 0xC || cin == 0xD) {
      midi_usb_payload_size = 2;
    } else {
      midi_usb_payload_size = 3;
    }

    midi_usb_payload[0] = packet[1];
    midi_usb_payload[1] = packet[2];
    midi_usb_payload[2] = packet[3];
    midi_usb_payload_pos = 0;
  }
#else
  #if !OT_SERIAL_MIDI_ENABLE
  (void)out;
  return false;
  #else
  if (!Serial) {
    return false;
  }
  if (Serial.available() <= 0) {
    return false;
  }
  *out = (uint8_t)Serial.read();
  return true;
  #endif
#endif
}

static inline void midiTransportService() {
#if OT_HAS_NATIVE_USB_MIDI
  tud_task();
#endif
}

static unsigned long countCaptureEdgesForMs(bool pitchChannel, uint16_t gateMs) {
  if (gateMs == 0U) {
    return 0UL;
  }

  const uint32_t startEdges = pitchChannel ? ihGetPitchCaptureCount() : ihGetVolumeCaptureCount();
  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < gateMs) {
    delay(1);
  }
  const uint32_t endEdges = pitchChannel ? ihGetPitchCaptureCount() : ihGetVolumeCaptureCount();
  const uint32_t edges = endEdges - startEdges;
  return (edges * 1000UL) / gateMs;
}

static inline uint32_t median3U32(uint32_t a, uint32_t b, uint32_t c) {
  if (a > b) { const uint32_t t = a; a = b; b = t; }
  if (b > c) { const uint32_t t = b; b = c; c = t; }
  if (a > b) { const uint32_t t = a; a = b; b = t; }
  return b;
}

static bool waitForCaptureSample(volatile bool *flag, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (!(*flag)) {
    if ((uint32_t)(millis() - t0) >= timeoutMs) {
      return false;
    }
  }
  return true;
}


Application::Application()
  : _state(PLAYING),
    _mode(NORMAL) {
};

void Application::setup() {
  OT_DEBUG_BEGIN();
  OT_DEBUG_PRINTLN_F("[DBG] setup: start");

  clearErrorIndicator();
  HW_LED1_ON;HW_LED2_OFF;

  pinMode(Application::BUTTON_PIN, INPUT_PULLUP);
  pinMode(Application::LED_PIN_1,    OUTPUT);
  pinMode(Application::LED_PIN_2,    OUTPUT);
  pinMode(OT_VOLUME_CAPTURE_PIN, INPUT);
  pinMode(OT_PITCH_CAPTURE_PIN, INPUT);
  analogReadResolution(OT_ADC_READ_BITS);
  OT_DEBUG_PRINTLN_F("[DBG] setup: pins+adc ready");

  digitalWrite(Application::LED_PIN_1, HIGH);    // turn the LED off by making the voltage LOW

   SPImcpDACinit();
  OT_DEBUG_PRINTLN_F("[DBG] setup: spi dac ready");
  if (!setAudioRatePreset(OT_AUDIO_RATE_PRESET)) {
    OT_DEBUG_PRINTLN_F("[DBG] setup: audio-rate preset failed");
    fatalErrorLoop(OT_ERR_AUDIO_RATE);
  }
  OT_DEBUG_PRINTLN_F("[DBG] setup: audio-rate preset ok");

EEPROM.get(0,pitchDAC);
EEPROM.get(2,volumeDAC);

SPImcpDAC2Asend(pitchDAC);
SPImcpDAC2Bsend(volumeDAC);

  
initialiseTimer();
initialiseInterrupts();
  outputFadeGateQ8 = 0U;
  outputFadeLastStepMs = millis();
  ihSetOutputFadeGateQ8(outputFadeGateQ8);
  OT_DEBUG_PRINTLN_F("[DBG] setup: interrupts ready");


EEPROM.get(4,pitchCalibrationBase);
EEPROM.get(8,volCalibrationBase);

  // One-shot sanity check: adapt stale EEPROM baseline (e.g. from previous timing domain)
  // to current live capture scale on boot.
  uint32_t livePitch = 0;
  uint32_t liveVol = 0;
  resetPitchFlag();
  if (waitForCaptureSample(&pitchValueAvailable, 250U)) {
    livePitch = pitch;
    resetPitchFlag();
  }
  resetVolFlag();
  if (waitForCaptureSample(&volumeValueAvailable, 250U)) {
    liveVol = vol;
    resetVolFlag();
  }

  if (livePitch > 0U) {
    const uint32_t low = livePitch / 2U;
    const uint32_t high = livePitch * 2U;
    if ((pitchCalibrationBase <= 0) ||
        ((uint32_t)pitchCalibrationBase < low) ||
        ((uint32_t)pitchCalibrationBase > high)) {
      OT_DEBUG_PRINT("[DBG] baseline pitch adapt ");
      OT_DEBUG_PRINT((long)pitchCalibrationBase);
      OT_DEBUG_PRINT(" -> ");
      OT_DEBUG_PRINTLN((unsigned long)livePitch);
      pitchCalibrationBase = (int32_t)livePitch;
      EEPROM.put(4, pitchCalibrationBase);
    }
  }

  if (liveVol > 0U) {
    const uint32_t low = liveVol / 2U;
    const uint32_t high = liveVol * 2U;
    if ((volCalibrationBase <= 0) ||
        ((uint32_t)volCalibrationBase < low) ||
        ((uint32_t)volCalibrationBase > high)) {
      OT_DEBUG_PRINT("[DBG] baseline vol adapt ");
      OT_DEBUG_PRINT((long)volCalibrationBase);
      OT_DEBUG_PRINT(" -> ");
      OT_DEBUG_PRINTLN((unsigned long)liveVol);
      volCalibrationBase = (int32_t)liveVol;
      EEPROM.put(8, volCalibrationBase);
    }
  }

  if (pitchCalibrationBase <= 0) {
    pitchCalibrationBase = 1;
  }
  if (volCalibrationBase <= 0) {
    volCalibrationBase = 5000;
  }
 
 init_parameters();
 loadMidiChannelPersistent();
 OT_DEBUG_PRINT("[DBG] setup: midi channel=");
 OT_DEBUG_PRINTLN((int)midi_channel);
 resetAudioFeatureDefaults();
 applyCleanSineTestMode();
#if OT_TESTMODE_CLEAN_SINE
 OT_DEBUG_PRINTLN_F("[DBG] clean-sine testmode enabled");
#endif
 midi_setup();
 OT_DEBUG_PRINTLN_F("[DBG] setup: done");
  
}

void Application::initialiseTimer() {
  ihInitialiseTimer();
}

void Application::initialiseInterrupts() {
  ihInitialiseInterrupts();
}

void Application::InitialisePitchMeasurement() {
   ihInitialisePitchMeasurement();
}

void Application::InitialiseVolumeMeasurement() {
   ihInitialiseVolumeMeasurement();
}

unsigned long Application::GetPitchMeasurement()
{
  return countCaptureEdgesForMs(true, 1000);

}

unsigned long Application::GetVolumeMeasurement()
{
  return countCaptureEdgesForMs(false, 1000);
}



#if CV_ENABLED                                 // Initialise PWM Generator for CV output
void initialiseCVOut() {

}
#endif

AppMode Application::nextMode() {
  return _mode == NORMAL ? MUTE : AppModeValues[_mode + 1];
}

void Application::loop() {
  int32_t pitch_v = 0, pitch_l = 0;            // Last value of pitch  (for filtering)
  int32_t vol_v = 0,   vol_l = 0;              // Last value of volume (for filtering)
  uint32_t volMedian0 = 0, volMedian1 = 0, volMedian2 = 0;
  bool volMedianPrimed = false;
  uint16_t volLimitedScaled = 0;
  bool volLimiterPrimed = false;
  static bool calButtonLatch = false;
  static uint32_t calArmMs = 0;
#if OT_DEBUG_ENABLED && (OT_DEBUG_SENSOR_LOGS || OT_DEBUG_RUNTIME_LOGS)
  static uint32_t dbgLastSensorLogMs = 0;
  static uint32_t dbgPitchRaw = 0;
  static int32_t dbgPitchFiltered = 0;
  static uint32_t dbgVolRaw = 0;
  static int32_t dbgVolFiltered = 0;
#endif
#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
  static uint32_t dbgLastAliveMs = 0;
  static uint32_t dbgLastAntennaMs = 0;
  static bool dbgButtonInit = false;
  static bool dbgButtonPressedPrev = false;
  static uint32_t dbgButtonPressStartMs = 0;
  static uint16_t dbgLastPitchPot = 0xFFFFU;
  static uint16_t dbgLastVolumePot = 0xFFFFU;
  static uint16_t dbgLastParamPot = 0xFFFFU;
  static uint16_t dbgLastDataPot = 0xFFFFU;
#endif

  uint16_t volumePotValue = 0;
  uint16_t pitchPotValue = 0;

  mloop:                   // Main loop avoiding the GCC "optimization"

#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
  {
    const uint32_t now = millis();
    if ((uint32_t)(now - dbgLastAliveMs) >= 1000U) {
      dbgLastAliveMs = now;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      OT_DEBUG_PRINT("[DBG] alive ms=");
      OT_DEBUG_PRINT(now);
      OT_DEBUG_PRINT(" err=");
      OT_DEBUG_PRINT((int)getErrorIndicator());
      OT_DEBUG_PRINT(" mode=");
      OT_DEBUG_PRINT((int)_mode);
      OT_DEBUG_PRINT(" state=");
      OT_DEBUG_PRINT((int)_state);
      OT_DEBUG_PRINT(" tick=");
      OT_DEBUG_PRINT((unsigned long)ihGetWaveTickCount());
      OT_DEBUG_PRINT(" capP=");
      OT_DEBUG_PRINT((unsigned long)ihGetPitchCaptureCount());
      OT_DEBUG_PRINT(" capV=");
      OT_DEBUG_PRINTLN((unsigned long)ihGetVolumeCaptureCount());
    }
  }
#endif

  pitchPotValue    = readPotLegacy(PITCH_POT);
  volumePotValue   = readPotLegacy(VOLUME_POT);
  
  #if OT_TESTMODE_CLEAN_SINE
  applyCleanSineTestMode();
  #else
  set_parameters();
  #endif
  serviceErrorIndicator();
  ihRecoverTimer();
  midi_input_poll();
  midi_flush();

#if OT_DEBUG_ENABLED && OT_DEBUG_SENSOR_LOGS
  {
    const uint32_t now = millis();
    if ((uint32_t)(now - dbgLastSensorLogMs) >= (uint32_t)OT_DEBUG_SENSOR_LOG_MS) {
      dbgLastSensorLogMs = now;
      OT_DEBUG_PRINT("[DBG] sensors pitchRaw=");
      OT_DEBUG_PRINT((unsigned int)dbgPitchRaw);
      OT_DEBUG_PRINT(" pitchFilt=");
      OT_DEBUG_PRINT((long)dbgPitchFiltered);
      OT_DEBUG_PRINT(" volRaw=");
      OT_DEBUG_PRINT((unsigned int)dbgVolRaw);
      OT_DEBUG_PRINT(" volFilt=");
      OT_DEBUG_PRINT((long)dbgVolFiltered);
      OT_DEBUG_PRINT(" scaledVol=");
      OT_DEBUG_PRINT((unsigned int)vScaledVolume);
      OT_DEBUG_PRINT(" inc=");
      OT_DEBUG_PRINT((unsigned int)vPointerIncrement);
      OT_DEBUG_PRINT(" mode=");
      OT_DEBUG_PRINT((int)_mode);
      OT_DEBUG_PRINT(" state=");
      OT_DEBUG_PRINT((int)_state);
      OT_DEBUG_PRINT(" tick=");
      OT_DEBUG_PRINT((unsigned long)ihGetWaveTickCount());
      OT_DEBUG_PRINT(" capP=");
      OT_DEBUG_PRINT((unsigned long)ihGetPitchCaptureCount());
      OT_DEBUG_PRINT(" capV=");
      OT_DEBUG_PRINTLN((unsigned long)ihGetVolumeCaptureCount());
    }
  }
#endif

#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
  {
    const uint32_t now = millis();
    const bool buttonPressed = HW_BUTTON_PRESSED;
    if (!dbgButtonInit) {
      dbgButtonInit = true;
      dbgButtonPressedPrev = buttonPressed;
      dbgButtonPressStartMs = now;
    }
    if (buttonPressed != dbgButtonPressedPrev) {
      if (buttonPressed) {
        dbgButtonPressStartMs = now;
        OT_DEBUG_PRINTLN_F("[DBG] button pressed");
      } else {
        OT_DEBUG_PRINT("[DBG] button released after ms=");
        OT_DEBUG_PRINTLN((unsigned long)(now - dbgButtonPressStartMs));
      }
      dbgButtonPressedPrev = buttonPressed;
    }

    bool potsChanged = false;
    const uint16_t potDeltaThreshold = 4U;
    if (dbgLastPitchPot == 0xFFFFU || (uint16_t)abs((int)pitchPotValue - (int)dbgLastPitchPot) >= potDeltaThreshold) {
      dbgLastPitchPot = pitchPotValue;
      potsChanged = true;
    }
    if (dbgLastVolumePot == 0xFFFFU || (uint16_t)abs((int)volumePotValue - (int)dbgLastVolumePot) >= potDeltaThreshold) {
      dbgLastVolumePot = volumePotValue;
      potsChanged = true;
    }
    if (dbgLastParamPot == 0xFFFFU || (uint16_t)abs((int)param_pot_value - (int)dbgLastParamPot) >= potDeltaThreshold) {
      dbgLastParamPot = param_pot_value;
      potsChanged = true;
    }
    if (dbgLastDataPot == 0xFFFFU || (uint16_t)abs((int)data_pot_value - (int)dbgLastDataPot) >= potDeltaThreshold) {
      dbgLastDataPot = data_pot_value;
      potsChanged = true;
    }
    if (potsChanged) {
      OT_DEBUG_PRINT("[DBG] pots pitch=");
      OT_DEBUG_PRINT((unsigned int)pitchPotValue);
      OT_DEBUG_PRINT(" volume=");
      OT_DEBUG_PRINT((unsigned int)volumePotValue);
      OT_DEBUG_PRINT(" param=");
      OT_DEBUG_PRINT((unsigned int)param_pot_value);
      OT_DEBUG_PRINT(" data=");
      OT_DEBUG_PRINTLN((unsigned int)data_pot_value);
    }

    if ((uint32_t)(now - dbgLastAntennaMs) >= 250U) {
      dbgLastAntennaMs = now;
      OT_DEBUG_PRINT("[DBG] ant pitchRaw=");
      OT_DEBUG_PRINT((unsigned int)dbgPitchRaw);
      OT_DEBUG_PRINT(" pitchFilt=");
      OT_DEBUG_PRINT((long)dbgPitchFiltered);
      OT_DEBUG_PRINT(" volRaw=");
      OT_DEBUG_PRINT((unsigned int)dbgVolRaw);
      OT_DEBUG_PRINT(" volFilt=");
      OT_DEBUG_PRINT((long)dbgVolFiltered);
      OT_DEBUG_PRINT(" scaledVol=");
      OT_DEBUG_PRINTLN((unsigned int)vScaledVolume);
    }
  }
#endif
  
  const bool buttonPressedNow = HW_BUTTON_PRESSED;
  if (!buttonPressedNow) {
    calButtonLatch = false;
  }

  if (_state == PLAYING && buttonPressedNow && !calButtonLatch) 
  {
    _state = CALIBRATING;
    calButtonLatch = true;
    calArmMs = millis();
    if (getErrorIndicator() == OT_ERR_CALIB) {
      clearErrorIndicator();
      calibrationErrorSetMs = 0;
      OT_DEBUG_PRINTLN_F("[DBG] calibration: cleared previous CALIB error");
    }
    OT_DEBUG_PRINTLN_F("[DBG] calibration: arm");
    resetTimer();
  }

  const uint32_t calHoldMs = (uint32_t)(millis() - calArmMs);

  if (_state == CALIBRATING && HW_BUTTON_RELEASED && !midiCalibrationRequested) 
  {
    if (calHoldMs >= OT_MODE_TOGGLE_PRESS_MS && calHoldMs < OT_CALIBRATION_PRESS_MS) 
    {
         _mode = nextMode();
         if (_mode==NORMAL) 
         {
          HW_LED1_ON;HW_LED2_OFF;
          _midistate = MIDI_SILENT;
         } 
         else 
         {
          HW_LED1_OFF;HW_LED2_ON;
          _midistate = MIDI_STOP;

         };
         // playModeSettingSound();
    }
    _state = PLAYING;
  };

  if (_state == CALIBRATING &&
      (midiCalibrationRequested || (buttonPressedNow && calHoldMs >= OT_CALIBRATION_PRESS_MS))) 
  {
    const int16_t oldPitchDacBeforeCalibration = pitchDAC;
    const int16_t oldVolumeDacBeforeCalibration = volumeDAC;

    OT_DEBUG_PRINTLN_F("[DBG] calibration: start");
    HW_LED1_OFF; HW_LED2_ON;
  
      
    OT_DEBUG_PRINTLN_F("[DBG] calibration: startup sound");
    playStartupSound();

    // Pause audio ISR while DAC trim calibration runs to avoid concurrent SPI access
    // from waveform output and calibration writes.
    OT_DEBUG_PRINTLN_F("[DBG] calibration: pause audio");
    ihDisableInt1();
    vScaledVolume = 0U;
    delay(2);
		
    // calibrate heterodyne parameters
    OT_DEBUG_PRINTLN_F("[DBG] calibration: calibrate pitch");
    const bool pitchCalOk = calibrate_pitch();
    OT_DEBUG_PRINTLN_F("[DBG] calibration: calibrate volume");
    const bool volumeCalOk = calibrate_volume();

    OT_DEBUG_PRINTLN_F("[DBG] calibration: baseline capture");
    const bool baseCalOk = calibrate();

    const bool calibrationOk = pitchCalOk && volumeCalOk && baseCalOk;
    OT_DEBUG_PRINT("[DBG] calibration overall=");
    OT_DEBUG_PRINTLN(calibrationOk ? "OK" : "FAIL");
  
    _mode=NORMAL;
    if (calibrationOk) {
      OT_DEBUG_PRINTLN_F("[DBG] calibration: eeprom write start");
      EEPROM.put(0, pitchDAC);
      EEPROM.put(2, volumeDAC);
      OT_DEBUG_PRINTLN_F("[DBG] calibration: eeprom write done");
      if (getErrorIndicator() == OT_ERR_CALIB) {
        clearErrorIndicator();
      }
      HW_LED1_ON;HW_LED2_OFF;
    } else {
      // Keep previous playable tuning when baseline capture failed.
      pitchDAC = oldPitchDacBeforeCalibration;
      volumeDAC = oldVolumeDacBeforeCalibration;
      SPImcpDAC2Asend(pitchDAC);
      SPImcpDAC2Bsend(volumeDAC);
      OT_DEBUG_PRINTLN_F("[DBG] calibration: eeprom restore start");
      EEPROM.put(0, pitchDAC);
      EEPROM.put(2, volumeDAC);
      OT_DEBUG_PRINTLN_F("[DBG] calibration: eeprom restore done");

      setErrorIndicator(OT_ERR_CALIB);
      calibrationErrorSetMs = millis();
      HW_LED1_OFF;HW_LED2_ON;
    }

    _state = PLAYING;
    midiCalibrationRequested = false;
    _midistate = MIDI_SILENT;

    // Never block forever if button signal is stuck low.
    if (HW_BUTTON_PRESSED) {
      const uint32_t releaseWaitStart = millis();
      while (HW_BUTTON_PRESSED && (uint32_t)(millis() - releaseWaitStart) < 1500U) {
        delay(1);
      }
      if (HW_BUTTON_PRESSED) {
        OT_DEBUG_PRINTLN_F("[DBG] calibration: button still pressed (timeout)");
      } else {
        OT_DEBUG_PRINTLN_F("[DBG] calibration: button released");
      }
    }

    OT_DEBUG_PRINTLN_F("[DBG] calibration: done");
    OT_DEBUG_PRINTLN_F("[DBG] calibration: reboot to apply");
    delay(30);
    NVIC_SystemReset();
  };

  if (calibrationErrorSetMs != 0 &&
      getErrorIndicator() == OT_ERR_CALIB &&
      (uint32_t)(millis() - calibrationErrorSetMs) >= OT_CALIB_ERROR_SHOW_MS) {
    clearErrorIndicator();
    calibrationErrorSetMs = 0;
    OT_DEBUG_PRINTLN_F("[DBG] calibration: auto-clear CALIB error");
  }

  serviceOutputFadeGate(_mode, _state);

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#endif



  if (pitchValueAvailable) {                        // If capture event

#if OT_DEBUG_ENABLED && (OT_DEBUG_RUNTIME_LOGS || OT_DEBUG_SENSOR_LOGS)
    dbgPitchRaw = pitch;
#endif
    pitch_v=pitch;                         // Averaging pitch values
    pitch_v=pitch_l+((pitch_v-pitch_l)>>2);
    pitch_l=pitch_v;
#if OT_DEBUG_ENABLED && (OT_DEBUG_RUNTIME_LOGS || OT_DEBUG_SENSOR_LOGS)
    dbgPitchFiltered = pitch_v;
#endif


//HW_LED2_ON;


    // set wave frequency for each mode
    switch (_mode) {
      case MUTE : /* NOTHING! */;                                        break;
      case NORMAL: {
        const uint8_t transposeShift = (registerValue < 1U) ? 1U : ((registerValue > 6U) ? 6U : registerValue);
        setWavetableSampleAdvance((((pitch_v - pitchCalibrationBase) >> 2) + 2048 - ((int32_t)pitchPotValue << 1)) >> transposeShift);
        break;
      }
    };
    
  //  HW_LED2_OFF;

    pitchValueAvailable = false;
  }

  if (volumeValueAvailable) {
    const uint32_t volMeasured = vol;
#if OT_VOLUME_SANITIZE_ENABLE
    uint32_t volSanitized = volMeasured;
    if (volSanitized < 64U) {
      volSanitized = 64U;
    }
    // Reject implausible spikes that would cause hard volume chopping.
    if (volCalibrationBase > 0) {
      const uint32_t maxReasonable = (uint32_t)volCalibrationBase * 4U;
      if (volSanitized > maxReasonable) {
        volSanitized = maxReasonable;
      }
    }
#else
    const uint32_t volSanitized = volMeasured;
#endif
#if OT_DEBUG_ENABLED && (OT_DEBUG_RUNTIME_LOGS || OT_DEBUG_SENSOR_LOGS)
    dbgVolRaw = volMeasured;
#endif

    uint32_t volPrepared = volSanitized;
#if OT_VOLUME_MEDIAN3_ENABLE
    if (!volMedianPrimed) {
      volMedian0 = volPrepared;
      volMedian1 = volPrepared;
      volMedian2 = volPrepared;
      volMedianPrimed = true;
    } else {
      volMedian0 = volMedian1;
      volMedian1 = volMedian2;
      volMedian2 = volPrepared;
    }
    volPrepared = median3U32(volMedian0, volMedian1, volMedian2);
#endif

    vol_v = (int32_t)volPrepared;                  // Averaging volume values
    const uint8_t volFilterShift = (OT_VOLUME_FILTER_SHIFT > 6U) ? 6U : (uint8_t)OT_VOLUME_FILTER_SHIFT;
    if (volFilterShift > 0U) {
      vol_v = vol_l + ((vol_v - vol_l) >> volFilterShift);
    }
    vol_l=vol_v;
#if OT_DEBUG_ENABLED && (OT_DEBUG_RUNTIME_LOGS || OT_DEBUG_SENSOR_LOGS)
    dbgVolFiltered = vol_v;
#endif

    const uint8_t volMapShift = (OT_VOLUME_MAP_SHIFT > 8U) ? 8U : (uint8_t)OT_VOLUME_MAP_SHIFT;
    vol_v = MAX_VOLUME - ((((vol_v - volCalibrationBase) * (int32_t)OT_VOLUME_RESPONSE_GAIN)) >> volMapShift)
            + ((int32_t)volumePotValue << 2) - 1024;

    // Limit and set volume value
    vol_v = min(vol_v, 4095);
    //    vol_v = vol_v - (1 + MAX_VOLUME - (volumePotValue << 2));
    vol_v = vol_v ;
    vol_v = max(vol_v, 0);
    uint8_t nextLoopHandPos = (uint8_t)(vol_v >> 4);
    const uint8_t volDeadband = (OT_VOLUME_CONTROL_DEADBAND > 127U) ? 127U : (uint8_t)OT_VOLUME_CONTROL_DEADBAND;
    if (volDeadband > 0U) {
      const int32_t d = (int32_t)nextLoopHandPos - (int32_t)loop_hand_pos;
      if (d <= (int32_t)volDeadband && d >= -(int32_t)volDeadband) {
        nextLoopHandPos = loop_hand_pos;
      }
    }
    loop_hand_pos = nextLoopHandPos;

    const uint16_t targetScaled = (uint16_t)(loop_hand_pos * (loop_hand_pos + 2));
    const uint16_t deltaLimit = (uint16_t)OT_VOLUME_DELTA_LIMIT_PER_UPDATE;
    if (!volLimiterPrimed) {
      volLimitedScaled = targetScaled;
      volLimiterPrimed = true;
    } else if (deltaLimit > 0U) {
      int32_t delta = (int32_t)targetScaled - (int32_t)volLimitedScaled;
      if (delta > (int32_t)deltaLimit) {
        delta = (int32_t)deltaLimit;
      } else if (delta < -(int32_t)deltaLimit) {
        delta = -(int32_t)deltaLimit;
      }
      volLimitedScaled = (uint16_t)((int32_t)volLimitedScaled + delta);
    } else {
      volLimitedScaled = targetScaled;
    }

    vScaledVolume = volLimitedScaled;
    
    volumeValueAvailable = false;
  }

  if (midi_timer >= millisToTicks(OT_MIDI_UPDATE_MS))
  {
    midi_application ();
    midi_timer = 0; 
  }

  goto mloop;                           // End of main loop
}

bool Application::calibrate()
{
  const int32_t oldPitchBase = pitchCalibrationBase;
  const int32_t oldVolBase = volCalibrationBase;

  uint32_t p0 = 0, p1 = 0, p2 = 0;
  uint32_t v0 = 0, v1 = 0, v2 = 0;
  bool pitchSampleOk = true;
  bool volSampleOk = true;

  resetPitchFlag();
  for (uint8_t i = 0; i < 3; ++i) {
    const uint32_t t0 = millis();
    while (!pitchValueAvailable) {
      if ((uint32_t)(millis() - t0) >= 250U) {
        pitchSampleOk = false;
        break;
      }
    }
    if (!pitchSampleOk) {
      break;
    }
    const uint32_t sample = pitch;
    if (i == 0) p0 = sample;
    if (i == 1) p1 = sample;
    if (i == 2) p2 = sample;
    resetPitchFlag();
  }

  resetVolFlag();
  for (uint8_t i = 0; i < 3; ++i) {
    const uint32_t t0 = millis();
    while (!volumeValueAvailable) {
      if ((uint32_t)(millis() - t0) >= 250U) {
        volSampleOk = false;
        break;
      }
    }
    if (!volSampleOk) {
      break;
    }
    const uint32_t sample = vol;
    if (i == 0) v0 = sample;
    if (i == 1) v1 = sample;
    if (i == 2) v2 = sample;
    resetVolFlag();
  }

  const uint32_t pitchPeriod = median3U32(p0, p1, p2);
  const uint32_t volPeriod = median3U32(v0, v1, v2);
  pitchSampleOk = pitchSampleOk && (pitchPeriod > 0U);
  volSampleOk = volSampleOk && (volPeriod > 0U);

  const bool baseOk = pitchSampleOk && volSampleOk;
  if (baseOk) {
    int32_t pitchBaseCandidate = (int32_t)pitchPeriod;
    int32_t volBaseCandidate = (int32_t)volPeriod;
    if (pitchBaseCandidate <= 0) {
      pitchBaseCandidate = 1;
      OT_DEBUG_PRINTLN_F("[DBG] calibrate: pitch base clamped to 1");
    }

    pitchCalibrationBase = pitchBaseCandidate;
    volCalibrationBase = volBaseCandidate;
    EEPROM.put(4,pitchCalibrationBase);
    EEPROM.put(8,volCalibrationBase);
  } else {
    // Keep previous valid baseline values on timeout/failure.
    pitchCalibrationBase = oldPitchBase;
    volCalibrationBase = oldVolBase;
  }

  OT_DEBUG_PRINT("[DBG] calibrate result basePitch=");
  OT_DEBUG_PRINT((long)pitchCalibrationBase);
  OT_DEBUG_PRINT(" baseVol=");
  OT_DEBUG_PRINT((long)volCalibrationBase);
  OT_DEBUG_PRINT(" pSamples=");
  OT_DEBUG_PRINT((unsigned long)p0);
  OT_DEBUG_PRINT("/");
  OT_DEBUG_PRINT((unsigned long)p1);
  OT_DEBUG_PRINT("/");
  OT_DEBUG_PRINT((unsigned long)p2);
  OT_DEBUG_PRINT(" vSamples=");
  OT_DEBUG_PRINT((unsigned long)v0);
  OT_DEBUG_PRINT("/");
  OT_DEBUG_PRINT((unsigned long)v1);
  OT_DEBUG_PRINT("/");
  OT_DEBUG_PRINT((unsigned long)v2);
  OT_DEBUG_PRINTLN_F("");
  OT_DEBUG_PRINT("[DBG] calibrate base source pitch=");
  OT_DEBUG_PRINTLN(pitchSampleOk ? "OK" : "FAIL");
  OT_DEBUG_PRINT("[DBG] calibrate base source volume=");
  OT_DEBUG_PRINTLN(volSampleOk ? "OK" : "FAIL");

  OT_DEBUG_PRINT("[DBG] calibrate base status=");
  OT_DEBUG_PRINTLN(baseOk ? "OK" : "FAIL");
  return baseOk;
}

bool Application::calibrate_pitch()
{
  static long target = 0;
  static long raw0 = 0;
  static long raw1 = 0;
  static long f0 = 0;
  static long f1 = 0;
  static uint16_t iterations = 0;
  const int16_t oldPitchDac = pitchDAC;

  SPImcpDACinit();

  target = PitchCalTarget;

  SPImcpDAC2Bsend(1600);

  int16_t lo = 0;
  int16_t hi = 4095;

  SPImcpDAC2Asend(lo);
  delay(100);
  raw0 = (long)GetPitchMeasurement();
  f0 = raw0 - target;

  SPImcpDAC2Asend(hi);
  delay(100);
  raw1 = (long)GetPitchMeasurement();
  f1 = raw1 - target;

  int16_t bestDac = (labs(f0) <= labs(f1)) ? lo : hi;
  long bestAbs = (labs(f0) <= labs(f1)) ? labs(f0) : labs(f1);

  // If no bracket, fall back to best endpoint; otherwise do robust bisection.
  const bool hasBracket = ((f0 <= 0 && f1 >= 0) || (f0 >= 0 && f1 <= 0));
  iterations = 0;
  if (hasBracket) {
    while (iterations < 12) {
      const int16_t mid = (int16_t)(((int32_t)lo + (int32_t)hi) / 2L);
      SPImcpDAC2Asend(mid);
      delay(100);
      const long fm = (long)GetPitchMeasurement() - target;

      if (labs(fm) < bestAbs) {
        bestAbs = labs(fm);
        bestDac = mid;
      }
      if (labs(fm) <= PitchCalibrationTolerance) {
        break;
      }

      if ((f0 <= 0 && fm >= 0) || (f0 >= 0 && fm <= 0)) {
        hi = mid;
        f1 = fm;
      } else {
        lo = mid;
        f0 = fm;
      }
      HW_LED2_TOGGLE;
      iterations++;
    }
  }

  const int16_t pitchDacCandidate = (int16_t)constrain((int32_t)bestDac, 0L, 4095L);
  const bool signalOk = (raw0 > 0) && (raw1 > 0);
  const bool converged = (bestAbs <= PitchCalibrationTolerance);
  const bool pitchCalOk = signalOk && converged;

  if (pitchCalOk) {
    pitchDAC = pitchDacCandidate;
  } else {
    pitchDAC = oldPitchDac;
  }

  OT_DEBUG_PRINT("[DBG] calibrate_pitch result eepromPitchDAC=");
  OT_DEBUG_PRINT((int)pitchDacCandidate);
  OT_DEBUG_PRINT(" target=");
  OT_DEBUG_PRINT(target);
  OT_DEBUG_PRINT(" raw0=");
  OT_DEBUG_PRINT(raw0);
  OT_DEBUG_PRINT(" raw1=");
  OT_DEBUG_PRINT(raw1);
  OT_DEBUG_PRINT(" f0=");
  OT_DEBUG_PRINT(f0);
  OT_DEBUG_PRINT(" f1=");
  OT_DEBUG_PRINT(f1);
  OT_DEBUG_PRINT(" bestAbs=");
  OT_DEBUG_PRINT(bestAbs);
  OT_DEBUG_PRINT(" iterations=");
  OT_DEBUG_PRINTLN((unsigned int)iterations);
  OT_DEBUG_PRINT("[DBG] calibrate_pitch status=");
  OT_DEBUG_PRINTLN(pitchCalOk ? "OK" : "FAIL");
  return pitchCalOk;
}

bool Application::calibrate_volume()
{
  static long target = 0;
  static long raw0 = 0;
  static long raw1 = 0;
  static long f0 = 0;
  static long f1 = 0;
  static uint16_t iterations = 0;
  const int16_t oldVolumeDac = volumeDAC;

  SPImcpDACinit();

  target = VolumeCalTarget;

  int16_t lo = 0;
  int16_t hi = 4095;

  SPImcpDAC2Bsend(lo);
  delay_NOP(44316);
  raw0 = (long)GetVolumeMeasurement();
  f0 = raw0 - target;

  SPImcpDAC2Bsend(hi);
  delay_NOP(44316);
  raw1 = (long)GetVolumeMeasurement();
  f1 = raw1 - target;

  int16_t bestDac = (labs(f0) <= labs(f1)) ? lo : hi;
  long bestAbs = (labs(f0) <= labs(f1)) ? labs(f0) : labs(f1);

  const bool hasBracket = ((f0 <= 0 && f1 >= 0) || (f0 >= 0 && f1 <= 0));
  iterations = 0;
  if (hasBracket) {
    while (iterations < 12) {
      const int16_t mid = (int16_t)(((int32_t)lo + (int32_t)hi) / 2L);
      SPImcpDAC2Bsend(mid);
      delay_NOP(44316);
      const long fm = (long)GetVolumeMeasurement() - target;

      if (labs(fm) < bestAbs) {
        bestAbs = labs(fm);
        bestDac = mid;
      }
      if (labs(fm) <= VolumeCalibrationTolerance) {
        break;
      }

      if ((f0 <= 0 && fm >= 0) || (f0 >= 0 && fm <= 0)) {
        hi = mid;
        f1 = fm;
      } else {
        lo = mid;
        f0 = fm;
      }
      HW_LED2_TOGGLE;
      iterations++;
    }
  }

  const int16_t volumeDacCandidate = (int16_t)constrain((int32_t)bestDac, 0L, 4095L);
  const bool signalOk = (raw0 > 0) && (raw1 > 0);
  const bool converged = (bestAbs <= VolumeCalibrationTolerance);
  const bool volumeCalOk = signalOk && converged;

  if (volumeCalOk) {
    volumeDAC = volumeDacCandidate;
  } else {
    volumeDAC = oldVolumeDac;
  }

  OT_DEBUG_PRINT("[DBG] calibrate_volume result eepromVolDAC=");
  OT_DEBUG_PRINT((int)volumeDacCandidate);
  OT_DEBUG_PRINT(" target=");
  OT_DEBUG_PRINT(target);
  OT_DEBUG_PRINT(" raw0=");
  OT_DEBUG_PRINT(raw0);
  OT_DEBUG_PRINT(" raw1=");
  OT_DEBUG_PRINT(raw1);
  OT_DEBUG_PRINT(" f0=");
  OT_DEBUG_PRINT(f0);
  OT_DEBUG_PRINT(" f1=");
  OT_DEBUG_PRINT(f1);
  OT_DEBUG_PRINT(" bestAbs=");
  OT_DEBUG_PRINT(bestAbs);
  OT_DEBUG_PRINT(" iterations=");
  OT_DEBUG_PRINTLN((unsigned int)iterations);
  OT_DEBUG_PRINT("[DBG] calibrate_volume status=");
  OT_DEBUG_PRINTLN(volumeCalOk ? "OK" : "FAIL");
  return volumeCalOk;
}

// calculate log2 of an unsigned from 1 to 65535 into a 4.12 fixed point unsigned
// To avoid use of log (double) function
uint16_t Application::log2U16 (uint16_t lin_input)
{
  uint32_t long_lin; // To turn input into a 16.16 fixed point
  //unsigned long bit_pos;
  uint32_t log_output; // 4.12 fixed point log calculation

  int32_t long_x1;
  int32_t long_x2;
  int32_t long_x3;

  const int32_t POLY_A0 = 37;  
  const int32_t POLY_A1 = 46390; 
  const int32_t POLY_A2 = -18778; 
  const int32_t POLY_A3 = 5155; 


  if (lin_input != 0)
  {    
    long_lin = (uint32_t) (lin_input) << 16; 
    log_output = 0; 

    // Calculate integer part of log2 and reduce long_lin into 16.16 between 1 and 2
    if (long_lin >= 16777216) // 2^(8 + 16)
    {
      log_output += 8 << 12;
      long_lin = long_lin  >> 8;        
    }

    if (long_lin >= 1048576) // 2^(4 + 16)
    {
      log_output += 4 << 12;
      long_lin = long_lin  >> 4;        
    }

    if (long_lin >= 262144) // 2^(2 + 16)
    {
      log_output += 2 << 12;
      long_lin = long_lin  >> 2;        
    }

    if (long_lin >= 131072) // 2^(1 + 16)
    {
      log_output += 1 << 12;
      long_lin = long_lin  >> 1;        
    }

    // long_lin is between 1 and 2 now (16.16 fixed point)
    // Calculate 3rd degree polynomial approximation log(x)=Polynomial(x-1) in signed long s15.16 and reduce to unsigned 4.12 at the very end. 

    long_lin = long_lin >> 1; // reduce to 17.15 bit to support signed operations here after

    long_x1 = long_lin-(32768); //(x-1) we have the decimal part in s15 now
    long_x2 = (long_x1 * long_x1) >> 15 ; // (x-1)^2
    long_x3 = (long_x2 * long_x1) >> 15 ; // (x-1)^3

    log_output += ( (POLY_A0) 
                  + ((POLY_A1 * long_x1) >> 15) 
                  + ((POLY_A2 * long_x2) >> 15) 
                  + ((POLY_A3 * long_x3) >> 15)  ) >> 3;
  }
  else
  {
    log_output=0; 
  }

  return log_output; 
}

void Application::hzToAddVal(float hz) {
  setWavetableSampleAdvance((uint16_t)(hz * HZ_ADDVAL_FACTOR));
}

void Application::playNote(float hz, uint16_t milliseconds = 500, uint8_t volume = 255) {
  vScaledVolume = volume * (volume + 2);
  hzToAddVal(hz);
  millitimer(milliseconds);
  vScaledVolume = 0;
}

void Application::playStartupSound() {
  // Nicer calibration start cue: warm rising arpeggio.
  playNote(MIDDLE_C * 0.75f, 110, 22);
  playNote(MIDDLE_C,        110, 24);
  playNote(MIDDLE_C * 1.5f, 120, 26);
  playNote(MIDDLE_C * 2.0f, 150, 30);
}

void Application::playCalibratingCountdownSound() {
  playNote(MIDDLE_C * 2.0f, 120, 24);
  playNote(MIDDLE_C * 1.6f, 120, 24);
}

void Application::playCalibrationSuccessSound() {
  // Bright upward confirmation.
  playNote(MIDDLE_C * 1.5f, 100, 26);
  playNote(MIDDLE_C * 2.0f, 100, 28);
  playNote(MIDDLE_C * 3.0f, 180, 32);
}

void Application::playCalibrationFailedSound() {
  // Clearly different: descending low "error" cue.
  playNote(MIDDLE_C * 1.2f, 130, 26);
  playNote(MIDDLE_C * 0.9f, 130, 26);
  playNote(MIDDLE_C * 0.6f, 220, 30);
}

void Application::playModeSettingSound() {
  for (int i = 0; i <= _mode; i++) {
    playNote(MIDDLE_C * 2, 200, 25);
    millitimer(100);
  }
}

void Application::delay_NOP(unsigned long time) {
  volatile unsigned long i = 0;
  for (i = 0; i < time; i++) {
      __asm__ __volatile__ ("nop");
  }
}



void Application::midi_setup() 
{
  midiTransportBegin();
  _midistate = MIDI_SILENT; 
}


void Application::midi_msg_send(uint8_t channel, uint8_t midi_cmd1, uint8_t midi_cmd2, uint8_t midi_value) 
{
  const uint8_t status = (midi_cmd1 & 0xF0) | (channel & 0x0F);
  midiTransportWrite3(status, midi_cmd2, midi_value);
}

void Application::midi_flush()
{
  midiTransportService();
}

void Application::midi_set_mute(bool muted)
{
  if (muted) {
    _mode = MUTE;
    _midistate = MIDI_STOP;
    midi_msg_send(midi_channel, 0xB0, 0x7B, 0x00);
    HW_LED1_OFF;
    HW_LED2_ON;
  } else {
    _mode = NORMAL;
    _midistate = MIDI_SILENT;
    HW_LED1_ON;
    HW_LED2_OFF;
  }
}

void Application::midi_handle_cc(uint8_t control, uint8_t value)
{
  switch (control) {
    case OT_CC_MUTE_TOGGLE:
      midi_set_mute(value >= 64);
      break;
    case OT_CC_PANIC:
      midi_msg_send(midi_channel, 0xB0, 0x7B, 0x00);
      _midistate = MIDI_SILENT;
      break;
    case OT_CC_LEGATO:
      flag_legato_on = (value >= 64) ? 1 : 0;
      break;
    case OT_CC_PITCH_BEND_ENABLE:
      flag_pitch_bend_on = (value >= 64) ? 1 : 0;
      break;
    case OT_CC_PITCH_BEND_RANGE: {
      static const uint8_t ranges[] = {1, 2, 4, 5, 7, 12, 24, 48};
      const uint8_t idx = min((uint8_t)7, (uint8_t)(value >> 4));
      midi_bend_range = ranges[idx];
      break;
    }
    case OT_CC_VOLUME_TRIGGER:
      midi_volume_trigger = value;
      break;
    case OT_CC_WAVETABLE:
      {
        uint16_t idx = ((uint16_t)value * (uint16_t)OT_WAVETABLE_COUNT) >> 7;
        if (idx >= OT_WAVETABLE_COUNT) idx = OT_WAVETABLE_COUNT - 1U;
        vWavetableSelector = (uint8_t)idx;
        ihSetWaveMorphTargetQ8(((uint16_t)vWavetableSelector) << 8);
      }
      break;
    case OT_CC_AUDIO_RATE_PRESET:
      setAudioRatePreset((uint8_t)min((uint8_t)2, (uint8_t)(value / 43)));
      break;
    case OT_CC_WAVEMORPH_ENABLE:
      ihSetWaveMorphEnabled(value >= 64);
      break;
    case OT_CC_TONETILT_ENABLE:
      ihSetToneTiltEnabled(value >= 64);
      break;
    case OT_CC_SOFTCLIP_ENABLE:
      ihSetSoftClipEnabled(value >= 64);
      break;
    case OT_CC_WAVEMORPH_SPEED: {
      const uint8_t step = 1U + (uint8_t)(value >> 3);  // 1..16
      ihSetWaveMorphStepQ8(step);
      break;
    }
    case OT_CC_TONETILT_AMOUNT:
      ihSetToneTiltWetMax((uint8_t)min((uint16_t)255U, (uint16_t)value * 2U));
      break;
    case OT_CC_SOFTCLIP_DRIVE: {
      // Higher CC = stronger clip (smaller cubic shift).
      const uint8_t shift = (uint8_t)(30U - ((uint16_t)value * 10U) / 127U); // 30..20
      ihSetSoftClipCubicShift(shift);
      break;
    }
    case OT_CC_VIBRATO_JITTER_ENABLE:
      ihSetVibratoJitterEnabled(value >= 64);
      break;
    case OT_CC_MIDI_CHANNEL_SET: {
      const uint8_t newChannel = min((uint8_t)15U, (uint8_t)(value >> 3));
      if (newChannel != midi_channel || newChannel != midi_in_channel) {
        midi_msg_send(midi_channel, 0xB0, 0x7B, 0x00);
        midi_channel = newChannel;
        midi_in_channel = newChannel;
        saveMidiChannelPersistent();
      }
      break;
    }
    case OT_CC_CAL_ARM:
      if (value == OT_CAL_ARM_KEY) {
        calibrationArmed = true;
        calibrationArmMillis = millis();
      }
      break;
    case OT_CC_CAL_CONFIRM:
      if (calibrationArmed && value == OT_CAL_CONFIRM_KEY &&
          (millis() - calibrationArmMillis) <= OT_MIDI_CAL_ARM_TIMEOUT_MS) {
        midiCalibrationRequested = true;
        _state = CALIBRATING;
        resetTimer();
      }
      calibrationArmed = false;
      break;
    default:
      break;
  }
}

void Application::midi_input_poll()
{
  if (calibrationArmed && (millis() - calibrationArmMillis) > OT_MIDI_CAL_ARM_TIMEOUT_MS) {
    calibrationArmed = false;
  }

  midiTransportService();
  uint8_t byteIn = 0;
  while (midiTransportReadByte(&byteIn)) {

    if (byteIn & 0x80) {
      midi_in_running_status = byteIn;
      const uint8_t type = midi_in_running_status & 0xF0;
      midi_in_expected = (type == 0xC0 || type == 0xD0) ? 1 : 2;
      midi_in_has_data1 = false;
      continue;
    }

    if (midi_in_running_status == 0 || midi_in_expected == 0) {
      continue;
    }

    if (!midi_in_has_data1) {
      midi_in_data1 = byteIn;
      midi_in_has_data1 = true;

      if (midi_in_expected == 1) {
        const uint8_t type = midi_in_running_status & 0xF0;
        const uint8_t channel = midi_in_running_status & 0x0F;
        if (channel == midi_in_channel && type == 0xC0) {
          const uint8_t tonePresetCount = (uint8_t)(sizeof(kTonePresets) / sizeof(kTonePresets[0]));
          if (midi_in_data1 < tonePresetCount) {
            applyTonePreset(midi_in_data1);
          }
        }
        midi_in_has_data1 = false;
      }
      continue;
    }

    const uint8_t data2 = byteIn;
    const uint8_t type = midi_in_running_status & 0xF0;
    const uint8_t channel = midi_in_running_status & 0x0F;
    if (type == 0xB0 && (channel == midi_in_channel || midi_in_data1 == OT_CC_MIDI_CHANNEL_SET)) {
      midi_handle_cc(midi_in_data1, data2);
    }
    midi_in_has_data1 = false;
  }
}

// midi_application sends note and volume and uses pitch bend to simulate continuous picth. 
// Calibrate pitch bend and other parameters accordingly to the receiver synth (see midi_calibrate). 
// New notes won't be generated as long as pitch bend will do the job. 
// The bigger is synth's pitch bend range the beter is the effect.  
void Application::midi_application ()
{
  int16_t delta_loop_cc_val = 0; 
  int16_t calculated_velocity = 0;  
  
  // Calculate loop antena cc value for midi 
  new_midi_loop_cc_val = loop_hand_pos >> 1; 
  new_midi_loop_cc_val = min (new_midi_loop_cc_val, 127);
  delta_loop_cc_val = (int16_t)new_midi_loop_cc_val - (int16_t)old_midi_loop_cc_val;

  // Calculate log freq 
  if ((vPointerIncrement < 18) || (vPointerIncrement > 65518)) 
  {
    // Lowest note
    long_log_note = 0;
  }
  else if ((vPointerIncrement > 26315) && (vPointerIncrement < 39221))
  {
    // Highest note
    long_log_note = 127; 
  }
  else if (vPointerIncrement < 32768)
  {
    // Positive frequencies
    // Find note in the playing range
    // 16795 = log2U16 (C0 [8.1758] * HZ_ADDVAL_FACTOR [2.09785])
    long_log_note = 12 * ((uint32_t) log2U16(vPointerIncrement) - 16795); // Precise note played in the logaritmic scale
  }
  else
  {
    // Negative frequencies
    // Find note in the playing range
    // 16795 = log2U16 (C0 [8.1758] * HZ_ADDVAL_FACTOR [2.09785])
    long_log_note = 12 * ((uint32_t) log2U16(65535-vPointerIncrement+1) - 16795); // Precise note played in the logaritmic scale
  }
  
  // Calculate rod antena cc value for midi 
  new_midi_rod_cc_val = (uint16_t) min((long_log_note * rod_cc_scale) >> 12, 16383); // 14 bit value

  // State machine for MIDI
  switch (_midistate)
  {
  case MIDI_SILENT:  
    // Always refresh midi loop antena cc. 
    if (new_midi_loop_cc_val != old_midi_loop_cc_val)
    {
      if (loop_midi_cc < 128)
      {
        midi_msg_send(midi_channel, 0xB0, loop_midi_cc, new_midi_loop_cc_val);
      }
      old_midi_loop_cc_val = new_midi_loop_cc_val;
    }
    else
    {
      // do nothing
    }

    // Always refresh midi rod antena cc if applicable. 
    if (new_midi_rod_cc_val != old_midi_rod_cc_val)
    {
      if (rod_midi_cc < 128) 
      {
        midi_msg_send(midi_channel, 0xB0, rod_midi_cc, (uint8_t)(new_midi_rod_cc_val >> 7));
        if (rod_midi_cc_lo < 128)
        {
          midi_msg_send(midi_channel, 0xB0, rod_midi_cc_lo, (uint8_t)(new_midi_rod_cc_val & 0x007F)); 
        }
      }
      old_midi_rod_cc_val = new_midi_rod_cc_val;
    }
    else
    {
      // do nothing
    }

    // If player's hand moves away from volume antenna
    if (new_midi_loop_cc_val > midi_volume_trigger)
    {
      // Set key follow to the minimum in order to use closest note played as the center note 
      midi_key_follow = 2048;

      // Calculate note and associated pitch bend 
      calculate_note_bend ();
      
      // Send pitch bend to reach precise played note (send 8192 (no pitch bend) in case of midi_bend_range == 1)
      midi_msg_send(midi_channel, 0xE0, midi_bend_low, midi_bend_high);
      old_midi_bend = new_midi_bend;

      // Calculate velocity
      const uint16_t midiElapsedMs = ticksToMillis(midi_timer);
      if (midiElapsedMs != 0)
      {
        calculated_velocity = ((127 - midi_volume_trigger) >> 1 ) + (VELOCITY_SENS * midi_volume_trigger * delta_loop_cc_val / midiElapsedMs);
        midi_velocity = min (abs (calculated_velocity), 127);
      }
      else 
      {
        // should not happen
        midi_velocity = 64;
      }

      
      // Play the note
      midi_msg_send(midi_channel, 0x90, new_midi_note, midi_velocity);
      old_midi_note = new_midi_note;

      _midistate = MIDI_PLAYING;
    }
    else
    {
      // Do nothing
    }
    break; 
  
  case MIDI_PLAYING:  
    // Always refresh midi loop antena cc. 
    if (new_midi_loop_cc_val != old_midi_loop_cc_val)
    {
      if (loop_midi_cc < 128)
      {
        midi_msg_send(midi_channel, 0xB0, loop_midi_cc, new_midi_loop_cc_val);
      }
      old_midi_loop_cc_val = new_midi_loop_cc_val;
    }
    else
    {
      // do nothing
    }

    // Always refresh midi rod antena cc if applicable. 
    if (new_midi_rod_cc_val != old_midi_rod_cc_val)
    {
      if (rod_midi_cc < 128) 
      {
        midi_msg_send(midi_channel, 0xB0, rod_midi_cc, (uint8_t)(new_midi_rod_cc_val >> 7));
        if (rod_midi_cc_lo < 128)
        {
          midi_msg_send(midi_channel, 0xB0, rod_midi_cc_lo, (uint8_t)(new_midi_rod_cc_val & 0x007F)); 
        }
      }
      old_midi_rod_cc_val = new_midi_rod_cc_val;
    }
    else
    {
      // do nothing
    }

    // If player's hand is far from volume antenna
    if (new_midi_loop_cc_val > midi_volume_trigger)
    {
      if ( flag_legato_on == 1)
      {
        // Set key follow so as next played note will be at limit of pitch bend range
        midi_key_follow = ((uint32_t) midi_bend_range * 4096) - PLAYER_ACCURACY;
      }
      else
      {
        // Set key follow to max so as no key follows
        midi_key_follow = 520192; // 127*2^12
      }

      // Calculate note and associated pitch bend 
      calculate_note_bend (); 
      
      // Refresh midi pitch bend value
      if (new_midi_bend != old_midi_bend)
      {
        midi_msg_send(midi_channel, 0xE0, midi_bend_low, midi_bend_high);   
        old_midi_bend = new_midi_bend;
      }
      else
      {
        // do nothing
      } 
      
      // Refresh midi note
      if (new_midi_note != old_midi_note) 
      {
        // Play new note before muting old one to play legato on monophonic synth 
        // (pitch pend management tends to break expected effect here)
        midi_msg_send(midi_channel, 0x90, new_midi_note, midi_velocity);
        midi_msg_send(midi_channel, 0x90, old_midi_note, 0);
        old_midi_note = new_midi_note;
      }
      else 
      {
        // do nothing
      } 
    }
    else // Means that player's hand moves to the volume antenna
    {
      // Send note off
      midi_msg_send(midi_channel, 0x90, old_midi_note, 0);

      _midistate = MIDI_SILENT;
    }
    break;
    
  case MIDI_STOP:
    // Send all note off
    midi_msg_send(midi_channel, 0xB0, 0x7B, 0x00);

    _midistate = MIDI_MUTE;
    break;

  case MIDI_MUTE:
    //do nothing
    break;
    
  }
}

void Application::calculate_note_bend ()
{
  int32_t long_log_bend;
  int32_t long_norm_log_bend;
    
  long_log_bend = ((int32_t)long_log_note) - (((int32_t) old_midi_note) * 4096); // How far from last played midi chromatic note we are 

  // If too much far from last midi chromatic note played (midi_key_follow depends on pitch bend range)
  if ((abs (long_log_bend) >= midi_key_follow) && (midi_key_follow != 520192))
  {
    new_midi_note = (uint8_t) ((long_log_note + 2048) >> 12);  // Select the new midi chromatic note - round to integer part by adding 1/2 before shifting
    long_log_bend = ((int32_t) long_log_note) - (((int32_t) new_midi_note) * 4096); // calculate bend to reach precise note played
  }
  else
  {
     new_midi_note = old_midi_note; // No change 
  }

  // If pitch bend activated 
  if (flag_pitch_bend_on == 1)
  {
    // use it to reach precise note played
    long_norm_log_bend = (long_log_bend / midi_bend_range);
    if (long_norm_log_bend > 4096)
    {
      long_norm_log_bend = 4096; 
    }
    else if (long_norm_log_bend < -4096)
    {
      long_norm_log_bend = -4096; 
    }
    new_midi_bend = 8192 + ((long_norm_log_bend * 8191) >> 12); // Calculate midi pitch bend
  }
  else
  {
    // Don't use pitch bend 
    new_midi_bend = 8192; 
  }
  

  // Prepare the 2 bites of picth bend midi message
  midi_bend_low = (int8_t) (new_midi_bend & 0x007F);
  midi_bend_high = (int8_t) ((new_midi_bend & 0x3F80)>> 7);
}



void Application::init_parameters ()
{
  // init data pot value to avoid 1st position to be taken into account

  param_pot_value = readPotLegacy(REGISTER_SELECT_POT);
  old_param_pot_value = param_pot_value;

  data_pot_value = readPotLegacy(WAVE_SELECT_POT);
  old_data_pot_value = data_pot_value;
}

void Application::set_parameters ()
{
  uint16_t data_steps;
  uint8_t paramIndex;
  uint8_t paramSlot;
  const bool audioActive = (_mode == NORMAL);
  const uint8_t kParamSlotCount = audioActive ? 4U : 5U;
  static const uint8_t kAudioParamMap[4] = {0U, 1U, 2U, 8U};
  static const uint8_t kMidiParamMap[5] = {3U, 4U, 5U, 6U, 7U};
  
  param_pot_value = readPotLegacy(REGISTER_SELECT_POT);
  data_pot_value = readPotLegacy(WAVE_SELECT_POT);
  paramSlot = (uint8_t)(((uint32_t)param_pot_value * kParamSlotCount) >> 10);
  if (paramSlot >= kParamSlotCount) {
    paramSlot = kParamSlotCount - 1U;
  }
  paramIndex = audioActive ? kAudioParamMap[paramSlot] : kMidiParamMap[paramSlot];

  // If parameter pot moved
  if (abs((int32_t)param_pot_value - (int32_t)old_param_pot_value) >= 8)
  {
    // Function pot feedback: red LED only.
    resetTimer();
    if ((paramSlot % 2U) == 0U)
    {
      HW_LED1_OFF;
    }
    else
    {
      HW_LED1_ON;
    }
    HW_LED2_OFF;

    // Memorize data pot value to monitor changes
    old_param_pot_value = param_pot_value;
  }
  
  // Else If data pot moved
  else if (abs((int32_t)data_pot_value - (int32_t)old_data_pot_value) >= 8)
  {
    bool resetAudioTimerAfterParam = true;
    // Modify selected parameter
    switch (paramIndex)
    {
    case 0:
      // Transpose
      switch (data_pot_value >> 8)
      {
      case 0:
        registerValue=3; // -1 Octave
        data_steps = 1;
        break; 
      case 1:
      case 2:
        registerValue=2; // Center
        data_steps = 2;
        break; 
      default:
        registerValue=1; // +1 Octave 
        data_steps = 3;
        break; 
      }
      break;
      
    case 1:
      // Tone preset
      {
        const uint8_t presetCount = (uint8_t)(sizeof(kTonePresets) / sizeof(kTonePresets[0]));
        data_steps = (uint16_t)(((uint32_t)data_pot_value * (uint32_t)presetCount) >> 10);
        if (data_steps >= presetCount) {
          data_steps = presetCount - 1U;
        }
        applyTonePreset((uint8_t)data_steps);
      }
      break;

    case 2:
      // Waveform
      {
        const uint16_t tableMaxQ8 = (uint16_t)((OT_WAVETABLE_COUNT - 1U) << 8);
        const uint16_t morphTargetQ8 = (uint16_t)(((uint32_t)data_pot_value * (uint32_t)tableMaxQ8) / 1023U);
        data_steps = (uint16_t)(morphTargetQ8 >> 8);
        if (data_steps >= OT_WAVETABLE_COUNT) {
          data_steps = OT_WAVETABLE_COUNT - 1U;
        }
        vWavetableSelector = (uint8_t)data_steps;
        ihSetWaveMorphTargetQ8(morphTargetQ8);
        resetAudioTimerAfterParam = false;
      }
      resetAudioFeatureDefaults();
      break;
        
    case 3:
      // Rod antenna mode
      data_steps = data_pot_value >> 8;
      switch (data_steps)
      {
      case 0:
        flag_legato_on = 0;
        flag_pitch_bend_on = 0;
        break; 
      case 1:
        flag_legato_on = 0;
        flag_pitch_bend_on = 1;
        break; 
      case 2:
        flag_legato_on = 1;
        flag_pitch_bend_on = 0;
        break; 
      default:
        flag_legato_on = 1;
        flag_pitch_bend_on = 1;
        break;  
      }
      break;
      
    case 4:
      // Pitch-Bend range
      data_steps = data_pot_value >> 7;
      switch (data_steps)
      {
      case 0:
        midi_bend_range = 1; 
        break; 
      case 1:
        midi_bend_range = 2; 
        break; 
      case 2:
        midi_bend_range = 4; 
        break; 
      case 3:
        midi_bend_range = 5; 
        break; 
      case 4:
        midi_bend_range = 7; 
        break; 
      case 5:
        midi_bend_range = 12; 
        break; 
      case 6:
        midi_bend_range = 24; 
        break;  
      default:
        midi_bend_range = 48; 
        break;  
      }
      break;
      
    case 5:
      // Volume trigger
      data_steps = data_pot_value >> 8;
      midi_volume_trigger = (uint8_t)((data_pot_value >> 3) & 0x007F);
      break;
      
    case 6:
      //Rod antenna cc
      data_steps = data_pot_value >> 7;
      switch (data_steps)
      {
      case 0:
        rod_midi_cc = 255; // Nothing
        rod_midi_cc_lo = 255; // Nothing
        rod_cc_scale = 128;
        break; 
      case 1:
        rod_midi_cc = 8; // Balance
        rod_midi_cc_lo = 255; // No least significant bits
        rod_cc_scale = 128;
        break; 
      case 2:
        rod_midi_cc = 10; // Pan
        rod_midi_cc_lo = 255; // No least significant bits
        rod_cc_scale = 128;
        break; 
      case 3:
        rod_midi_cc = 16; // General Purpose 1 (14 Bits)
        rod_midi_cc_lo = 48; // General Purpose 1 least significant bits
        rod_cc_scale = 128;
        break; 
      case 4:
        rod_midi_cc = 17; // General Purpose 2 (14 Bits)
        rod_midi_cc_lo = 49; // General Purpose 2 least significant bits
        rod_cc_scale = 128;
        break; 
      case 5:
        rod_midi_cc = 18; // General Purpose 3 (7 Bits)
        rod_midi_cc_lo = 255; // No least significant bits
        rod_cc_scale = 128;
        break; 
      case 6:
        rod_midi_cc = 19; // General Purpose 4 (7 Bits)
        rod_midi_cc_lo = 255; // No least significant bits
        rod_cc_scale = 128;
        break; 
      default:
        rod_midi_cc = 74; // Cutoff (exists of both loop and rod)
        rod_midi_cc_lo = 255; // No least significant bits
        rod_cc_scale = 128;
        break; 
      }
      break;
      
          
    case 7:
      // Loop antenna cc
      data_steps = data_pot_value >> 7;
      switch (data_steps)
      {
      case 0:
        loop_midi_cc = 1; // Modulation
        break; 
      case 1:
        loop_midi_cc = 7; // Volume
        break; 
      case 2:
        loop_midi_cc = 11; // Expression
        break; 
      case 3:
        loop_midi_cc = 71; // Resonnance
        break; 
      case 4:
        loop_midi_cc = 74; // Cutoff (exists of both loop and rod)
        break; 
      case 5:
        loop_midi_cc = 91; // Reverb
        break; 
      case 6:
        loop_midi_cc = 93; // Chorus
        break; 
      default:
        loop_midi_cc = 95; // Phaser
        break; 
      }
      break;

    default:
      // Master out gain (new last parameter slot): 10% .. 100% (Q8: 26..256)
      data_steps = data_pot_value >> 7;
      ihSetMasterOutGainQ8((uint16_t)(26U + ((uint32_t)data_pot_value * 230U) / 1023U));
      break;
    }

    // Value pot feedback: yellow LED only.
    if (resetAudioTimerAfterParam) {
      resetTimer();
    }
    if ((data_steps % 2U) == 0U)
    {
      HW_LED2_OFF;
    }
    else
    {
      HW_LED2_ON;
    }
    HW_LED1_OFF;


    // Memorize data pot value to monitor changes
    old_data_pot_value = data_pot_value;
  }

  else
  {
    if (timerExpiredMillis(OT_LED_RESTORE_MS))
    //restore LED status
    {
      if (_mode == NORMAL)
      {
        HW_LED1_ON;
        HW_LED2_OFF;
      }
      else
      {
        HW_LED1_OFF;
        HW_LED2_ON;
      }
    }
  }
}
