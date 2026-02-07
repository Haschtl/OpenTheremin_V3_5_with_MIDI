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
const int16_t PitchFreqOffset = 700;
const int16_t VolumeFreqOffset = 700;
const int8_t HYST_VAL = 40;

static int32_t pitchCalibrationBase = 0;
static int32_t pitchCalibrationBaseFreq = 0;
static int32_t pitchCalibrationConstant = 0;
static int32_t pitchSensitivityConstant = 70000;
static int16_t pitchDAC = 0;
static int16_t volumeDAC = 0;
static float qMeasurement = 0;

static int32_t volCalibrationBase   = 0;

static uint8_t new_midi_note =0;
static uint8_t old_midi_note =0;

static uint8_t new_midi_loop_cc_val =0;
static uint8_t old_midi_loop_cc_val =0;

static uint8_t midi_velocity = 0;

static uint8_t loop_hand_pos = 0; 

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

static volatile uint32_t pitch_measure_edges = 0;
static volatile uint32_t volume_measure_edges = 0;
static uint8_t midi_in_running_status = 0;
static uint8_t midi_in_data1 = 0;
static uint8_t midi_in_expected = 0;
static bool midi_in_has_data1 = false;

#if OT_HAS_NATIVE_USB_MIDI
static uint8_t midi_usb_payload[3] = {0, 0, 0};
static uint8_t midi_usb_payload_size = 0;
static uint8_t midi_usb_payload_pos = 0;
#endif

static inline void midiTransportBegin() {
#if OT_HAS_NATIVE_USB_MIDI
  // USB stack is started by core init.
#else
  #if OT_DEBUG_ENABLED
  OT_DEBUG_PRINTLN_F("[DBG] midi transport disabled in DEBUG build");
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
  #if OT_DEBUG_ENABLED
  (void)status;
  (void)data1;
  (void)data2;
  #else
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
  #if OT_DEBUG_ENABLED
  (void)out;
  return false;
  #else
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

static void onPitchMeasureEdge() {
  pitch_measure_edges++;
}

static void onVolumeMeasureEdge() {
  volume_measure_edges++;
}

static unsigned long countEdgesForMs(uint8_t pin, void (*isr)(), volatile uint32_t *edgeCounter, uint16_t gateMs) {
  const pin_size_t irq = digitalPinToInterrupt(pin);

  noInterrupts();
  *edgeCounter = 0;
  interrupts();

  attachInterrupt(irq, isr, RISING);
  delay(gateMs);
  detachInterrupt(irq);

  noInterrupts();
  const uint32_t edges = *edgeCounter;
  interrupts();

  return (edges * 1000UL) / gateMs;
}

static bool waitForFlagOrTimeout(volatile bool *flag, uint16_t timeoutMs) {
  const uint32_t t0 = millis();
  while (!(*flag)) {
    if ((uint32_t)(millis() - t0) >= (uint32_t)timeoutMs) {
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
  OT_DEBUG_PRINTLN_F("[DBG] setup: interrupts ready");


  EEPROM.get(4,pitchCalibrationBase);
  EEPROM.get(8,volCalibrationBase);
 
 init_parameters();
 loadMidiChannelPersistent();
 OT_DEBUG_PRINT("[DBG] setup: midi channel=");
 OT_DEBUG_PRINTLN((int)midi_channel);
 resetAudioFeatureDefaults();
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

unsigned long Application::GetQMeasurement()
{
  // On non-AVR targets we use API timers, so no oscillator-to-clock correction is needed.
  return 16000000UL;

}


unsigned long Application::GetPitchMeasurement()
{
  return countEdgesForMs(OT_PITCH_CAPTURE_PIN, onPitchMeasureEdge, &pitch_measure_edges, 1000);

}

unsigned long Application::GetVolumeMeasurement()
{
  return countEdgesForMs(OT_VOLUME_CAPTURE_PIN, onVolumeMeasureEdge, &volume_measure_edges, 1000);
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
  static bool calButtonLatch = false;
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
  static uint16_t dbgPitchRaw = 0;
  static int32_t dbgPitchFiltered = 0;
  static uint16_t dbgVolRaw = 0;
  static int32_t dbgVolFiltered = 0;
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
      OT_DEBUG_PRINTLN((int)_state);
    }
  }
#endif

  pitchPotValue    = readPotLegacy(PITCH_POT);
  volumePotValue   = readPotLegacy(VOLUME_POT);
  
  set_parameters ();
  serviceErrorIndicator();
  ihRecoverTimer();
  midi_input_poll();
  midi_flush();

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
    OT_DEBUG_PRINTLN_F("[DBG] calibration: arm");
    resetTimer();
  }

  if (_state == CALIBRATING && HW_BUTTON_RELEASED && !midiCalibrationRequested) 
  {
    if (timerExpiredMillis(OT_MODE_TOGGLE_PRESS_MS)) 
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

  if (_state == CALIBRATING && (midiCalibrationRequested || timerExpiredMillis(OT_CALIBRATION_PRESS_MS))) 
  {
    OT_DEBUG_PRINTLN_F("[DBG] calibration: start");
    HW_LED1_OFF; HW_LED2_ON;
  
      
    OT_DEBUG_PRINTLN_F("[DBG] calibration: startup sound");
    playStartupSound();
	
    // calibrate heterodyne parameters
    OT_DEBUG_PRINTLN_F("[DBG] calibration: calibrate pitch");
    calibrate_pitch();
    OT_DEBUG_PRINTLN_F("[DBG] calibration: calibrate volume");
    calibrate_volume();


    OT_DEBUG_PRINTLN_F("[DBG] calibration: reinit timer/interrupts");
    initialiseTimer();
    initialiseInterrupts();
   
    OT_DEBUG_PRINTLN_F("[DBG] calibration: countdown sound");
    playCalibratingCountdownSound();
    OT_DEBUG_PRINTLN_F("[DBG] calibration: baseline capture");
    calibrate();
  
    _mode=NORMAL;
    HW_LED1_ON;HW_LED2_OFF;

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
    _state = PLAYING;
  };

#if CV_ENABLED
  #error "CV_ENABLED is not supported on UNO R4 backend"
#endif



  if (pitchValueAvailable) {                        // If capture event

#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
    dbgPitchRaw = pitch;
#endif
    pitch_v=pitch;                         // Averaging pitch values
    pitch_v=pitch_l+((pitch_v-pitch_l)>>2);
    pitch_l=pitch_v;
#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
    dbgPitchFiltered = pitch_v;
#endif


//HW_LED2_ON;


    // set wave frequency for each mode
    switch (_mode) {
      case MUTE : /* NOTHING! */;                                        break;
      case NORMAL      : setWavetableSampleAdvance(((pitchCalibrationBase-pitch_v)+2048-(pitchPotValue<<2))>>registerValue); break;
    };
    
  //  HW_LED2_OFF;

    pitchValueAvailable = false;
  }

  if (volumeValueAvailable) {
    vol = max(vol, 5000);
#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
    dbgVolRaw = vol;
#endif

    vol_v=vol;                  // Averaging volume values
    vol_v=vol_l+((vol_v-vol_l)>>2);
    vol_l=vol_v;
#if OT_DEBUG_ENABLED && OT_DEBUG_RUNTIME_LOGS
    dbgVolFiltered = vol_v;
#endif

    switch (_mode) {
      case MUTE:  vol_v = 0;                                                      break;
      case NORMAL:      vol_v = MAX_VOLUME-(volCalibrationBase-vol_v)/2+(volumePotValue<<2)-1024;                                     break;
    };

    // Limit and set volume value
    vol_v = min(vol_v, 4095);
    //    vol_v = vol_v - (1 + MAX_VOLUME - (volumePotValue << 2));
    vol_v = vol_v ;
    vol_v = max(vol_v, 0);
    loop_hand_pos = vol_v >> 4;

    // Exponential-ish volume curve with full 16-bit headroom.
    vScaledVolume = (uint16_t)((uint32_t)loop_hand_pos * (uint32_t)loop_hand_pos);
    
    volumeValueAvailable = false;
  }

  if (midi_timer >= millisToTicks(OT_MIDI_UPDATE_MS))
  {
    midi_application ();
    midi_timer = 0; 
  }

  goto mloop;                           // End of main loop
}

void Application::calibrate()
{
  resetPitchFlag();
  resetTimer();
  savePitchCounter();
  if (!waitForFlagOrTimeout(&pitchValueAvailable, 20U)) {
    OT_DEBUG_PRINTLN_F("[DBG] calibrate: pitch flag timeout");
  }
  pitchCalibrationBase = pitch;
  if (pitchCalibrationBase <= 0) {
    pitchCalibrationBase = 1;
    OT_DEBUG_PRINTLN_F("[DBG] calibrate: pitch base clamped to 1");
  }
  pitchCalibrationBaseFreq = FREQ_FACTOR/pitchCalibrationBase;
  pitchCalibrationConstant = FREQ_FACTOR/pitchSensitivityConstant/2+200;

  resetVolFlag();
  resetTimer();
  saveVolCounter();
  if (!waitForFlagOrTimeout(&volumeValueAvailable, 20U)) {
    OT_DEBUG_PRINTLN_F("[DBG] calibrate: volume flag timeout");
  }
  volCalibrationBase = vol;
  
  
  EEPROM.put(4,pitchCalibrationBase);
  EEPROM.put(8,volCalibrationBase);

  OT_DEBUG_PRINT("[DBG] calibrate result basePitch=");
  OT_DEBUG_PRINT((long)pitchCalibrationBase);
  OT_DEBUG_PRINT(" baseVol=");
  OT_DEBUG_PRINT((long)volCalibrationBase);
  OT_DEBUG_PRINT(" pitchBaseFreq=");
  OT_DEBUG_PRINT((long)pitchCalibrationBaseFreq);
  OT_DEBUG_PRINT(" pitchConst=");
  OT_DEBUG_PRINTLN((long)pitchCalibrationConstant);
  
}

void Application::calibrate_pitch()
{
  
static int16_t pitchXn0 = 0;
static int16_t pitchXn1 = 0;
static int16_t pitchXn2 = 0;
static float q0 = 0;
static long pitchfn0 = 0;
static long pitchfn1 = 0;
static long pitchfn = 0;


  // limit the number of calibration iteration to 12 
  // the algorythm used is normaly faster than dichotomy which normaly finds a 12Bit number in 12 iterations max
  static uint16_t l_iteration_pitch = 0;
  
  InitialisePitchMeasurement();
  interrupts();
  SPImcpDACinit();

  qMeasurement = GetQMeasurement();  // Measure Arudino clock frequency 

q0 = (16000000/qMeasurement*500000);  //Calculated set frequency based on Arudino clock frequency

pitchXn0 = 0;
pitchXn1 = 4095;

pitchfn = q0-PitchFreqOffset;        // Add offset calue to set frequency



SPImcpDAC2Bsend(1600);

SPImcpDAC2Asend(pitchXn0);
delay(100);
pitchfn0 = GetPitchMeasurement();

SPImcpDAC2Asend(pitchXn1);
delay(100);
pitchfn1 = GetPitchMeasurement();

 
l_iteration_pitch = 0;
while ((abs(pitchfn0 - pitchfn1) > PitchCalibrationTolerance) && (l_iteration_pitch < 12))
{      
      
SPImcpDAC2Asend(pitchXn0);
delay(100);
pitchfn0 = GetPitchMeasurement()-pitchfn;

SPImcpDAC2Asend(pitchXn1);
delay(100);
pitchfn1 = GetPitchMeasurement()-pitchfn;

pitchXn2=pitchXn1-((pitchXn1-pitchXn0)*pitchfn1)/(pitchfn1-pitchfn0); // new DAC value


pitchXn0 = pitchXn1;
pitchXn1 = pitchXn2;

HW_LED2_TOGGLE;
      
l_iteration_pitch ++;
}
      
delay(100);

EEPROM.put(0,pitchXn0);

OT_DEBUG_PRINT("[DBG] calibrate_pitch result eepromPitchDAC=");
OT_DEBUG_PRINT((int)pitchXn0);
OT_DEBUG_PRINT(" q0=");
OT_DEBUG_PRINT((long)q0);
OT_DEBUG_PRINT(" target=");
OT_DEBUG_PRINT(pitchfn);
OT_DEBUG_PRINT(" f0=");
OT_DEBUG_PRINT(pitchfn0);
OT_DEBUG_PRINT(" f1=");
OT_DEBUG_PRINT(pitchfn1);
OT_DEBUG_PRINT(" iterations=");
OT_DEBUG_PRINTLN((unsigned int)l_iteration_pitch);
  
}

void Application::calibrate_volume()
{


static int16_t volumeXn0 = 0;
static int16_t volumeXn1 = 0;
static int16_t volumeXn2 = 0;
static float q0 = 0;
static long volumefn0 = 0;
static long volumefn1 = 0;
static long volumefn = 0;

  // limit the number of calibration iteration to 12 
  // the algorythm used is normaly faster than dichotomy which normaly finds a 12Bit number in 12 iterations max
  static uint16_t l_iteration_volume = 0; 
    
  InitialiseVolumeMeasurement();
  interrupts();
  SPImcpDACinit();


volumeXn0 = 0;
volumeXn1 = 4095;

q0 = (16000000/qMeasurement*460765);
volumefn = q0-VolumeFreqOffset;



SPImcpDAC2Bsend(volumeXn0);
delay_NOP(44316);//44316=100ms

volumefn0 = GetVolumeMeasurement();

SPImcpDAC2Bsend(volumeXn1);

delay_NOP(44316);//44316=100ms
volumefn1 = GetVolumeMeasurement();



l_iteration_volume = 0;
while ((abs(volumefn0 - volumefn1) > VolumeCalibrationTolerance) && (l_iteration_volume < 12))
{

SPImcpDAC2Bsend(volumeXn0);
delay_NOP(44316);//44316=100ms
volumefn0 = GetVolumeMeasurement()-volumefn;

SPImcpDAC2Bsend(volumeXn1);
delay_NOP(44316);//44316=100ms
volumefn1 = GetVolumeMeasurement()-volumefn;

volumeXn2=volumeXn1-((volumeXn1-volumeXn0)*volumefn1)/(volumefn1-volumefn0); // calculate new DAC value


volumeXn0 = volumeXn1;
volumeXn1 = volumeXn2;

HW_LED2_TOGGLE;

l_iteration_volume ++;
}

EEPROM.put(2,volumeXn0);

OT_DEBUG_PRINT("[DBG] calibrate_volume result eepromVolDAC=");
OT_DEBUG_PRINT((int)volumeXn0);
OT_DEBUG_PRINT(" q0=");
OT_DEBUG_PRINT((long)q0);
OT_DEBUG_PRINT(" target=");
OT_DEBUG_PRINT(volumefn);
OT_DEBUG_PRINT(" f0=");
OT_DEBUG_PRINT(volumefn0);
OT_DEBUG_PRINT(" f1=");
OT_DEBUG_PRINT(volumefn1);
OT_DEBUG_PRINT(" iterations=");
OT_DEBUG_PRINTLN((unsigned int)l_iteration_volume);


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
  playNote(MIDDLE_C, 150, 25);
  playNote(MIDDLE_C * 2, 150, 25);
  playNote(MIDDLE_C * 4, 150, 25);
}

void Application::playCalibratingCountdownSound() {
  playNote(MIDDLE_C * 2, 150, 25);
  playNote(MIDDLE_C * 2, 150, 25);
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
  
  param_pot_value = readPotLegacy(REGISTER_SELECT_POT);
  data_pot_value = readPotLegacy(WAVE_SELECT_POT);

  // If parameter pot moved
  if (abs((int32_t)param_pot_value - (int32_t)old_param_pot_value) >= 8)
  {
    // Blink the LED relatively to pot position
    resetTimer();
    if (((param_pot_value >> 7) % 2) == 0)
    {
      HW_LED1_OFF;
      HW_LED2_OFF;
    }
    else
    {
      HW_LED1_ON;
      HW_LED2_ON;
    }

    // Memorize data pot value to monitor changes
    old_param_pot_value = param_pot_value;
  }
  
  // Else If data pot moved
  else if (abs((int32_t)data_pot_value - (int32_t)old_data_pot_value) >= 8)
  {
    // Modify selected parameter
    switch (param_pot_value >> 7)
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
      data_steps = (uint16_t)(((uint32_t)data_pot_value * (uint32_t)OT_WAVETABLE_COUNT) >> 10);
      if (data_steps >= OT_WAVETABLE_COUNT) {
        data_steps = OT_WAVETABLE_COUNT - 1U;
      }
      vWavetableSelector = (uint8_t)data_steps;
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
      
          
    default:
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
    }

    // Blink the LED relatively to pot position
    resetTimer();
    if ((data_steps % 2) == 0)
    {
      HW_LED1_OFF;
      HW_LED2_OFF;
    }
    else
    {
      HW_LED1_ON;
      HW_LED2_ON;
    }


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
