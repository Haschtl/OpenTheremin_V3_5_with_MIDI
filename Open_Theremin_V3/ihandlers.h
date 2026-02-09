#ifndef _IHANDLERS_H
#define _IHANDLERS_H

#include "Arduino.h"

extern volatile uint32_t pitch;              // Pitch value
extern volatile uint32_t vol;                // Volume value
extern volatile uint16_t vScaledVolume;      // Volume byte

extern volatile uint32_t pitch_counter;      // Pitch counter
extern volatile uint32_t pitch_counter_l;    // Last value of pitch counter

extern volatile uint32_t vol_counter;      // Pitch counter
extern volatile uint32_t vol_counter_l;    // Last value of pitch counter

extern volatile bool volumeValueAvailable;   // Volume read flag
extern volatile bool pitchValueAvailable;    // Pitch read flag
extern volatile bool reenableInt1;    // Pitch read flag

extern volatile uint8_t  vWavetableSelector;
extern volatile uint16_t vPointerIncrement;  // Table pointer increment

inline void resetPitchFlag()   { pitchValueAvailable = false; }
inline void resetVolFlag()     { volumeValueAvailable = false; }

inline void savePitchCounter() { pitch_counter_l=pitch_counter; }
inline void saveVolCounter()   { vol_counter_l=vol_counter; };

inline void setWavetableSampleAdvance(uint16_t val) { vPointerIncrement = val;}

void ihDisableInt1();
void ihEnableInt1();
inline void disableInt1() { ihDisableInt1(); }
inline void enableInt1()  { ihEnableInt1(); }

void ihInitialiseTimer();
void ihInitialiseInterrupts();
void ihInitialisePitchMeasurement();
void ihInitialiseVolumeMeasurement();
void ihRecoverTimer();
uint32_t ihGetAudioTickHz();
bool ihSetAudioTickHz(uint32_t hz);
uint32_t ihGetWaveTickCount();
uint32_t ihGetPitchCaptureCount();
uint32_t ihGetVolumeCaptureCount();
void ihSetWaveMorphEnabled(bool enabled);
void ihSetToneTiltEnabled(bool enabled);
void ihSetSoftClipEnabled(bool enabled);
void ihSetWaveMorphStepQ8(uint8_t stepQ8);
void ihSetToneTiltWetMax(uint8_t wetMax);
void ihSetSoftClipCubicShift(uint8_t cubicShift);
void ihSetVibratoJitterEnabled(bool enabled);
bool ihGetWaveMorphEnabled();
bool ihGetToneTiltEnabled();
bool ihGetSoftClipEnabled();
bool ihGetVibratoJitterEnabled();
uint8_t ihGetWaveMorphStepQ8();
uint8_t ihGetToneTiltWetMax();
uint8_t ihGetSoftClipCubicShift();
void ihSetMasterOutGainQ8(uint16_t gainQ8);
uint16_t ihGetMasterOutGainQ8();
void ihSetOutputFadeGateQ8(uint16_t gateQ8);
uint16_t ihGetOutputFadeGateQ8();
void resetPitchFlag();
void resetVolFlag();
void savePitchCounter();
void saveVolCounter();

#endif // _IHANDLERS_H
