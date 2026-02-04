#ifndef _TIMER_H
#define _TIMER_H

#include "build.h"

extern volatile uint32_t timer;
extern volatile uint32_t midi_timer;

inline uint32_t millisToTicks(uint16_t milliseconds) {
  return ((uint32_t)milliseconds * (uint32_t)OT_AUDIO_TICK_HZ + 999U) / 1000U;
}

inline uint16_t ticksToMillis(uint32_t ticks) {
  const uint32_t milliseconds = (ticks * 1000U + ((uint32_t)OT_AUDIO_TICK_HZ / 2U)) / (uint32_t)OT_AUDIO_TICK_HZ;
  return (milliseconds > 0xFFFFU) ? 0xFFFFU : (uint16_t)milliseconds;
}

inline void resetTimer() {
  timer = 0;
}

inline void incrementTimer() {
  timer++;
}

inline void incrementMidiTimer() {
  midi_timer++;
}

inline bool timerExpired(uint32_t ticks) {
  return timer >= ticks;
}

inline bool timerUnexpired(uint32_t ticks) {
  return timer < ticks;
}

inline bool timerExpiredMillis(uint16_t milliseconds) {
  return timerExpired(millisToTicks(milliseconds));
}

inline bool timerUnexpiredMillis(uint16_t milliseconds) {
  return timerUnexpired(millisToTicks(milliseconds));
}

void ticktimer (uint32_t ticks);
void millitimer (uint16_t milliseconds);


#endif // _TIMER_H
