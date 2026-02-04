#ifndef _TIMER_H
#define _TIMER_H

#include "build.h"

extern volatile uint32_t timer;
extern volatile uint32_t midi_timer;

uint32_t millisToTicks(uint16_t milliseconds);
uint16_t ticksToMillis(uint32_t ticks);

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
