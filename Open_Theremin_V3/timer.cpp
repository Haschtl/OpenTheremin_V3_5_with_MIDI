#include "Arduino.h"

#include "timer.h"
#include "ihandlers.h"

volatile uint32_t timer = 0;
volatile uint32_t midi_timer = 0;

uint32_t millisToTicks(uint16_t milliseconds) {
  const uint32_t tickHz = ihGetAudioTickHz();
  return ((uint32_t)milliseconds * tickHz + 999U) / 1000U;
}

uint16_t ticksToMillis(uint32_t ticks) {
  const uint32_t tickHz = ihGetAudioTickHz();
  const uint32_t milliseconds = (ticks * 1000U + (tickHz / 2U)) / tickHz;
  return (milliseconds > 0xFFFFU) ? 0xFFFFU : (uint16_t)milliseconds;
}

void ticktimer (uint32_t ticks) {
  resetTimer();
  while (timerUnexpired(ticks))
    ;  // NOP
};

void millitimer (uint16_t milliseconds) {
  ticktimer(millisToTicks(milliseconds));
}


