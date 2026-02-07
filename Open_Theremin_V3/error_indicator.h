#ifndef _ERROR_INDICATOR_H
#define _ERROR_INDICATOR_H

#include <Arduino.h>

enum OtErrorIndicatorCode : uint8_t {
  OT_ERR_NONE = 0,
  OT_ERR_DMA = 1,
  OT_ERR_TIMER = 2,
  OT_ERR_AUDIO_RATE = 3,
  OT_ERR_CALIB = 4,
};

void setErrorIndicator(uint8_t code);
void clearErrorIndicator();
uint8_t getErrorIndicator();
void serviceErrorIndicator();
[[noreturn]] void fatalErrorLoop(uint8_t code);

#endif // _ERROR_INDICATOR_H
