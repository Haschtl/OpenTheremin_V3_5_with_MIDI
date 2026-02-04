#include "error_indicator.h"

#include "hw.h"

static volatile uint8_t g_errorCode = OT_ERR_NONE;

void setErrorIndicator(uint8_t code) {
  if (code == OT_ERR_NONE) {
    return;
  }
  g_errorCode = code;
}

void clearErrorIndicator() {
  g_errorCode = OT_ERR_NONE;
}

uint8_t getErrorIndicator() {
  return g_errorCode;
}

void serviceErrorIndicator() {
  const uint8_t code = g_errorCode;
  if (code == OT_ERR_NONE) {
    return;
  }

  const uint32_t onMs = 120U;
  const uint32_t offMs = 120U;
  const uint32_t gapMs = 520U;
  const uint32_t slotMs = onMs + offMs;
  const uint32_t cycleMs = (uint32_t)code * slotMs + gapMs;
  const uint32_t phase = millis() % cycleMs;

  const uint32_t blinkIdx = phase / slotMs;
  const uint32_t inSlotMs = phase % slotMs;
  const bool blinkOn = (blinkIdx < code) && (inSlotMs < onMs);

  if (blinkOn) {
    HW_LED1_ON;
    HW_LED2_ON;
  } else {
    HW_LED1_OFF;
    HW_LED2_OFF;
  }
}

[[noreturn]] void fatalErrorLoop(uint8_t code) {
  setErrorIndicator(code);
  pinMode(OT_LED1_PIN, OUTPUT);
  pinMode(OT_LED2_PIN, OUTPUT);
  while (true) {
    serviceErrorIndicator();
    delay(10);
  }
}
