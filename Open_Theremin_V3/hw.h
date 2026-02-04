#ifndef _HW_H
#define _HW_H

#include "pins.h"

inline bool hwButtonPressed() {
  return digitalRead(OT_BUTTON_PIN) == LOW;
}

inline void hwLed1Set(bool on) {
  digitalWrite(OT_LED1_PIN, on ? HIGH : LOW);
}

inline void hwLed2Set(bool on) {
  digitalWrite(OT_LED2_PIN, on ? HIGH : LOW);
}

inline void hwLed1Toggle() {
  digitalWrite(OT_LED1_PIN, !digitalRead(OT_LED1_PIN));
}

inline void hwLed2Toggle() {
  digitalWrite(OT_LED2_PIN, !digitalRead(OT_LED2_PIN));
}

#define HW_BUTTON_STATE    (digitalRead(OT_BUTTON_PIN))
#define HW_BUTTON_PRESSED  (hwButtonPressed())
#define HW_BUTTON_RELEASED (!hwButtonPressed())

#define HW_LED1_ON         (hwLed1Set(true))
#define HW_LED1_OFF        (hwLed1Set(false))

#define HW_LED2_ON         (hwLed2Set(true))
#define HW_LED2_OFF        (hwLed2Set(false))

#define HW_LED1_TOGGLE     (hwLed1Toggle())
#define HW_LED2_TOGGLE     (hwLed2Toggle())

#endif // _HW_H

