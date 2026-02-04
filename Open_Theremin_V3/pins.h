#ifndef _OT_PINS_H
#define _OT_PINS_H

#include <Arduino.h>

#if !defined(ARDUINO_UNOR4_MINIMA) && !defined(ARDUINO_UNOR4_WIFI)
#error "This firmware targets Arduino UNO R4 (Minima/WiFi) only."
#endif

// Shield wiring for the UNO R4 port.
static const uint8_t OT_BUTTON_PIN = 6;
static const uint8_t OT_LED1_PIN = 18;  // A4
static const uint8_t OT_LED2_PIN = 19;  // A5

static const uint8_t OT_VOLUME_CAPTURE_PIN = 2;
static const uint8_t OT_PITCH_CAPTURE_PIN = 8;

static const uint8_t OT_DAC_LDAC_PIN = 7;
static const uint8_t OT_DAC1_CS_PIN = 10;
static const uint8_t OT_DAC2_CS_PIN = 9;

#endif // _OT_PINS_H
