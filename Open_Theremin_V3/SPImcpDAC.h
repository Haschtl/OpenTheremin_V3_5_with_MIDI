/* Control the mcp 4921/4922 DACs over Arduino SPI */

#ifndef SPImcpDac_h
#define SPImcpDac_h

#include <Arduino.h>
#include <SPI.h>

#include "pins.h"

static inline void SPImcpDACinit()
{
  pinMode(OT_DAC_LDAC_PIN, OUTPUT);
  digitalWrite(OT_DAC_LDAC_PIN, HIGH);
  pinMode(OT_DAC1_CS_PIN, OUTPUT);
  digitalWrite(OT_DAC1_CS_PIN, HIGH);
  pinMode(OT_DAC2_CS_PIN, OUTPUT);
  digitalWrite(OT_DAC2_CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
}

static inline void SPImcpDACtransmit(uint16_t data)
{
  SPI.transfer16(data);
}

static inline void SPImcpDAClatch()
{
  digitalWrite(OT_DAC_LDAC_PIN, LOW);
  digitalWrite(OT_DAC_LDAC_PIN, HIGH);
}

static inline void SPImcpDACsend(uint16_t data)
{
  digitalWrite(OT_DAC1_CS_PIN, LOW);
  data &= 0x0FFF;
  data |= 0x7000;
  SPImcpDACtransmit(data);
  digitalWrite(OT_DAC1_CS_PIN, HIGH);
}

static inline void SPImcpDAC2Asend(uint16_t data)
{
  digitalWrite(OT_DAC2_CS_PIN, LOW);
  data &= 0x0FFF;
  data |= 0x7000;
  SPImcpDACtransmit(data);
  digitalWrite(OT_DAC2_CS_PIN, HIGH);
  SPImcpDAClatch();
}

static inline void SPImcpDAC2Bsend(uint16_t data)
{
  digitalWrite(OT_DAC2_CS_PIN, LOW);
  data &= 0x0FFF;
  data |= 0xF000;
  SPImcpDACtransmit(data);
  digitalWrite(OT_DAC2_CS_PIN, HIGH);
  SPImcpDAClatch();
}

#endif
