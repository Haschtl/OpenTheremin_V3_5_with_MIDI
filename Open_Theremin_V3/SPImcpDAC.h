/* Control the mcp 4921/4922 DACs over Arduino SPI */

#ifndef SPImcpDac_h
#define SPImcpDac_h

#include <Arduino.h>
#include <SPI.h>

#include "pins.h"
#include "build.h"

#ifndef digitalWriteFast
#define digitalWriteFast digitalWrite
#endif

static inline void SPImcpDACinit()
{
  pinMode(OT_DAC_LDAC_PIN, OUTPUT);
  digitalWriteFast(OT_DAC_LDAC_PIN, HIGH);
  pinMode(OT_DAC1_CS_PIN, OUTPUT);
  digitalWriteFast(OT_DAC1_CS_PIN, HIGH);
  pinMode(OT_DAC2_CS_PIN, OUTPUT);
  digitalWriteFast(OT_DAC2_CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(OT_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
}

static inline void SPImcpDACtransmit(uint16_t data)
{
  SPI.transfer16(data);
}

static inline uint16_t SPImcpDACformatA(uint16_t data)
{
  return (uint16_t)((data & 0x0FFFU) | 0x7000U);
}

static inline uint16_t SPImcpDACformatB(uint16_t data)
{
  return (uint16_t)((data & 0x0FFFU) | 0xF000U);
}

static inline void SPImcpDAClatch()
{
  digitalWriteFast(OT_DAC_LDAC_PIN, LOW);
  digitalWriteFast(OT_DAC_LDAC_PIN, HIGH);
}

static inline void SPImcpDACsend(uint16_t data)
{
  digitalWriteFast(OT_DAC1_CS_PIN, LOW);
  SPImcpDACtransmit(SPImcpDACformatA(data));
  digitalWriteFast(OT_DAC1_CS_PIN, HIGH);
}

static inline void SPImcpDACsendPrepared(uint16_t dataWithConfig)
{
  digitalWriteFast(OT_DAC1_CS_PIN, LOW);
  SPImcpDACtransmit(dataWithConfig);
  digitalWriteFast(OT_DAC1_CS_PIN, HIGH);
}

static inline void SPImcpDAC2Asend(uint16_t data)
{
  digitalWriteFast(OT_DAC2_CS_PIN, LOW);
  SPImcpDACtransmit(SPImcpDACformatA(data));
  digitalWriteFast(OT_DAC2_CS_PIN, HIGH);
  SPImcpDAClatch();
}

static inline void SPImcpDAC2Bsend(uint16_t data)
{
  digitalWriteFast(OT_DAC2_CS_PIN, LOW);
  SPImcpDACtransmit(SPImcpDACformatB(data));
  digitalWriteFast(OT_DAC2_CS_PIN, HIGH);
  SPImcpDAClatch();
}

#endif
