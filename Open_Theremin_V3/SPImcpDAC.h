/* Control the mcp 4921/4922 DACs over Arduino SPI */

#ifndef SPImcpDac_h
#define SPImcpDac_h

#include <Arduino.h>
#include "build.h"
#include "error_indicator.h"

#if OT_USE_DMA
#include "spimcpdac_backend.h"
#else
#include <SPI.h>
#endif

#include "pins.h"

#ifndef digitalWriteFast
#define digitalWriteFast digitalWrite
#endif

static inline uint32_t otGetPrimask()
{
#if defined(__ARM_ARCH)
  uint32_t value;
  __asm__ __volatile__("MRS %0, primask" : "=r"(value));
  return value;
#else
  return 0U;
#endif
}

static inline void SPImcpDACinit()
{
  pinMode(OT_DAC_LDAC_PIN, OUTPUT);
  digitalWriteFast(OT_DAC_LDAC_PIN, HIGH);
  pinMode(OT_DAC1_CS_PIN, OUTPUT);
  digitalWriteFast(OT_DAC1_CS_PIN, HIGH);
  pinMode(OT_DAC2_CS_PIN, OUTPUT);
  digitalWriteFast(OT_DAC2_CS_PIN, HIGH);

#if OT_USE_DMA
  if (!otSpiDmaInit(OT_SPI_CLOCK_HZ)) {
#if OT_DMA_HARDFAIL
    fatalErrorLoop(OT_ERR_DMA);
#else
    setErrorIndicator(OT_ERR_DMA);
    return;
#endif
  }
#else
  SPI.begin();
  SPI.beginTransaction(SPISettings(OT_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
#endif
}

static inline void SPImcpDACtransmit(uint16_t data)
{
#if OT_USE_DMA
  const bool irqWasDisabled = (otGetPrimask() != 0U);
  if (irqWasDisabled) {
    interrupts();
  }
  if (!otSpiDmaTransfer16(data)) {
#if OT_DMA_HARDFAIL
    fatalErrorLoop(OT_ERR_DMA);
#else
    setErrorIndicator(OT_ERR_DMA);
    if (irqWasDisabled) {
      noInterrupts();
    }
    return;
#endif
  }
  if (irqWasDisabled) {
    noInterrupts();
  }
#else
  SPI.transfer16(data);
#endif
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
