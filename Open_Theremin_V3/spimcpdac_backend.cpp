#include "spimcpdac_backend.h"

#include "build.h"
#include "error_indicator.h"

#if OT_USE_DMA

#include <hal_data.h>
#include "r_spi.h"

static volatile bool g_spiTransferDone = false;
static volatile bool g_spiTransferError = false;
static volatile bool g_spiTransferBusy = false;
static volatile uint16_t g_spiGoodFramesAfterError = 0;

static spi_cfg_t g_spiCfg;
static spi_extended_cfg_t g_spiExtCfg;
static bool g_spiInitDone = false;

static void otSpiCallback(spi_callback_args_t *p_args) {
  if (!p_args) {
    return;
  }

  if (p_args->event == SPI_EVENT_TRANSFER_COMPLETE) {
    g_spiTransferDone = true;
  } else {
    g_spiTransferError = true;
    g_spiTransferDone = true;
  }
}

bool otSpiDmaInit(uint32_t clockHz) {
  if (g_spiInitDone) {
    return true;
  }

  g_spiCfg = g_spi0_cfg;
  g_spiExtCfg = *reinterpret_cast<spi_extended_cfg_t const *>(g_spi0_cfg.p_extend);

  if (FSP_SUCCESS != R_SPI_CalculateBitrate(clockHz, &g_spiExtCfg.spck_div)) {
    return false;
  }

  g_spiCfg.p_extend = &g_spiExtCfg;
  g_spiCfg.p_callback = otSpiCallback;
  g_spiCfg.p_context = nullptr;

  if (!g_spiCfg.p_transfer_tx || !g_spiCfg.p_transfer_rx) {
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }

  if (FSP_SUCCESS != g_spiCfg.p_transfer_tx->p_api->open(g_spiCfg.p_transfer_tx->p_ctrl, g_spiCfg.p_transfer_tx->p_cfg)) {
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }
  if (FSP_SUCCESS != g_spiCfg.p_transfer_rx->p_api->open(g_spiCfg.p_transfer_rx->p_ctrl, g_spiCfg.p_transfer_rx->p_cfg)) {
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }

  g_spi0.p_api->close(&g_spi0_ctrl);

  fsp_err_t err = g_spi0.p_api->open(&g_spi0_ctrl, &g_spiCfg);
  if (err != FSP_SUCCESS) {
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }

  g_spiInitDone = true;
  return true;
}

bool otSpiDmaTransfer16(uint16_t data) {
  if (!g_spiInitDone || g_spiTransferBusy) {
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }

  uint16_t rxDummy = 0;

  g_spiTransferBusy = true;
  g_spiTransferDone = false;
  g_spiTransferError = false;

  fsp_err_t err = g_spi0.p_api->writeRead(&g_spi0_ctrl, &data, &rxDummy, 1, SPI_BIT_WIDTH_16_BITS);
  if (err != FSP_SUCCESS) {
    g_spiTransferBusy = false;
    setErrorIndicator(OT_ERR_DMA);
    return false;
  }

  const uint32_t startUs = micros();
  while (!g_spiTransferDone) {
    if ((uint32_t)(micros() - startUs) > OT_DMA_TRANSFER_TIMEOUT_US) {
      g_spiTransferBusy = false;
      setErrorIndicator(OT_ERR_DMA);
      return false;
    }
    __asm__ __volatile__("nop");
  }

  g_spiTransferBusy = false;
  if (g_spiTransferError) {
    setErrorIndicator(OT_ERR_DMA);
    g_spiGoodFramesAfterError = 0;
    return false;
  }
  if (getErrorIndicator() == OT_ERR_DMA) {
    if (g_spiGoodFramesAfterError < 2000U) {
      g_spiGoodFramesAfterError++;
    } else {
      clearErrorIndicator();
    }
  }
  return true;
}

#else

bool otSpiDmaInit(uint32_t clockHz) {
  (void)clockHz;
  return false;
}

bool otSpiDmaTransfer16(uint16_t data) {
  (void)data;
  return false;
}

#endif
