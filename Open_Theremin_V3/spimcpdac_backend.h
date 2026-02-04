#ifndef _SPIMCPDAC_BACKEND_H
#define _SPIMCPDAC_BACKEND_H

#include <Arduino.h>

bool otSpiDmaInit(uint32_t clockHz);
bool otSpiDmaTransfer16(uint16_t data);

#endif // _SPIMCPDAC_BACKEND_H
