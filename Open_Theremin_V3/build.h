// Build definitions

#ifndef _BUILD_H
#define _BUILD_H



// Set to build with control voltage output (experimental)
#define CV_ENABLED 0

// DAC SPI clock (MCP49xx max is typically 20MHz, keep board/wiring quality in mind).
#define OT_SPI_CLOCK_HZ 20000000

// Audio ISR tick rate in Hz (31.25kHz keeps legacy behavior).
// Original: 31250
#define OT_AUDIO_TICK_HZ 48000 

// Median filtering for analog pots (odd number, currently supported: 3).
#define OT_POT_MEDIAN_SAMPLES 3

#endif // _BUILD_H
