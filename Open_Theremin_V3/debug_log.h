#ifndef _OT_DEBUG_LOG_H
#define _OT_DEBUG_LOG_H

#include <Arduino.h>

#if defined(DEBUG)
#define OT_DEBUG_ENABLED 1
#else
#define OT_DEBUG_ENABLED 0
#endif

#ifndef OT_DEBUG_BAUD
#define OT_DEBUG_BAUD 115200
#endif

#ifndef OT_DEBUG_WAIT_FOR_HOST_MS
#define OT_DEBUG_WAIT_FOR_HOST_MS 2000
#endif

#ifndef OT_DEBUG_RUNTIME_LOGS
#define OT_DEBUG_RUNTIME_LOGS 0
#endif

// Default to USB serial so debug output is visible in PlatformIO monitor.
#ifndef OT_DEBUG_PORT
#define OT_DEBUG_PORT Serial
#endif

#if OT_DEBUG_ENABLED
#define OT_DEBUG_BEGIN()           do { \
  OT_DEBUG_PORT.begin(OT_DEBUG_BAUD); \
  const uint32_t _dbg_t0 = millis(); \
  while (!OT_DEBUG_PORT && ((uint32_t)(millis() - _dbg_t0) < (uint32_t)OT_DEBUG_WAIT_FOR_HOST_MS)) { delay(10); } \
  OT_DEBUG_PORT.println(F("[DBG] serial ready")); \
} while (0)
#define OT_DEBUG_PRINT(x)          do { OT_DEBUG_PORT.print(x); } while (0)
#define OT_DEBUG_PRINTLN(x)        do { OT_DEBUG_PORT.println(x); } while (0)
#define OT_DEBUG_PRINTLN_F(x)      do { OT_DEBUG_PORT.println(F(x)); } while (0)
#else
#define OT_DEBUG_BEGIN()           do { } while (0)
#define OT_DEBUG_PRINT(x)          do { (void)sizeof(x); } while (0)
#define OT_DEBUG_PRINTLN(x)        do { (void)sizeof(x); } while (0)
#define OT_DEBUG_PRINTLN_F(x)      do { } while (0)
#endif

#endif // _OT_DEBUG_LOG_H
