#ifndef _PGMSPACE_COMPAT_H
#define _PGMSPACE_COMPAT_H

#include <stdint.h>

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_word_near
#define pgm_read_word_near(addr) (*(const int16_t *)(addr))
#endif

#endif // _PGMSPACE_COMPAT_H
