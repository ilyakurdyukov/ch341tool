#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define PROGMEM

#ifdef __cplusplus
extern "C"{
#endif

typedef bool boolean;
typedef uint8_t byte;

static inline byte pgm_read_byte(const byte *data) { return *data; }

void Arduino_yield(void);
void Arduino_delay(unsigned long ms);
unsigned long Arduino_millis(void);

static inline void yield(void) { Arduino_yield(); }
static inline void delay(unsigned long ms) { Arduino_delay(ms); }
static inline unsigned long millis(void) { return Arduino_millis(); }

typedef const char __FlashStringHelper;
#define F(str) str

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
#include "Print.h"
#endif

#endif
