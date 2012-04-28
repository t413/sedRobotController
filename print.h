#ifndef print_h__
#define print_h__

#include <avr/pgmspace.h>
#include "usb_debug_only.h"

#ifdef PROGMEM 
#undef PROGMEM 
#define PROGMEM __attribute__((section(".progmem.data"))) 
#endif

// this macro allows you to write print("some text") and
// the string is automatically placed into flash memory :)
#define print(s) print_P(PSTR(s))
#define pchar(c) usb_debug_putchar(c)


#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


void print_P(const char *s);
void phex(unsigned char c);
void phex16(unsigned int i);
void printNumber(signed long n, uint8_t base);

#endif
