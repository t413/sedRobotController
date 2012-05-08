#ifndef small_printf_h__
#define small_printf_h__

//#include <avr/pgmspace.h>
//#include "usb_rawhid.h"

/*
#ifdef PROGMEM 
#undef PROGMEM 
#define PROGMEM __attribute__((section(".progmem.data"))) 
#endif

// this macro allows you to write print("some text") and
// the string is automatically placed into flash memory :)
#define print(s) print_P(PSTR(s))
#define pchar(c) usb_debug_putchar(c) 
*/


int stringf (char *out, const char *format, ...);

#endif
