
// SPI
// by Tim O'Brien (t413.com), Sept. 16, 2010.


#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi.h"


void init_spi(void) {
    DDRB |= (1<<0) | (1<<1) | (1<<2) | (1<<3); //set (CS, SCLK, MOSI, MISO) as outputs
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0); //enable SPI as Master w/ clock fck/128  
}

unsigned char exchangeSPI(unsigned char byte) {
    SPDR = byte;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

