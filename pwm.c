//
//  pwm.c
//  teensy_copter
//
//  Created by Tim O'Brien on 4/13/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#include <avr/io.h>
#include <inttypes.h>
#include "pwm.h"


//TODO: make variable argument init that inits spesific channels.

void pwm_init(void) { 

    /* ------------------------------------------------------- */
    /* ---- set up timer 1 to enable channels A, B, and C ---- */
    
    //Setup OC1A to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0);
    
    //Setup OC1B to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1B1);
    TCCR1A &= ~(1 << COM1B0);
    
    //Setup OC1C to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1C1);
    TCCR1A &= ~(1 << COM1C0);
    
    // !(WGM10) WGM11, WGM12 and WGM13 set for FAST PWM output (mode 14 in datasheet) so ICR1 is top 
    TCCR1A |= (1 << WGM11); 
    TCCR1A &= ~(1 << WGM10); 
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    
    //Timing:
    // Using ICR1 as TOP For 50Hz Output at max resolution set at 40,000 ... 16,000,000 / 8 * 40001 = ~50hz 
    //(F_CPU / (CLKPR+1))/(50*8);  // should equal 40000 when at 16Mhz.

    //TCCR1B |= (1 << CS11); // <--- Prescaling by 8 (page 138)
    TCCR1B &= ~((1 << CS11) | (1 << CS12)); // <-- prescaling by 2 I think?
    TCCR1B |= (1 << CS10); // (fast mode)

    ICR1 = 1000; //counter counts to this value before restarting. Called TOP.

    
    
    /* -------------------------------------------- */
    /* ---- set up timer 3 to enable channel A ---- */
    
    //Setup OC3A to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR3A |= (1 << COM3A1); 
    TCCR3A &= ~(1 << COM3A0); 
    
    //Setup OC3B to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR3A |= (1 << COM3B1);
    TCCR3A &= ~(1 << COM3B0);
    
    //Setup OC3C to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR3A |= (1 << COM3C1);
    TCCR3A &= ~(1 << COM3C0);
     
    TCCR3A |= (1 << WGM31); 
    TCCR3A &= ~(1 << WGM30); 
    TCCR3B |= (1 << WGM32) | (1 << WGM33);
    
    //Timing:
    //TCCR3B |= (1 << CS11); // <--- Prescaling by 8 (page 138)
    TCCR3B &= ~((1 << CS11) | (1 << CS12)); // (fast mode)
    TCCR3B |= (1 << CS10); // (fast mode)
    ICR3 = 1000; //counter counts to this value before restarting. Called TOP.
    
    
    
    /* ----------------------- */
    /*-- set pins to outputs --*/
    DDRB |= (1 << 5); // ->  OC1A
    DDRB |= (1 << 6); // ->  OC1B
    DDRB |= (1 << 7); // ->  OC1C
    DDRC |= (1 << 6); // ->  OC3A
    DDRC |= (1 << 5); // ->  OC3B
    DDRC |= (1 << 4); // ->  OC3C
    // Set OCRA1 to something - Servo is based on 1 - 2ms pulse if 40000 = 20ms pulse whats between 1 and 2 (2000 TO 4000) 
    
}


#define map(x,in_min,in_max,out_min,out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))


void write_pwm(uint8_t which, uint16_t in_val){
    switch (which){
        case 0 : OCR1B = in_val; break; 
        case 1 : OCR1A = in_val; break; 
        case 2 : OCR1C = in_val; break; 
        case 3 : OCR3A = in_val; break; 
        case 4 : OCR3B = in_val; break; 
        case 5 : OCR3C = in_val; break; 
    }
}

void write_pwms(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4, uint16_t m5) {
	OCR1B = m0;  //0 -> b6 -> oc1b
	OCR1A = m1;  //1 -> b5 -> oc1a
	OCR1C = m2;  //2 -> b7 -> oc1c
	OCR3A = m3;  //3 -> c6 -> oc3a
	OCR3B = m4;  //4 -> c5 -> oc3b
	OCR3C = m5;  //5 -> c4 -> oc3c
}





