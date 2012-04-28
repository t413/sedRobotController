
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pwm.h"
#include "usb_debug_only.h"
#include "print.h"
#include "analog.h"
#include "interrupt_timer.h"
#include "spi.h"
#include "nrf24l01.h"

#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_TOGGLE	(PORTD ^= (1<<6))

#define LED_CONFIG	(DDRD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
void write_motor(uint8_t motor, int16_t val);
int16_t limit(int16_t in, int16_t min, int16_t max);

struct servoMotion {
    uint16_t pid_goal_pos; //actual current position for PID to achieve
    uint16_t prev_pos;     //position when new pos was requested
    uint16_t final_pos;    //request a position here, will be velocity controlled
    uint32_t pos_rqst_time;//time of requested position
    uint32_t move_duration;//time the requested transition takes to complete
    int16_t i_error;       //integral error
    int16_t last_val;      //for derivitive calculation
};

int main(void) {

	// set for 16 MHz clock, and make sure the LED is off
	CPU_PRESCALE(0);
	LED_CONFIG;
	LED_OFF;	
	usb_init();
	pwm_init();
    timer0_init();
    init_spi();
    analogReference(ADC_REF_EXTERNAL);
    
    //for H-Bridge control lines: 
	DDRA = 0xFF;
    DDRC |= 0x0F; //pins 0-3;
	PORTF = 0xFF;
    
	_delay_ms(100);

	write_pwms(0,0,0,0,0,0);
	print("Yo, 'sup world?\n");
    struct servoMotion svo[6];
    for (uint8_t m = 0; m < 6; m++) {  //loop through each svo instance and initialize it.
        svo[m].pid_goal_pos = svo[m].final_pos = svo[m].prev_pos = analogRead(m);
        svo[m].i_error = svo[m].pos_rqst_time = svo[m].last_val = svo[m].move_duration =  0;
    } 

	uint32_t lastCntl = 0, lastLED = 0, lastMove = 4000;
    uint8_t lastMoveDir = 0;
	while (1) {
        uint32_t current = millis();
        
        //periodic position requesting
        if ((current - lastMove) > 500) {
            for (uint8_t m = 0; m < 6; m++) { 
                if (current > (svo[m].pos_rqst_time + svo[m].move_duration + rand()%1000)) {
                    svo[m].final_pos = rand()%850 + 100; //request a new position
                    svo[m].prev_pos = svo[m].pid_goal_pos; //mark our current position
                    svo[m].pos_rqst_time = current; //mark the current time
                    svo[m].move_duration = rand()%6000 + 1000 + 2*abs(svo[m].final_pos - svo[m].prev_pos); //randomly set transition duration
                }
            }
            lastMove = current; 
            lastMoveDir = !lastMoveDir;
        }
        
        //control system
        if ((current - lastCntl) > 10) {
            
            //trajectory controll
            for (uint8_t m = 0; m < 6; m++) {
                float d = svo[m].move_duration; //duration of effect
                float t = limit( ((signed)current - (signed)svo[m].pos_rqst_time), 0, d); //current time
                int16_t b = (svo[m].prev_pos); //begining val
                int16_t c = ((signed)svo[m].final_pos - (signed)svo[m].prev_pos); //requested change in val
                //--PATH OPTIONS--
                /*
                 //linear path:
                svo[m].pid_goal_pos = ((c*t)/(float)d + b);
                //quadratic easing:
                if ((t/=d/2) < 1) { svo[m].pid_goal_pos = c/2*t*t + b; } 
                else { svo[m].pid_goal_pos = -c/2 * ((--t)*(t-2) - 1) + b; }
                 */
                //cubic easing
                if ((t/=d/2) < 1) { svo[m].pid_goal_pos = c/2*t*t*t + b; } 
                else { svo[m].pid_goal_pos = c/2*((t-=2)*t*t + 2) + b; }
            }
            
            //PID control
            for (uint8_t m = 0; m < 6; m++) {
                int16_t error = ( analogRead(m) - (signed)svo[m].pid_goal_pos );
                svo[m].i_error = limit( svo[m].i_error/2 + error, -200, 200);
                write_motor(m, (error * 20) + (svo[m].i_error) );
                svo[m].last_val = error; //store last error
            }
            lastCntl = current;
        }
        
        //led / debug-print control
        if ((current - lastLED) > 50) { 
            LED_TOGGLE; 
            //printNumber((current),DEC);print(",");
            printNumber(svo[0].pid_goal_pos,DEC); print(",");
            printNumber(svo[0].final_pos,DEC); print(",");
            for (uint8_t m = 0; m < 1; m++) {
                uint16_t a = analogRead(m);
                //if (a == 1023) continue;
                printNumber(a,DEC); print(",");
            }
            print("\n");
            lastLED = current;
        }

	}
}



void write_motor(uint8_t motor, int16_t val) {
    uint8_t pin0 = (motor < 4)? (motor * 2) : ((motor-4)*2); //For PORTA vs PORTC
    
    //volatile uint8_t* hport = (motor < 4)? &PORTA : &PORTC;
    //*hport |=  (1 << (val > 0)? pin0 : (pin0+1));
    //*hport &= ~(1 << (val < 0)? pin0 : (pin0+1));
    write_pwm(motor, (val > 0)? val : ((signed)-val)); //write positive value to pwm EN pin
    if (motor == 5 || motor == 3) { val = -val; }//reverse these chanles
    if (motor < 4){
        PORTA |=  (1 << ((val > 0)? pin0 : (pin0+1)));
        PORTA &= ~(1 << ((val > 0)? (pin0+1) : pin0));
    } else {
        PORTC |=  (1 << ((val > 0)? pin0 : (pin0+1)));
        PORTC &= ~(1 << ((val > 0)? (pin0+1) : pin0));
    }
}


int16_t limit (int16_t in_val, int16_t min, int16_t max) {
    if (in_val > max) return max;
    else if (in_val < min) return min;
    else return in_val;
}


    


