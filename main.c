
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
#define FakePWM0(port,pin,val,period,incrcng_cntr) (port = ((incrcng_cntr %(period))>(val)) ? (port|(1<<(pin))) : (port&~(1<<(pin))))

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
    float p,i;             //PID gains
    int16_t i_error;       //integral error
    int16_t last_val;      //for derivitive calculation
};
void newPosition(uint16_t goal, uint16_t duration, struct servoMotion * svo, uint32_t current_time);

enum modes {
    POSITION = 0,
    ADJ_P, 
    ADJ_I,
};
#define NUM_MODES 3

int main(void) {

	// set for 16 MHz clock, and make sure the LED is off
	CPU_PRESCALE(0);
	LED_CONFIG;
	LED_OFF;	
	usb_init();
	pwm_init();
    timer0_init();
    init_spi();
    configure_receiver();
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
        svo[m].p = 20; svo[m].i = 2;
    } 

	uint32_t lastCntl = 0, lastPrint = 0, lastMove = 4000;
    enum modes mode = POSITION;
    uint8_t adj_servo = 0;
	while (1) {
        uint32_t t_tics = tics();
        uint32_t current = t_tics >> 11; //better than dividing by 2000 to get ms
        
        //periodic position requesting
        if ((current - lastMove) > 500) {
            for (uint8_t m = 0; m < 6; m++) { 
                if (m==0 && (mode == POSITION)) { continue; }
                if (current > (svo[m].pos_rqst_time + svo[m].move_duration + rand()%1000)) {
                    uint16_t dt = rand()%6000 + 1000 + 2*abs(svo[m].final_pos - svo[m].prev_pos);//transition duration
                    newPosition(rand()%850 + 100, dt, &(svo[m]), current);
                }
            }
            lastMove = current; 
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
                svo[m].i_error = limit( svo[m].i_error/svo[m].i + error, -200, 200);
                write_motor(m, (error * svo[m].p) + (svo[m].i_error) );
                svo[m].last_val = error; //store last error
            }
            lastCntl = current;
        }
        
        //debug-print control
        if ((current - lastPrint) > 50) { 
            printNumber(svo[0].pid_goal_pos,DEC); print(",");
            printNumber(svo[0].final_pos,DEC); print(",");
            for (uint8_t m = 0; m < 1; m++) {
                uint16_t a = analogRead(m);
                //if (a == 1023) continue;
                printNumber(a,DEC); print(",");
            }
            print("\n");
            
            lastPrint = current;
        }
        
        //fancy LED output
        {
        #define pwm_steps 20
        uint8_t led_clk_scl = 4-mode;
        if (((current >> led_clk_scl)/pwm_steps)%2) FakePWM0(PORTD,6,(((current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade in
        else FakePWM0(PORTD,6,pwm_steps-(((current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade out
        }
        
        //Nordic fob read
        if (!(PINE & (1<<6))) { //check Nordic module for data
            uint8_t stat = rx_send_byte(0xFF);
            if (stat & 0x40) { //We have data!  
                uint8_t reci[4];
                receive_data(reci);
                
                mode = (mode + (reci[0] == 0x0F)) % NUM_MODES; //center button
                if (reci[0] == 0x0F) { print("mode = "); printNumber( mode,DEC ); print("\n"); }
                
                if (mode == POSITION) {
                    int16_t dpos = (reci[0] == 0x1D)? 100:((reci[0] == 0x1E)? -200:0);
                    
                    newPosition(svo[0].final_pos + dpos, 1000, &(svo[0]), current);
                }
                else if (mode == ADJ_P) {
                    if      (reci[0] == 0x1D) svo[0].p *= 1.2f; //UP
                    else if (reci[0] == 0x1E) svo[0].p /= 1.2f; //DOWN
                    print(" P="); printNumber( svo[0].p ,DEC ); print("\n");
                }
                else if (mode == ADJ_I) {
                    if      (reci[0] == 0x1D) svo[0].i *= 1.2f; //UP
                    else if (reci[0] == 0x1E) svo[0].i /= 1.2f; //DOWN
                    print(" I="); printNumber( svo[0].i ,DEC ); print("\n");
                }
                
                switch(reci[0]) {
                    case 0x17: print("Left button"); break;
                    case 0x1E: print("Bottom button"); break;
                    case 0x1B: print("Right button"); break;
                    case 0x1D: print("Top button"); break;
                }
                print(" Presses="); printNumber( reci[2],DEC ); print("\n");
            }
        }

	}
}


void newPosition(uint16_t goal, uint16_t duration, struct servoMotion * svo, uint32_t current_time) {
    
    //TODO: add velocity matching support..
    // needs the diritive of the position function, solve for t, set start time accordingly.
    goal = limit(goal, 100, 960); //limit incoming value.
    svo->final_pos = goal;
    svo->prev_pos = svo[0].pid_goal_pos;
    svo->pos_rqst_time = current_time; //mark the current time
    svo->move_duration = duration;
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


    


