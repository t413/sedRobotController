
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pwm.h"
#include "usb_rawhid.h"
#include "sedRobotAPI.h"
#include "analog.h"
#include "interrupt_timer.h"
#include "spi.h"
#include "nrf24l01.h"
#include "printf.h"
#include "controlSystem.h"

#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_TOGGLE	(PORTD ^= (1<<6))
#define LED_CONFIG	(DDRD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

void fakePWM(volatile uint8_t* port, const uint8_t pin_mask, const uint16_t val, const uint16_t period, const uint32_t incrcng_cntr);
void fadingLED( const uint32_t current_time, const uint8_t clock_scale, const uint8_t pwm_steps, volatile uint8_t* port, const const uint8_t pin_mask, const uint32_t incrcng_cntr);

#define old_print(x) 
#define printNumber(x,y) 


typedef enum {
    MEAS_RANGE = 0,
    POSITION,
    ADJ_P, 
    ADJ_I,
} Mode;
#define NUM_MODES 4

void serialCommandRecieve(char* buf, ServoMotion* svo, uint32_t current, Mode* mode);

char buffer[64]; //for the USB HID packet buffer

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
    
	_delay_ms(1000);

	stringf(buffer,"Hello Tim\n"); usb_rawhid_send((uint8_t*)buffer, 50);
	
	write_pwms(0,0,0,0,0,0);
    ServoMotion svo[6];
    for (uint8_t m = 0; m < 6; m++) {  //loop through each svo instance and initialize it.
        svo[m].pid_goal_pos = svo[m].final_pos = svo[m].prev_pos = svo[m].max_p = svo[m].min_p = analogRead(m);
        svo[m].i_error = svo[m].pos_rqst_time = svo[m].last_val = svo[m].move_duration =  0;
        svo[m].p = 13; svo[m].i = 0.8;
    } 

	uint32_t lastCntl = 0, lastPrint = 0;
    Mode mode = MEAS_RANGE;
	while (1) {
        uint32_t t_tics = tics();
        uint32_t current = t_tics >> 11; //better than dividing by 2000 to get ms
        
        //debug-print control
        if (usb_configured()) {
            if ((current - lastPrint) > 100) { 
				buffer[0] = 0xFF;
				usb_rawhid_recv((uint8_t*)buffer,1);
				if (buffer[0] != 0xFF) { serialCommandRecieve(buffer,svo,current,&mode); }
                for (uint8_t m = 0; m < 6; m++) {
                    uint16_t val = analogRead(m);
                    if (val == 1023) continue;
                    stringf(buffer,"svo[%d] <%d,%d,%d> [%d-%d] f%f\n",
                            m,svo[m].pid_goal_pos,svo[m].final_pos, val,svo[m].min_p,svo[m].max_p,svo[m].i_error);
                    usb_rawhid_send((uint8_t*)buffer, 5);
                }
                lastPrint = current;
            }
        }
        
        //periodic position requesting
        /*if ((current - lastMove) > 500) {
            for (uint8_t m = 0; m < 6; m++) { 
                //if (m==0 && (mode == POSITION)) { continue; }
                if (current > (svo[m].pos_rqst_time + svo[m].move_duration + rand()%1000)) {
                    uint16_t dt = rand()%6000 + 1000 + 2*abs(svo[m].final_pos - svo[m].prev_pos);//transition duration
                    newPosition(rand()%850 + 100, dt, &(svo[m]), current);
                }
            }
            lastMove = current; 
        }*/
        
        //control system
        if ((current - lastCntl) > 10) {
            if (mode == MEAS_RANGE) { disableControlSystem(svo,current); } 
            else { updateControlSystem(svo, current); }
            lastCntl = current;
        }
        
        /*if (mode == MEAS_RANGE) {
            for (uint8_t m = 0; m < 6; m++) {
                uint16_t val = analogRead(m);
                if (val > svo[m].max_p) { svo[m].max_p = val; }
                else if (val < svo[m].min_p) { svo[m].min_p = val; }
                newPosition( val, 1000, &(svo[m]), current);
            }
        }*/
        
        
        //fancy LED output
        fadingLED( current, (4-mode), (20), &PORTD, (1<<6), t_tics);
                
        //Nordic fob read
        if (!(PINE & (1<<6))) { //check Nordic module for data
            uint8_t stat = rx_send_byte(0xFF);
            if (stat & 0x40) { //We have data!  
                uint8_t reci[4];
                receive_data(reci);
                
                mode = (mode + (reci[0] == 0x0F)) % NUM_MODES; //center button
                if (reci[0] == 0x0F) { 
                    stringf(buffer,"mode = %d\n",mode); usb_rawhid_send((uint8_t*)buffer, 50);
                }
                
                if (mode == POSITION) {
                    int16_t dpos = (reci[0] == 0x1D)? 100:((reci[0] == 0x1E)? (-100) : 0); //up = 100, down = -100
                    for (uint8_t m = 0; m < 6; m++) {
                        uint16_t npos = limit( svo[m].final_pos + dpos, svo[m].min_p, svo[m].max_p);
                        
                        stringf(buffer,"npos = %d, [%d %d]\n",npos,svo[m].min_p,svo[m].max_p); 
                        usb_rawhid_send((uint8_t*)buffer, 50);
                        
                        newPosition( npos, 1500, &(svo[m]), current);
                    }
                }
                else if (mode == ADJ_P) {
                    if      (reci[0] == 0x1D) for (uint8_t m = 0; m < 6; m++) { svo[0].p *= 1.2f; } //UP
                    else if (reci[0] == 0x1E) for (uint8_t m = 0; m < 6; m++) { svo[0].p /= 1.2f; }; //DOWN
                    stringf(buffer," P = %f\n",svo[0].p); usb_rawhid_send((uint8_t*)buffer, 50);
                }
                else if (mode == ADJ_I) {
                    if      (reci[0] == 0x1D) for (uint8_t m = 0; m < 6; m++) { svo[0].i *= 1.2f; } //UP
                    else if (reci[0] == 0x1E) for (uint8_t m = 0; m < 6; m++) { svo[0].i /= 1.2f; } //DOWN
                    stringf(buffer," I = %f\n",svo[0].i); usb_rawhid_send((uint8_t*)buffer, 50);
                }
                
                char press = ' ';
                switch(reci[0]) {
                    case 0x17: press = '<'; break; //left
                    case 0x1E: press = '_'; break; //bottom
                    case 0x1B: press = '>'; break; //right
                    case 0x1D: press = '^'; break; //top
                }
                stringf(buffer,"%c Button Pressed (%d presses)\n",press, reci[2]); usb_rawhid_send((uint8_t*)buffer, 50);
            }
        }

	}
}



void fakePWM(volatile uint8_t* port, const uint8_t pin_mask, const uint16_t val, const uint16_t period, const uint32_t incrcng_cntr) {
    if ((incrcng_cntr %(period))>(val)) {
        *port |= pin_mask;
    } else {
        *port &= ~pin_mask;
    }
}


void fadingLED(
               const uint32_t current_time, //time in milliseconds
               const uint8_t clock_scale,   //scale down factor, larger == slower fade
               const uint8_t pwm_steps,     //more steps = nicer fade IF called often
               volatile uint8_t* port,//something like (&PORTA)
               const const uint8_t pin_mask,//which pins to use
               const uint32_t incrcng_cntr  //time, faster than ms. I use clock tick counter.
               ) {
    
    uint32_t scaled_time = (current_time >> clock_scale);
    
    //Fade in:
    if ((scaled_time/pwm_steps)%2) { 
        fakePWM(port,pin_mask,((scaled_time)%pwm_steps),pwm_steps,incrcng_cntr);
    }
    //Fade out:
    else {
        fakePWM(port,pin_mask,pwm_steps-((scaled_time)%pwm_steps),pwm_steps,incrcng_cntr); //fade out
    }
}


void serialCommandRecieve(char* buf, ServoMotion* svo, uint32_t current, Mode* mode) {
	if (sedAPI_is_pkt((uint8_t*)buf)) { //packet passes checksum, etc
		Sed_base_pkt_t* pkt = (Sed_base_pkt_t*) buffer;
		
		//process different packet types:
		if (pkt->pktType == SED_MOVE_ANGLES){
			Sed_move_angles_t* rec = (Sed_move_angles_t*) &pkt->data_start;
			for (uint8_t m=0; m<6; m++) {
				if ((rec->which_en >> m) & 0x01) { //this chanel is enabled by the mask
					newPosition(rec->angles[m], rec->duration, &(svo[m]), current);
				}
			}
		}
		else if (pkt->pktType == SED_ENABLE_DRIVE){ *mode = POSITION; }
		else if (pkt->pktType == SED_DISABLE_DRIVE){ *mode = MEAS_RANGE; }
		else if (pkt->pktType == SED_SET_CONF_MIN){ 
			for (uint8_t m = 0; m < 6; m++) { svo[m].min_p = analogRead(m); }
		} else if (pkt->pktType == SED_SET_CONF_MAX){ 
			for (uint8_t m = 0; m < 6; m++) { svo[m].max_p = analogRead(m); }
		}

	}
	
	/*if (!strcmp(buf,"mode")) {
		(*mode) = (*mode + 1) % NUM_MODES;
	} else if (!strcmp(buf,"help")) {
		stringf(buffer,"Hello Tim\n"); usb_rawhid_send((uint8_t*)buffer, 50);
	}*/
}

    


