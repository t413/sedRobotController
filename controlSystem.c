//
//  controlSystem.c
//  sedRobotController
//
//  Created by Tim O'Brien on 5/7/12.
//  Copyright (c) 2012 t413.com. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include "controlSystem.h"
#include "pwm.h"
#include "analog.h"

#define MAX_POS 960
#define MIN_POS 100

void newPosition(uint16_t goal, uint16_t duration, ServoMotion * svo, uint32_t current_time) {
    
    //TODO: add velocity matching support..
    // needs the diritive of the position function, solve for t, set start time accordingly.
    goal = limit(goal, MIN_POS, MAX_POS); //limit incoming value.
    svo->final_pos = goal;
    svo->prev_pos = svo->pid_goal_pos;
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
		PORTA |=  (1 << ((val >= 0)? pin0 : (pin0+1)));
		PORTA &= ~(1 << ((val >= 0)? (pin0+1) : pin0));
	} else {
		PORTC |=  (1 << ((val >= 0)? pin0 : (pin0+1)));
		PORTC &= ~(1 << ((val >= 0)? (pin0+1) : pin0));
	}
}

void disableControlSystem(ServoMotion* svo, uint32_t current) {
	for (uint8_t m = 0; m < 6; m++) { 
		write_motor(m, 0); //disable motors
	}
}

void updateControlSystem(ServoMotion* svo, uint32_t current) {
	
	for (uint8_t m = 0; m < 6; m++) {
		//Trajectory control:
		float d = svo[m].move_duration; //duration of effect
		float t = limit( ((signed)current - (signed)svo[m].pos_rqst_time), 0, d); //current time
		int16_t b = (svo[m].prev_pos); //begining val
		int16_t c = ((signed)svo[m].final_pos - (signed)svo[m].prev_pos); //requested change in val
		
		//Trajectory path options
		TrajectoryPath path = CUBIC; 
		if (path == INSTANT) { 
			svo[m].pid_goal_pos = b + c;
		} else if (path == LINEAR) { 
			svo[m].pid_goal_pos = ((c*t)/(float)d + b);
		} else if (path == QUADRATIC) { 
			if ((t/=d/2) < 1) { svo[m].pid_goal_pos = c/2*t*t + b; } 
			else { svo[m].pid_goal_pos = -c/2 * ((--t)*(t-2) - 1) + b; }
		} else if (path == CUBIC) { 
			if ((t/=d/2) < 1) { svo[m].pid_goal_pos = c/2*t*t*t + b; } 
			else { svo[m].pid_goal_pos = c/2*((t-=2)*t*t + 2) + b; }
		}
		
		//PID control
		int16_t error = ( (signed)analogRead(m) - (signed)svo[m].pid_goal_pos );
		svo[m].i_error += error * svo[m].i;
		if (svo[m].i_error > 1000) { svo[m].i_error = 1000; }
		else if (svo[m].i_error < -1000) { svo[m].i_error = -1000; }
		write_motor(m, (error * svo[m].p) + (svo[m].i_error) );
		svo[m].last_val = error; //store last error
	}
}

int16_t limit (int16_t in_val, int16_t min, int16_t max) {
    if (in_val > max) return max;
    else if (in_val < min) return min;
    else return in_val;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



