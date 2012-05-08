//
//  controlSystem.h
//  sedRobotController
//
//  Created by Tim O'Brien on 5/7/12.
//  Copyright (c) 2012 t413.com. All rights reserved.
//

#ifndef sedRobotController_controlSystem_h
#define sedRobotController_controlSystem_h

typedef struct  {
    int16_t pid_goal_pos; //actual current position for PID to achieve
    uint16_t prev_pos;     //position when new pos was requested
    uint16_t final_pos;    //request a position here, will be velocity controlled
    uint32_t pos_rqst_time;//time of requested position
    uint32_t move_duration;//time the requested transition takes to complete
    float p,i;             //PID gains
    float i_error;       //integral error
    int16_t last_val;      //for derivitive calculation
    int16_t max_p,min_p;
} ServoMotion;

typedef enum {
	INSTANT = 0,
	LINEAR = 1,
	QUADRATIC = 2,
	CUBIC = 3
} TrajectoryPath;

void updateControlSystem(ServoMotion* svo, uint32_t current);
void disableControlSystem(ServoMotion* svo, uint32_t current);

void newPosition(uint16_t goal, uint16_t duration, ServoMotion * svo, uint32_t current_time);

void write_motor(uint8_t motor, int16_t val);
int16_t limit(int16_t in, int16_t min, int16_t max);
long map(long x, long in_min, long in_max, long out_min, long out_max);


#endif
