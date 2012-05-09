//
//  sedRobotAPI.h
//  sedRobotController
//
//  Created by Tim O'Brien on 5/7/12.
//  Copyright (c) 2012 t413.com. All rights reserved.
//

#ifndef sedRobotController_sedRobotAPI_h
#define sedRobotController_sedRobotAPI_h
#include <inttypes.h>


/*  Packets look like this:
 *  
 *  /------------------------Header--------------------------/ /--data--/ /-------checksum-------/
 *  | Start byte0 | Start1 | packet ID | packet Type | length | **data** | Checksum0 | Checksum1 |
 *  
 *  the data section is variable length is specified (in __ bytes long).
 *  all bytes are sent as uint8_t (unsigned 8 bit ints)
 */

/* sed_base_pkt_t
 * base packet struct equivalent. For easy processing.
 */
typedef struct {
	uint8_t start0,start1;
	uint8_t pktID,pktType;
	uint8_t length;
	uint8_t data_start;
} __attribute__ ((__packed__)) Sed_base_pkt_t;


/* --------------------------------- */
/* --- Data layer Packet layouts --- */
/* --------------------------------- */
typedef enum {
	SED_ENABLE_DRIVE	= 0, //command only, length=0
	SED_ENABLE_MASKED	= 1, //len=1, 1=enabled, bits 0-5 for motors 0-5
	SED_DISABLE_DRIVE	= 2, //command only
	SED_MOVE_ANGLES		= 3, //see struct below
	SED_SET_CONF_MIN	= 4, //command only
	SED_SET_CONF_MAX	= 5, //command only
} Sed_command_type;

// for SED_MOVE_ANGLES
typedef struct {
	uint8_t which_en;  //mask for which channels to use
	int16_t angles[6]; 
	int16_t duration;
	uint8_t traject_path;
} __attribute__ ((__packed__)) Sed_move_angles_t;



/* ------------------------*/
/* -- sed API functions -- */
/* ------------------------*/
void sedAPI_set_send_fn(void (*sendFn)(uint8_t));
void sedAPI_send_angles(uint8_t pktID, Sed_move_angles_t* datas);
void sedAPI_send_pkt(uint8_t pktID, uint8_t which, uint8_t* data, uint8_t length);
void sedAPI_make_pkt(uint8_t* packet, uint8_t pktID, uint8_t pktType, uint8_t* data, uint8_t length);

uint8_t sedAPI_is_pkt(const uint8_t * data);

#endif
