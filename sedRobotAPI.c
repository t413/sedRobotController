//
//  sedRobotAPI.c
//  sedRobotController
//
//  Created by Tim O'Brien on 5/7/12.
//  Copyright (c) 2012 t413.com. All rights reserved.
//

#include <stdio.h>
#include "sedRobotAPI.h"
#define START0 0x93
#define START1 0xE0

/* -----------------------------------*/
/* -- private variables/prototypes -- */
/* -----------------------------------*/
void (*serialSend)(uint8_t);
void sedAPI_checksum( const uint8_t pktID, const uint8_t pktType, const uint8_t length, const uint8_t *buf,
					 uint8_t *c0, uint8_t *c1 );

/* ------------------------*/
/* --  setup functions  -- */
/* ------------------------*/
void sedAPI_set_send_fn(void (*sendFn)(uint8_t)) {
	serialSend = sendFn;
}



/* ------------------------*/
/* -- sending functions -- */
/* ------------------------*/

void sedAPI_send_angles(uint8_t pktID, Sed_move_angles_t* datas) {
	uint8_t* data = (uint8_t*)datas;
	
	sedAPI_send_pkt( pktID, SED_MOVE_ANGLES, data, sizeof(Sed_move_angles_t));
}


void sedAPI_send_pkt(uint8_t pktID, uint8_t pktType, uint8_t* data, uint8_t length) {
    (*serialSend)( START0 ); //send header start
    (*serialSend)( START1 ); //send header start
    (*serialSend)( pktID );
    (*serialSend)( pktType );
    (*serialSend)( length );
	
    uint8_t cksum0, cksum1;
	sedAPI_checksum(pktID, pktType, length, data, &cksum0, &cksum1);
	
    for (uint8_t i = 0; i < length; i++) {
        (*serialSend)( data[i] );
    }
    (*serialSend)( cksum0 ); //finally, send the checksum!
    (*serialSend)( cksum1 ); //finally, send the checksum!
}

void sedAPI_make_pkt(uint8_t* packet, uint8_t pktID, uint8_t pktType, uint8_t* data, uint8_t length) {
    *(packet++) = START0; //send header start
    *(packet++) = START1; //send header start
    *(packet++) = pktID;
    *(packet++) = pktType;
    *(packet++) = length;
	
    uint8_t cksum0, cksum1;
	sedAPI_checksum(pktID, pktType, length, data, &cksum0, &cksum1);
	
    for (uint8_t i = 0; i < length; i++) {
        *(packet++) = data[i];
    }
    *(packet++) = cksum0; //finally, send the checksum!
    *(packet++) = cksum1; //finally, send the checksum!
}

/* --------------------------*/
/* -- receiving functions -- */
/* --------------------------*/

uint8_t sedAPI_is_pkt(uint8_t * data)
{
	Sed_base_pkt_t* p = (Sed_base_pkt_t*)data;
	
    if ((p->start0 != START0) || (p->start1 != START1)) { return 2; } //error, downloaded packet header missmatch.
    
    if (p->length > 128) { return 4; } //error, packet too long.
    
	uint8_t cksum0,cksum1;
	sedAPI_checksum(pktID, pktType, length, data, &cksum0, &cksum1);
	
    return 1;
}






void sedAPI_checksum( const uint8_t pktID, const uint8_t pktType, const uint8_t length, const uint8_t *buf,
					 uint8_t *c0, uint8_t *c1 ) {
    uint8_t cksum0 = START0, cksum1 = START0; //start0
	cksum0 += START1; cksum1 += cksum0; //add in start1
	cksum0 += pktID; cksum1 += cksum0; //add in packet ID
	cksum0 += pktType; cksum1 += cksum0; //add in packet type
	cksum0 += length; cksum1 += cksum0; //add in length
	
    for ( uint8_t i = 0; i < length; i++ ) {
        cksum0 += (uint8_t)buf[i];
        cksum1 += cksum0;
    }
    *c0 = cksum0;
    *c1 = cksum1;
}


