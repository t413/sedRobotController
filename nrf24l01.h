/******************************************************************************
*
* File: nrf24l01.h
* 
* Copyright S. Brennen Ball, 2006-2007
* 
* The author provides no guarantees, warantees, or promises, implied or
*	otherwise.  By using this software you agree to indemnify the author
* 	of any damages incurred by using it.
*
*****************************************************************************/

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stddef.h>
#include <stdlib.h>
#include "spi.h"
#include <util/delay.h>
#define delay_us(microseconds)		_delay_us(microseconds)
#include <avr/io.h>



//2.4G Configuration - Receiver
void configure_receiver(void);
//Sends one byte to nRF
uint8_t rx_send_byte(uint8_t cmd);
//Sends command to nRF
uint8_t rx_send_command(uint8_t cmd, uint8_t data);
//Sends the 4 bytes of payload
void rx_send_payload(uint8_t cmd, uint8_t * data_array);
//Basic SPI to nRF
uint8_t rx_spi_byte(uint8_t outgoing);

void receive_data(uint8_t * data_array);

#endif /*NRF24L01_H_*/
