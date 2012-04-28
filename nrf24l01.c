


#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "nrf24l01.h"


#define RF_DELAY	5

#define CE_REG		PORTE
#define CE_PIN		7
#define CSN_REG		PORTB //chip select port
#define CSN_PIN		0     //chip select pin number
#define IRQ_REG		PORTE
#define IRQ_PIN		6



//RX Functions
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//Reads the current RX buffer into the data array
//Forces an RX buffer flush
void receive_data(uint8_t * data_array)
{
	CE_REG &= ~(1<<CE_PIN); //Stand by mode
	CSN_REG &= ~(1<<CSN_PIN);
    exchangeSPI(0x61); //Read RX Payload
	data_array[0] = exchangeSPI(0xFF);
	data_array[1] = exchangeSPI(0xFF);
	data_array[2] = exchangeSPI(0xFF);
	data_array[3] = exchangeSPI(0xFF);
	CSN_REG |= (1<<CSN_PIN);
	
	rx_send_byte(0xE2); //Flush RX FIFO
	
	rx_send_command(0x27, 0x40); //Clear RF FIFO interrupt
    
    CE_REG |= (1<<CE_PIN); //Go back to receiving!
}

//2.4G Configuration - Receiver
//This setups up a RF-24G for receiving at 1mbps
void configure_receiver(void)
{
    DDRE |= (1 << CE_PIN); //set Chip Enable as output.
    
    CE_REG &= ~(1<<CE_PIN); //Go into standby mode
    
	rx_send_command(0x20, 0x39); //Enable RX IRQ, CRC Enabled, be a receiver
    
	rx_send_command(0x21, 0x00); //Disable auto-acknowledge
    
	rx_send_command(0x23, 0x03); //Set address width to 5bytes (default, not really needed)
    
	rx_send_command(0x26, 0x07); //Air data rate 1Mbit, 0dBm, Setup LNA
    
	rx_send_command(0x31, 0x04); //4 byte receive payload
    
	rx_send_command(0x25, 0x02); //RF Channel 2 (default, not really needed)
    
	uint8_t data_array[4] = {0xE7,0xE7,0xE7,0xE7};
	rx_send_payload(0x2A, data_array); //Set RX pipe 0 address
    
	rx_send_command(0x20, 0x3B); //RX interrupt, power up, be a receiver
    
    CE_REG |= (1<<CE_PIN); //Start receiving!
}    

//Sends the 4 bytes of payload
void rx_send_payload(uint8_t cmd, uint8_t * data_array)
{
	uint8_t i;
    
	CSN_REG &= ~(1<<CSN_PIN); //Select chip
	exchangeSPI(cmd);
	
	for(i = 0 ; i < 4 ; i++)
		exchangeSPI(data_array[i]);
    
	CSN_REG |= (1<<CSN_PIN); //Deselect chip
}

//Sends command to nRF
uint8_t rx_send_command(uint8_t cmd, uint8_t data)
{
	uint8_t status;
    
	CE_REG &= ~(1<<CE_PIN); //Stand by mode
    
	CSN_REG &= ~(1<<CSN_PIN); //Select chip
	exchangeSPI(cmd);
	status = exchangeSPI(data);
	CSN_REG |= (1<<CSN_PIN); //Deselect chip
	
	return(status);
}

//Sends one byte to nRF
uint8_t rx_send_byte(uint8_t cmd)
{
	uint8_t status;
    
	CSN_REG &= ~(1<<CSN_PIN); //Select chip
	status = exchangeSPI(cmd);
	CSN_REG |= (1<<CSN_PIN); //Deselect chip
	
	return(status);
}




