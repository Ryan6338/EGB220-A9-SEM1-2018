#ifndef SERIAL_H
#define SERIAL_H

#define F_CPU 16000000

#include <util/delay.h>
#include <avr/io.h>
#include <String.h>
#include <stdlib.h>


/* init_serial ( unsigned int baud )
 * Call this function to initialise the serial registers
 * and set to specified baud rate.
 */
void init_serial( unsigned int baud );


/* serial_tx ( uint8_t data )
 * 
 * Send a single byte over UART
 *
 * params:
 * data - byte to transmit over uart
 */
void serial_tx( uint8_t data );


/* serial_tx ( uint8_t data )
 * 
 * Wait for a single byte over UART
 * Blocks thread until byte is recieved
 *
 * returns: byte of data recieved over UART
 */
uint8_t serial_rx( void );


/* serial_println(uint8_t * msg, uint8_t len)
 * 
 * Send a string over UART terminated by new line sequence
 *
 * params:
 *	uint8_t * msg - String to send over UART
 *	uint8_t len - Length of string. String terminates when len is
 *								reached. Null byte is also treated as termination,
 *								even before len is reached
 *
 */
void serial_println(uint8_t * msg, uint8_t len);


/* serial_tx_uint16_t(uint16_t i)
 *
 * Transmit a uint16_t formatted as a string
 * over UART
 */
void serial_tx_uint16_t(uint16_t i);

/* serial_tx_uint16_t(uint16_t i)
 *
 * Transmit a uint16_t formatted as a string
 * over UART
 */
void serial_tx_int16_t(int16_t i);


/* serial_cls()
 *
 * Clear UART reciever terminal using ANSI esc sequence
 */
void serial_cls();


/**
 * serial_cls()
 *
 * Set cursor on UART reciever terminal to (0,0) using ANSI esc sequence
 */
void serial_cursor_0();

#endif