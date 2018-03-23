#include "Serial.h"

void init_serial( unsigned int baud )
{
	
	//Calculate baud divider value using desired baud rate
	uint16_t baud_divider = ((uint32_t) F_CPU / (16l * (uint32_t) baud)) - 1;
	
	/* Set baud rate */
	UBRR1H = (unsigned char) (baud_divider >> 8);
	UBRR1L = (unsigned char) baud_divider;
	
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);
	
	/* Set frame format: 8data, 1stop bit */
	UCSR1C = (3<<UCSZ10);
	
	DDRD |=  (1<<3); //Enable TXD pin as output
	DDRD &= ~(1<<2); //Enable RXD pin as input
}

uint8_t serial_rx( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) );
	
	/* Get and return received data from buffer */
	return (uint8_t) UDR1;
	
	/* Wait for transmission to finish */
	while ( !(UCSR1A & (1<<RXC1)) );
}

void serial_tx( uint8_t data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );
	
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void serial_print(uint8_t * msg, uint8_t len) {
	uint8_t i;
	
	//Loop through msg until length value is reached
	for (i = 0; i < len; i++) {
		
		uint8_t c = msg[i];
		//If character is blank, exit loop
		if (c == 0) {
			break;
		} else {
			serial_tx((uint8_t) c);
		}
	}
}

void serial_println(uint8_t * msg, uint8_t len) {
	uint8_t i;
	
	//Print message to serial
	serial_print(msg, len);
	
	//Transmit new line sequence
	serial_tx('\n');
	serial_tx('\r');
}


void serial_tx_uint16_t(uint16_t i) {
	//Initialise buffer. Max value for uint16_t is 65535, thus max chars needed is 6 (one for termination)
	uint8_t buf[6];
	
	//Set buffer to 0
	memset(buf, 0, 6);
	
	//Convert unsigned integer i to string in buf
	utoa(i, (char*)buf, 10);
	
	//Transmit contents of buffer over UART
	serial_println(buf, 6);
}

void serial_cls() {
	uint8_t buf[2];
	
	//Escape sequence to clear screen: ESC c
	buf[0] = 27;
	buf[1] = (uint8_t) 'c';
	
	//Send cls command over UART
	serial_println(buf, 2);
}

void serial_cursor_0() {
	uint8_t buf[4];
	
	//Escape sequence to set cursor to 0,0: ESC[;H
	buf[0] = 27;
	buf[1] = (uint8_t) '[';
	buf[2] = (uint8_t) ';';
	buf[3] = (uint8_t) 'H';
	
	//Transmit escape sequence
	serial_println(buf, 4);
}