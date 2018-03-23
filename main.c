#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <String.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>

#include "SensorArray.h"
#include "Serial.h"

//Set leds using bitmap
void set_leds(uint8_t format) {
	if (format & 1) {
		PORTE |=  (1<<6);
	} else {
		PORTE &= ~(1<<6);
	}
	
	if ((format >> 1) & 1) {
		PORTB |=  (1<<0);
	} else {
		PORTB &= ~(1<<0);
	}
	
	if ((format >> 2) & 1) {
		PORTB |=  (1<<1);
	} else {
		PORTB &= ~(1<<1);
	}
	
	if ((format >> 3) & 1) {
		PORTB |=  (1<<2);
	} else {
		PORTB &= ~(1<<2);
	}
}

void initialize_registers() {
	//Set LED0-3 to output pins
	DDRE |= (1<<6);
	DDRB |= 0b111;
	
	//Enable buttons as inputs
	DDRC &= ~(0b11<<6);
	PORTC |= (0b11<<6);
	
	//uint8_t last_switch_states = 0;
	
	//Enable motor pins as outputs
	DDRB |= (1<<7) | (1<<6) | (1<<5);
	DDRD |= (1<<0);
	
	//Prescaler set to 256
	TCCR0A |= (1<<7)|(1<<5)|(1<<1)|1;
	TCCR0B |= (1<<2);
	
	//Prescaler set to 64
	TCCR1A |= (1<<7)|(1<<5)|(1<<1)|1;
	TCCR1B |= (1<<1);
}

//Set motor power (-1.0 to 1.0)
void set_motor_power_LR(uint8_t left, uint8_t right) {
	if (left > 0) {
		OCR0A = left;
		OCR0B = 0;
	} else {
		OCR0B = left;
		OCR0A = 0;
	}
	
	if (right > 0) {
		OCR1A = right;
		OCR1B = 0;
	} else {
		OCR1B = right;
		OCR1A = 0;
	}
}

uint8_t sw0_states;
uint8_t sw1_states;
uint8_t current_button_states;

int main(void) {
	
	initialize_registers();
	initialise_sensors();
	
	//Initialise serial 1. TXD PD3, RXD PD2.
	init_serial(9600);
	
	uint16_t reflected_light_values[8];
	
	//buf = (char*) malloc(20 * sizeof(char*));
	
	while (1) {
		
		get_reflected_light_values(reflected_light_values);
		
		// Transmit reflected light values over serial. Updates a single
		// line at a time.
		serial_cursor_0();
		serial_tx_uint16_t(reflected_light_values[0]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[1]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[2]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[3]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[4]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[5]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[6]);
		_delay_ms(50);
		serial_tx_uint16_t(reflected_light_values[7]);
		_delay_ms(200);
		
		//USART_Transmit((uint8_t) buf[0]);
		//_delay_ms(20);
		
		/*
		//Update button state
		sw0_button_state = sw0_button_state << 1 | ((PINC >> 6) & 1);
		sw1_button_state = sw1_button_state << 1 | ((PINC >> 7) & 1);
		
		if (sw0_states == 0b11111111) current_button_states |= (1 << 0);
		if (sw1_states == 0b11111111) current_button_states |= (1 << 1);
		
		if (sw0_states == 0) current_button_states &= ~(1 << 0);
		if (sw1_states == 0) current_button_states &= ~(1 << 1);
		
		if (((current_button_states >> 0) & 1) == 1) set_motor_power_LR(0, 0);
		if (((current_button_states >> 1) & 1) == 1) set_motor_power_LR(1, 1);*/
	
		/*uint8_t speed = reflected_light_values[0];
		set_motor_power_LR(speed, speed);*/
	}
	
	return 1;
}