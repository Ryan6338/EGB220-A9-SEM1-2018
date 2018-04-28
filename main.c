#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <math.h>

#include "Serial.h"
#include "PID_Line_Code.h"
#include "SensorArray.h"

#define KP 1.3
#define KI 0.4
#define KD 0.15

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
	TCCR0A |= (1<<7) | (1<<5) | (1<<WGM10);
	TCCR0B |= (1<<CS02);
	
	//Set Waveform generation to 8-bit, phase corrected PWM.
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	
	TCCR1B |= (1<<CS12); //Set prescaler to 256
	TCCR1B &= ~((1<<CS11) | (1<<CS10) | (1<<WGM13) | (1<<WGM12));
}

//Set motor power (-1.0 to 1.0)
void set_motor_power_LR(double a, double b) {
	
	double scale = fabs(a) > fabs(b) ? fabs(a) : fabs(b);
	if (scale < 1) scale = 1;
	
	double left = a / scale;
	double right = b / scale;
	
	if (left > 0) {
		OCR0A = (uint8_t) (left * 255);
		OCR0B = 0;
	} else {
		OCR0B = (uint8_t) (-left * 255);
		OCR0A = 0;
	}
	
	if (right > 0) {
		OCR1A = (uint8_t) (right * 255);
		OCR1B = 0;
	} else {
		OCR1B = (uint8_t) (-right * 255);
		OCR1A = 0;
	}
}

int main(void) {
	
	initialize_registers();
	initialise_sensors();
	
	//Initialise serial 1. TXD PD3, RXD PD2.
	init_serial(9600);
	
	uint16_t reflected_light_values[8];
	
	//buf = (char*) malloc(20 * sizeof(char*));
	
	double integral = 0;
	double last_error = 0;
	double last_derivate = 0;
	
	while (1) {
		get_reflected_light_values(reflected_light_values);

		//Error between 0 and 1
		double error = calculate_error(reflected_light_values) / 63;
		
		double derivative = ((error - last_error) / 0.002) * 0.5 + last_derivate * 0.5;
		last_derivate = derivative;
		last_error = error;
		
		integral = integral * 0.99 + error * 0.002;
		
		if (integral * error < 0) integral = 0;
		
		double turn_value = error * KP + integral * KI + derivative * KD;
		
		set_motor_power_LR(1 - turn_value, 1 + turn_value);
		
		_delay_ms(2);
	}
	
	return 1;
}

// Transmit reflected light values over serial. Updates a single
// line at a time.

/*

float turn_value = error * KP;
float power_left = 0.15 - turn_value;
float power_right = 0.15 + turn_value;

set_motor_power_LR(power_left, power_right);*/

/*button_states = button_states | button_states << 2;

//Update button state
sw0_states = sw0_states << 1 | ((PINC >> 6) & 1);
sw1_states = sw1_states << 1 | ((PINC >> 7) & 1);

if (sw0_states == 0b11111111) button_states |= (1 << 0);
if (sw1_states == 0b11111111) button_states |= (1 << 1);

if (sw0_states == 0) button_states &= ~(1 << 0);
if (sw1_states == 0) button_states &= ~(1 << 1);

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
		
				/*serial_tx_uint16_t(reflected_light_values[0]);
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
		_delay_ms(200);*/
		
		
///GOOD CODE
/*int main(void) {
	
	initialize_registers();
	initialise_sensors();
	
	//Initialise serial 1. TXD PD3, RXD PD2.
	init_serial(9600);
	
	uint16_t reflected_light_values[8];
	
	//buf = (char*) malloc(20 * sizeof(char*));
	
	double integral = 0;
	double last_error = 0;
	double last_derivate = 0;
	
	while (1) {
		get_reflected_light_values(reflected_light_values);

		//Error between 0 and 1
		double error = calculate_error(reflected_light_values) / 63;
		
		double derivative = ((error - last_error) / 0.002) * 0.5 + last_derivate * 0.5;
		last_derivate = derivative;
		last_error = error;
		
		integral = integral * 0.99 + error * 0.002;
		
		double turn_value = error * KP + integral * KI + derivative * KD;
		
		set_motor_power_LR(0.5 - turn_value, 0.5 + turn_value);
		
		_delay_ms(2);
	}
	
	return 1;
}*/