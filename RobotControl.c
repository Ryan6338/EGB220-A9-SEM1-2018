#include "RobotControl.h"

void initialise_registers() {
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
	
	//Enable interrupts on timer 0
	TIMSK0 |= (1<<0);
	
	/*
	 * PWM Setup
	 */
	TCCR0A |= (1<<7) | (1<<5) | (1<<WGM10); //8-bit, phase corrected PWM
	TCCR0B |= (1<<CS02); //Prescaler = 256
	
	//Set Waveform generation to 8-bit, phase corrected PWM.
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10); 
	TCCR1A &= ~(1<<WGM11);
	
	TCCR1B |= (1<<CS12); //Set prescaler to 256
	TCCR1B &= ~((1<<CS11) | (1<<CS10) | (1<<WGM13) | (1<<WGM12)); //Ensure other bits are cleared
}

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

//Set motor power (-1.0 to 1.0)
void set_motor_power_LR(double b, double a) {
	
	double scale = fabs(a) > fabs(b) ? fabs(a) : fabs(b); //Set scale to largest speed

	if (scale < 1) scale = 1;
	
	double left = a / scale;
	double right = b / scale;
	
	if (left < 0) {
		OCR0A = (uint8_t) (-left * 255);
		OCR0B = 0;
	} else {
		OCR0B = (uint8_t) (left * 255);
		OCR0A = 0;
	}
	
	if (right < 0) {
		OCR1A = (uint8_t) (-right * 255);
		OCR1B = 0;
	} else {
		OCR1B = (uint8_t) (right * 255);
		OCR1A = 0;
	}
}