#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include "SensorArray.h"

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
	
	uint8_t last_switch_states = 0;
	
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
void set_motor_power_LR(float left, float right) {
	if (left > 0) {
		OCR0A = (uint8_t) (left * 255);
		OCR0B = 0;
	} else {
		OCR0B = (uint8_t) -(left * 255);
		OCR0A = 0;
	}
	
	if (right > 0) {
		OCR1A = (uint8_t) (right * 65535);
		OCR1B = 0;
	} else {
		OCR1B = (uint8_t) -(right * 65535);
		OCR1A = 0;
	}
}

int main(void) {
	
	initialize_registers();
	initialise_sensors();
	
	uint16_t * reflected_light_values = get_reflected_light_values();
	
	set_motor_power_LR((double) reflected_light_values[0] / 1024.0, 0);
	
	return 1;
}