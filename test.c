#define F_CPU 16000000
#define LIGHT_DELAY 100
#define REPEAT_COUNT 5

#include <avr/io.h>
#include <avr/delay.h>

void set_leds(uint8_t format) {
	if ((format >> 3) & 1) {
		PORTE |=  (1<<6);
	} else {
		PORTE &= ~(1<<6);
	}
	
	if ((format >> 2) & 1) {
		PORTB |=  (1<<0);
	} else {
		PORTB &= ~(1<<0);
	}
	
	if ((format >> 1) & 1) {
		PORTB |=  (1<<1);
	} else {
		PORTB &= ~(1<<1);
	}
	
	if (format & 1) {
		PORTB |=  (1<<2);
	} else {
		PORTB &= ~(1<<2);
	}
}

void main() {
	//Set LED0-4 to output pins
	DDRE |= (1<<6);
	DDRB |= 0b111;
	
	DDRC &= ~(0b11<<6);
	PORTC |= (0b11<<6);
	
	uint8_t code = 0b0000;
	
	uint8_t last_switch_states = 0;
	
	while (1) {
		uint8_t current_switch_states = (PINC>>6)&0b11;
		
		if (current_switch_states&1 && !(last_switch_states&1)) {
			code--;
		}
		
		if ((current_switch_states>>1)&1 && !((last_switch_states>>1)&1)) {
			code++;
		}
		
		last_switch_states = current_switch_states;
		
		set_leds(code);
		
	}
	
}