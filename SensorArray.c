#include "SensorArray.h"

void initialise_sensors() {
	//Setup registers for inputs on sensor pins
	DDRF &= ~(1<<4) 	//S1
				& ~(1<<5) 	//S2
				& ~(1<<6) 	//S3
				& ~(1<<7);	//S4
				
	DDRB &= ~(1<<4);	//S5
	
	DDRD &= ~(1<<7) 	//S6
				& ~(1<<6)		//S7
				& ~(1<<4);	//S8
	
	
	
	//Set LED ON pin as output
	DDRB |= (1<<3);
	PORTB |= (1<<3); //Turn on (Pull high)
	
	
	//Initialize ADC
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) //Prescaler - 64 (250kHz)
					| (1<<ADEN);	//Enable ADC

	ADCSRB |= (1<<ADHSM); // Enable ADC High-Speed mode for bettery resolution at high speed				
					
	ADMUX |=	(1<<REFS0); 	//Set reference voltage to AVcc (5v)
					//| (1<<ADLAR);	//Left adjust ADC
}

uint16_t read_sensor(uint8_t sensor) {
	//Set ADC channel using ADMUX4:0 & ADCSRB5
	//See datasheet p.313 for ADC channel map
	
	ADCSRB &= ~(1<<MUX5); //Clear MUX5
	ADMUX &= ~0b11111; //Clear ADMUX 4:0
	
	switch (sensor) {
		case 0:
			ADMUX |= 0b00100; //S1, ADC4
			break;
		case 1:
			ADMUX |= 0b00101; //S2, ADC5
			break;
		case 2:
			ADMUX |= 0b00110; //S3, ADC6
			break;
		case 3:
			ADMUX |= 0b00111; //S4, ADC7
			break;
		case 4:
			ADCSRB |= (1<<5);
			ADMUX |= 0b00011; //S5, ADC11
			break;
		case 5:
			ADCSRB |= (1<<5);
			ADMUX |= 0b00010; //S6, ADC10
			break;
		case 6:
			ADCSRB |= (1<<5);
			ADMUX |= 0b00001; //S7, ADC9
			break;
		case 7:
			ADCSRB |= (1<<5);
			ADMUX |= 0b00000; //S8, ADC8
			break;
	}
	
	//Start ADC conversion. ADSC clears when complete.
	ADCSRA |= (1<<ADSC);
	
	//Block until ADSC clears then return value. ////Thread blocking is not good
	while(((ADCSRA>>ADSC)&1));
	
	return (ADCH<<8) | ADCL; //Return value read from ADCH & ADCL registers
	
}

//Read all sensor values and return in an array
uint16_t * get_reflected_light_values() {
	reflected_light[0] = read_sensor(0);
	reflected_light[1] = read_sensor(1);
	reflected_light[2] = read_sensor(2);
	reflected_light[3] = read_sensor(3);
	reflected_light[4] = read_sensor(4);
	reflected_light[5] = read_sensor(5);
	reflected_light[6] = read_sensor(6);
	reflected_light[7] = read_sensor(7);
	
	return reflected_light;
}