#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <math.h>

//#include "Serial.h"
#include "PID_Line_Code.h"
#include "SensorArray.h"

#define KP 2
#define KI 2.4
#define KD 0.7
#define MAX_SPEED 1

volatile uint32_t overflow_counter = 0;
volatile uint8_t sw0_states = 0;
volatile uint8_t sw1_states = 0;
volatile uint8_t button_states = 0;

ISR(TIMER0_OVF_vect) {
	overflow_counter++;
	
	/**
    *  Debouncing buttons
	 **/
	//Shift button position into switch states. Takes ~3ms to achieve a switch
	sw0_states = sw0_states << 1 | ((PINC >> 6) & 1);
	sw1_states = sw1_states << 1 | ((PINC >> 7) & 1);
	
	//If button fully down, set on
	if (sw0_states == 0xFF) button_states |= (1 << 0);
	if (sw1_states == 0xFF) button_states |= (1 << 1);
	
	//If button fully off, set off
	if (sw0_states == 0) button_states &= ~(1 << 0);
	if (sw1_states == 0) button_states &= ~(1 << 1);
}

double get_time() {
	return 256.0 * ((overflow_counter * 256.0 + TCNT0) / F_CPU);
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

//Set motor power (-1.0 to 1.0)
void set_motor_power_LR(double b, double a) {
	
	double scale = fabs(a) > fabs(b) ? fabs(a) : fabs(b); //Set scale to largest speed

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
	
	sei();
	
	initialize_registers();
	initialise_sensors();
	
	//Initialise serial 1. TXD PD3, RXD PD2.
	//init_serial(9600);
	
	uint16_t reflected_light_values[8];
	
	uint8_t robot_running = 0;
	
	double integral = 0;
	double last_error = 0;
	double last_derivate = 0;
	double last_time = get_time();
	double lost_time = 0;
	
	
	set_leds(0b0101);
	while (1) {
		
		get_reflected_light_values(reflected_light_values);
	
		double current_time = get_time();
		double dt = current_time - last_time;
		last_time = current_time;
		
		if (button_states & 1) robot_running = 1;
		else if (button_states >> 1 & 1) robot_running = 0;
		
		if (robot_running == 1) {
			
			set_leds(button_states);
			
			get_reflected_light_values(reflected_light_values);

			switch (check_sensor_states(reflected_light_values)) {
			case 0: //Normal Operation
				lost_time = 0; // Robot not lost
			
				//Error value (between -1 and 1)
				double error = calculate_error(reflected_light_values);
				
				//Exponential filter on derivative to reduce erratic behaviour
				double derivative = ((error - last_error) / 0.01) * 0.5 + last_derivate * 0.5;
				last_derivate = derivative;
				last_error = error;
				
				integral = integral * (1 - 0.01) + error * 0.01;
				
				//filtered_turn = filtered_turn * (1 - TURN_FILTER) + turn_value * TURN_FILTER;
				if (integral * error < 0) integral = 0;
				
				double turn_value = error * KP + integral * KI + derivative * KD;
				
				set_differential_power(MAX_SPEED, turn_value);
				break;
				
			case WHITE: //White line crossover
			
				if (sin(get_time() * 150) > 0) set_leds(0b1111);
				else (set_leds(0));
			
				if (lost_time == -1) lost_time = get_time();
				else if (get_time() - lost_time < 0.2) set_motor_power_LR(MAX_SPEED, MAX_SPEED);
				else if (get_time() - lost_time < 0.4) set_motor_power_LR(-0.1, -0.1); //Brake hard
				else if (get_time() - lost_time < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - lost_time > 0.6) robot_state = STOPPED;
				break;
			case BLACK: //All black, robot lost.
			
				if (sin(get_time() * 150) > 0) set_leds(0b1111);
				else (set_leds(0));
			
				if (lost_time == -1) lost_time = get_time(); //Set lost time to current time
				else if (get_time() - lost_time < 0.4 && get_time() - lost_time > 0.2) set_motor_power_LR(-0.1, -0.1); //Brake hard
				else if (get_time() - lost_time < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - lost_time > 0.6) robot_state = STOPPED;

			case 1: //White line crossover
				if (lost_time == 0) lost_time = get_time();
				else if (get_time() - lost_time > 0.4) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - lost_time > 0.2) set_motor_power_LR(-0.1, -0.1); //Brake hard
				else set_motor_power_LR(MAX_SPEED, MAX_SPEED);
				break;
			case 2: //All black, robot lost.
				if (lost_time == 0) lost_time = get_time(); //Set lost time to current time
				else if (get_time() - lost_time > 0.4) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - lost_time > 0.2) set_motor_power_LR(-0.1, -0.1); //Brake hard
				break;
			}
			_delay_ms(1);
		} else {
			set_leds(0b0000);
		}
	}
	
	return 1;
}

//USART_Transmit((uint8_t) buf[0]);
//_delay_ms(20);

/*if (sw0_states == 0) current_button_states &= ~(1 << 0);
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