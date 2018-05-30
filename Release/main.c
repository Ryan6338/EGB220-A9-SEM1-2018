#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "PID_Line_Code.h"
#include "SensorArray.h"
#include "RobotControl.h"
#include "main.h"

static volatile uint32_t overflow_counter = 0;
static volatile uint8_t sw0_states = 0;
static volatile uint8_t sw1_states = 0;
static volatile uint8_t button_states = 0;

ISR(TIMER0_OVF_vect) {
	overflow_counter++;
	
	/**
    *  Debouncing buttons
	 **/
	
	//Shift button position into switch states. Takes ~3ms to achieve a switch
	sw0_states = sw0_states << 1 | ((PINC >> 6) & 1);
	sw1_states = sw1_states << 1 | ((PINC >> 7) & 1);
	
	//If button fully down, set on
	if (sw0_states == 0b1111) button_states |= (1 << 0);
	if (sw1_states == 0b1111) button_states |= (1 << 1);
	
	//If button fully off, set off
	if (sw0_states == 0) button_states &= ~(1 << 0);
	if (sw1_states == 0) button_states &= ~(1 << 1);
}

double get_time() {
	return 256.0 * ((overflow_counter * 256.0 + TCNT0) / F_CPU);
}

int main(void) {
	
	sei(); //Enable interrupts
	
	initialise_registers();
	initialise_sensors();
	
	uint16_t reflected_light_values[10];
	
	RobotState robot_state = STOPPED;
	
	double last_time = get_time();
	double lost_time = -1;
	double start_time = -1;
	double filtered_turn = 0;
	
	uint8_t last_state = 0;
	uint16_t addr = 0;
	
	while (1) {
		
		get_reflected_light_values(reflected_light_values);
		
		double current_time = get_time();
		double dt = current_time - last_time;
		last_time = current_time;
		
		if ((button_states >> 1) & 1 && robot_state == STOPPED) robot_state = COUNTDOWN;
		else if (button_states & 1) robot_state = STOPPED;
		
		switch (robot_state) {
			
		case COUNTDOWN:
			if (start_time == -1) start_time = get_time();
			
			double countdown_time = get_time() - start_time;
			
			if (countdown_time < COUNTDOWN_TIME / 4.0) set_leds(0b1000);
			else if (countdown_time < COUNTDOWN_TIME * 2 / 4.0) set_leds(0b1100);
			else if (countdown_time < COUNTDOWN_TIME * 3 / 4.0) set_leds(0b1110);
			else if (countdown_time < COUNTDOWN_TIME * 4 / 4.0) set_leds(0b1111);
			else if (countdown_time > COUNTDOWN_TIME * 4 / 4.0) robot_state = RUNNING;
			break;
			
			
		case RUNNING:
			get_reflected_light_values(reflected_light_values);
			
			switch (check_sensor_states(reflected_light_values)) {
			case NORMAL: //Normal Operation
			
				/*if (filtered_turn > -SLIGHT_TURN && filtered_turn < SLIGHT_TURN) {
					set_leds(0b1001);
				} else if (filtered_turn > SLIGHT_TURN && filtered_turn < HARD_TURN) {
					set_leds(0b0010);
				} else if (filtered_turn > HARD_TURN) {
					set_leds(0b0001);
				} else if (filtered_turn < -SLIGHT_TURN && filtered_turn > -HARD_TURN) {
					set_leds(0b0100);
				} else if (filtered_turn < -HARD_TURN) {
					set_leds(0b1000);
				}*/
				
				set_leds(0b0000);
			
				lost_time = -1; // Robot not lost
				
				double turn_value = calculate_PID_turn_value(reflected_light_values, dt);
				
				filtered_turn = filtered_turn * (1 - TURN_FILTER) + turn_value * TURN_FILTER;
				
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
				break;
			}
			
			break;
		
		
		case STOPPED:
			start_time = -1;
			set_motor_power_LR(0, 0);
			set_leds(0b0000);
			break;
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