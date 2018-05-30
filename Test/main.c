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

static volatile struct {
	uint8_t button0_state : 1;
	uint8_t button1_state : 1;
	uint8_t button0_changed : 1;
	uint8_t button1_changed : 1;
} button_states;

static volatile uint32_t overflow_counter = 0;
static volatile uint8_t sw0_states = 0;
static volatile uint8_t sw1_states = 0;

ISR(TIMER0_OVF_vect) {
	overflow_counter++;
	
	/**
    *  Debouncing buttons
	 **/
	
	//Shift button position into switch states. Takes ~3ms to achieve a switch
	sw0_states = sw0_states << 1 | ((PINC >> 6) & 1);
	sw1_states = sw1_states << 1 | ((PINC >> 7) & 1);
	
	//If button fully down, set on
	if (sw0_states == 0b1111 && button_states.button0_state != 1) {
		button_states.button0_state = 1;
		button_states.button0_changed = 1;
	}
	
	if (sw1_states == 0b1111 && button_states.button1_state != 1) {
		button_states.button1_state = 1;
		button_states.button1_changed = 1;
	}
	
	//If button fully off, set off
	if (sw0_states == 0 && button_states.button0_state != 0) {
		button_states.button0_state = 0;
		button_states.button0_changed = 1;
	}
	
	if (sw1_states == 0 && button_states.button1_state != 0) {
		button_states.button1_state = 0;
		button_states.button1_changed = 1;
	}
	
}

double get_time() {
	return 256.0 * ((overflow_counter * 256.0 + TCNT0) / F_CPU);
}

double turns[30];

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
	
	double max_speed = 1;
	
	RobotState selection = 0;
	
	double turns[30];
	uint8_t current_turn = 0;
	
	double sum_turn_value = 0;
	uint32_t turn_value_count = 0;
	
	uint8_t right_sensor_count = 0;
	double last_right_sensor_count_time = 0;
	double last_left_sensor_count_time = 0;
	
	uint16_t addr = 0;
	
	while (1) {
	
		get_reflected_light_values(reflected_light_values);
	
		if (button_states.button0_state && button_states.button0_changed) {
			int i;
			for (i = 0; i < 10; i++) {
				button_states.button0_changed = 0;
				eeprom_write_word((uint16_t*) addr, reflected_light_values[i]);
				addr += 2;
				if (i % 2) set_leds (0b1111);
				else set_leds(0b0000);
				
				_delay_ms(50);
			}
		}
	
		set_leds(right_sensor_count);
	
		if (reflected_light_values[9] < 500) {
			if (get_time() - last_right_sensor_count_time > 0.02) {
				right_sensor_count++;
				last_right_sensor_count_time = get_time();
			}
			last_right_sensor_count_time = get_time();
		}
			
		if (reflected_light_values[0] < 500) {
			if (get_time() - last_left_sensor_count_time > 0.02) {
				current_turn++;
			}
			last_left_sensor_count_time = get_time();
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