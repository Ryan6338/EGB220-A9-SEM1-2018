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

#define CONSTANTS_FAST 0.5, 2.9, 1.35
#define CONSTANTS_HALF 0.5, 2.9, 1.35
#define CONSTANTS_SLOW 0.5, 2.9, 1.35

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
		button_states.button0_changed = 1; //Set changed flag, must be unset in main code
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

//Return run time in seconds
double get_time() {
	return 256.0 * ((overflow_counter * 256.0 + TCNT0) / F_CPU);
}

int main(void) {
	
	sei(); //Enable interrupts
	
	initialise_registers();
	initialise_sensors();
	
	uint16_t reflected_light_values[10];
	
	RobotState robot_state = STOPPED;
	RobotState selection = 0;
	
	double last_time = get_time();
	double time0 = -1; //General timer
	double max_speed = 1;
	double current_speed = 1;
	double offset = 0;
	
	//Array of turn values between markers
	double turns[30];
	eeprom_read_block(turns, (uint16_t*) 1, 120);
	
	uint8_t current_turn = 0;
	uint8_t slow_marker = 0;
	double sum_turn_value = 0;
	uint32_t turn_value_count = 0;
	
	uint8_t right_sensor_count = 0;
	double last_right_sensor_count_time = 0;
	double last_left_sensor_count_time = 0;
	
	while (1) {
		
		get_reflected_light_values(reflected_light_values);
		
		double current_time = get_time();
		double dt = current_time - last_time;
		last_time = current_time;
		
		if (button_states.button0_state && button_states.button0_changed) {
			button_states.button0_changed = 0; //Clear changed flag
			
			if (robot_state == STOPPED)
				selection = (selection + 1) % 4;
			else robot_state = STOPPED;
		} else if (button_states.button1_state && button_states.button1_changed) {
			button_states.button1_changed = 0;
			if (selection != STOPPED && robot_state == STOPPED) 
				robot_state = COUNTDOWN;
		}
		
		switch (robot_state) {
			
		case COUNTDOWN:
			if (time0 == -1) time0 = get_time();
			
			double countdown_time = get_time() - time0;
			
			if (countdown_time < COUNTDOWN_TIME / 4.0) set_leds(0b1000);
			else if (countdown_time < COUNTDOWN_TIME * 2 / 4.0) set_leds(0b1100);
			else if (countdown_time < COUNTDOWN_TIME * 3 / 4.0) set_leds(0b1110);
			else if (countdown_time < COUNTDOWN_TIME * 4 / 4.0) set_leds(0b1111);
			else if (countdown_time > COUNTDOWN_TIME * 4 / 4.0) {
				current_turn = 0;
				robot_state = selection;
				selection = STOPPED;
				
				switch(robot_state) {
				case RUNNING_FULLSPEED:
					max_speed = 1;
					set_pid_constants(CONSTANTS_FAST);
					break;
				case CALIBRATION:
					sum_turn_value = 0;
					turn_value_count = 0;
				case RUNNING_HALFSPEED:
					max_speed = 0.5;
					set_pid_constants(CONSTANTS_HALF);
					break;
				}
			}
			break;
			
		//All these modes run the same code
		case CALIBRATION:		
		case RUNNING_HALFSPEED:
		case RUNNING_FULLSPEED:
			//Read sensor values into reflected_light_values
			get_reflected_light_values(reflected_light_values);
			
			//If left outrigger senses white
			if (reflected_light_values[0] < 600) {
				//If it has been more than 20ms since it has last seen white
				if (get_time() - last_left_sensor_count_time > 0.02) {
					
					//If robot is in calibration mode
					if (robot_state == CALIBRATION) {
						//Calculate average turn value
						double avg_turn_value = sum_turn_value / (double) turn_value_count;
					
						//If average turn value is small, it was a straight
						//if (avg_turn_value < 0.1) avg_turn_value = 0;
						
						//If this section and last section was straight, this section is slow zone
						if (avg_turn_value == 0 && turns[current_turn - 1] == 0 && slow_marker != current_turn-1)
							slow_marker = current_turn;
						
						//Store turns value
						turns[current_turn] = avg_turn_value;
						
						//Set turning offset to 0 as it is unknown
						offset = 0;
						
						//Reset parameters to calculate turn value 
						sum_turn_value = 0;
						turn_value_count = 0;
						
						current_turn++;
						
					} else { 
					
						current_turn++;
						
						offset = turns[current_turn]; //Offset = average turn value			
						if (current_turn == slow_marker) {
							current_speed = 0.2;
							set_pid_constants(CONSTANTS_SLOW);
						} else {
							current_speed = max_speed;
							if(robot_state == RUNNING_FULLSPEED) set_pid_constants(CONSTANTS_FAST);
							else set_pid_constants(CONSTANTS_HALF);
						}
					}
				}
				
				//Set last white seen time to current time
				last_left_sensor_count_time = get_time();
			}
			
			//If right outrigger senses white
			if (reflected_light_values[9] < 600) {
				//If it has been more than 20ms since it last sensed white
				if (get_time() - last_right_sensor_count_time > 0.02) {
					//Increment times sensor has seen marker
					right_sensor_count++;
					
					//Second right marker is finish line
					if (right_sensor_count >= 2){
						//If robot is in calibration mode, save turn values to EEPROM
						if (robot_state == CALIBRATION){
							eeprom_write_byte((uint8_t*) 0, current_turn);
							eeprom_write_block((void*) turns, (uint16_t*) 1, 120);
						}
						//Run FINISH sequence
						robot_state = FINISH;
						break;
					}
				}
				//Set last white seen time to current time
				last_right_sensor_count_time = get_time();
			}
			
			switch (check_sensor_states(reflected_light_values)) {
			case NORMAL: //Normal Operation				
				set_leds(0b0000);
			
				time0 = -1; // Robot not lost
				
				//Calculate correction value.
				double correction_value = calculate_PID_turn_value(reflected_light_values, dt);
				sum_turn_value += correction_value;
				turn_value_count++;
				
				set_differential_power(current_speed, correction_value + offset);
				break;
			case WHITE: //White line crossover
			
				if (sin(get_time() * 150) > 0) set_leds(0b1111);
				else (set_leds(0));
			
				if (time0 == -1) time0 = get_time();
				else if (get_time() - time0 < 0.2) set_motor_power_LR(current_speed, current_speed);
				else if (get_time() - time0 < 0.4) set_motor_power_LR(-0.1, -0.1); //Brake hard
				else if (get_time() - time0 < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - time0 > 0.6) robot_state = 0;
				break;
			case BLACK: //All black, robot lost.
			
				if (sin(get_time() * 150) > 0) set_leds(0b1111);
				else (set_leds(0));
			
				if (time0 == -1) time0 = get_time(); //Set lost time to current time
				else if (get_time() - time0 < 0.4 && get_time() - time0 > 0.2) set_motor_power_LR(-0.1, -0.1); //Brake hard
				else if (get_time() - time0 < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - time0 > 0.6) robot_state = 0;
				break;
			}
			
			break;
		
		case FINISH:
			if (time0 == -1) time0 = get_time();
			if (get_time() - time0 < 0.5 / max_speed) set_motor_power_LR(max_speed, max_speed);
			else if (get_time() - time0 < (0.5 / max_speed) + 0.2) set_motor_power_LR(-0.2, -0.2);
			else robot_state = 0;
			break;
		
		case STOPPED:
			time0 = -1;
			set_motor_power_LR(0, 0);
			if (selection > 0) set_leds(1 << selection - 1); else set_leds(0);
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