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

#define CONSTANTS_FAST 1.7, 0.2, 0.6
#define CONSTANTS_CALIB 0.9, 1.2, 0.7
#define CONSTANTS_STRAIGHT 0.05, 0.3, 0.75
#define CONSTANTS_HALF 2, 0.1, 1
#define CONSTANTS_SLOW 1.2, 0.5, 1.8

#define TURN_THRESHOLD 0.1

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

uint8_t inRange(double a, double range) {
	return (a < range && a > -range);
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
	double timeMarker = -1;
	double lastTimeMarker;
	double max_speed = 1;
	double current_speed = 1;
	double slow_time = eeprom_read_dword((uint32_t*)202);
	double offset = 0;
	
	//Array of turn values between markers
	double turns[50];
	eeprom_read_block(turns, (uint16_t*) 2, 200);
	int a;
	
	uint8_t slow_marker = 0;
	
	//Process EEPROM data to find slow marker and set all turns below theshold to 0
	for (a = 1; a<eeprom_read_byte(0); a++) {
		if (inRange(turns[a], TURN_THRESHOLD) && inRange(turns[a-1], TURN_THRESHOLD) && slow_marker != a-1) slow_marker = a;
	}
	
	uint8_t current_turn = 0;
	double filtered_turn_value = 0;
	
	//Cancel flags so crossovers aren't counted
	uint8_t cancel = 0;
	uint8_t save = 0;
	
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
				time0 = -1;
				selection = STOPPED;
				
				right_sensor_count = 0;
				
				switch(robot_state) {
				case RUNNING_FULLSPEED:
					max_speed = 0.99;
					current_speed = max_speed;
					set_pid_constants(CONSTANTS_FAST);
					break;
				case CALIBRATION:
					filtered_turn_value = 0;
					set_pid_constants(CONSTANTS_CALIB);
					max_speed = 0.7;
					current_speed = max_speed;
					break;
				case RUNNING_HALFSPEED:
					max_speed = 0.5;
					current_speed = max_speed;
					set_pid_constants(CONSTANTS_HALF);
					break;
				}
			}
			break;
			
		//All these modes run the same code
		case CALIBRATION:		
			set_leds(current_turn);
		case RUNNING_HALFSPEED:
		case RUNNING_FULLSPEED:
			//Read sensor values into reflected_light_values
			get_reflected_light_values(reflected_light_values);
			
			//If left outrigger senses white
			if (reflected_light_values[0] < 650) {
				if (check_sensor_states(reflected_light_values) == WHITE) cancel |= (1<<1);
				
				//If it has been more than 20ms since it last sensed white
				if (get_time() - last_left_sensor_count_time > 0.02) {
					
					if ((cancel>>1) & 1){
						cancel &= ~(1<<1);
						break;
					} 
					
					//If robot is in calibration mode
					if (robot_state == CALIBRATION) {
						
						if (current_turn == 0) filtered_turn_value = 0;
						
						//If this section and last section was straight, this section is slow zone
						if (inRange(filtered_turn_value, TURN_THRESHOLD) && inRange(turns[current_turn - 1], TURN_THRESHOLD) && slow_marker != current_turn-1) {
							slow_marker = current_turn;
							slow_time = timeMarker - lastTimeMarker; 
						}
						
						if (right_sensor_count == 1) turns[current_turn] = filtered_turn_value;
						
						
						//Set turning offset to 0 as it is unknown
						offset = 0;
						
						//Reset parameters to calculate turn value 
						filtered_turn_value = 0;
						
						lastTimeMarker = timeMarker;
						timeMarker = get_time();
						current_turn++;
						
					} else { 
					
						turns[current_turn] = turns[current_turn] * 0.75 + (offset + filtered_turn_value) * 0.25;
					
						//Increase turn value
						current_turn++;
						
						offset = turns[current_turn]; //Offset = average turn value			
						if ((current_turn == slow_marker - 1 && robot_state != CALIBRATION)|| (current_turn == slow_marker && robot_state != CALIBRATION)) {
							current_speed = 0.12;
							set_pid_constants(CONSTANTS_SLOW);
						} else if (inRange(turns[current_turn], TURN_THRESHOLD)) {
							set_leds(0b1111);
							current_speed = 0.95;
							set_pid_constants(CONSTANTS_STRAIGHT);
						} else {
							set_leds(0b0000);
							current_speed = max_speed;
							if(robot_state == RUNNING_FULLSPEED) set_pid_constants(CONSTANTS_FAST);
							else set_pid_constants(CONSTANTS_HALF);
						}
					}
				}
				
				//Set last white seen time to current time
				last_left_sensor_count_time = get_time();
			}
			
			if (time0 == -1 && current_turn == slow_marker) time0 = get_time();
			if ((current_turn == slow_marker - 1 && robot_state != CALIBRATION )) {
				current_speed = 0.12;
			} if ((current_turn == slow_marker && robot_state != CALIBRATION)){
				current_speed = 0.12;
			}
			
			//If right outrigger senses white
			if (reflected_light_values[9] < 500) {
				
				if (check_sensor_states(reflected_light_values) == WHITE) cancel |= (1<<0);
				
				//If it has been more than 20ms since it last sensed white
				if (get_time() - last_right_sensor_count_time > 0.02) {
					
					if (cancel & (1 << 0)){
						cancel &= ~(1 << 0);
						break;
					} 
					
					//Increment times sensor has seen marker
					right_sensor_count++;
					
					//Second right marker is finish line
					if (right_sensor_count >= 2) {
						//If robot is in calibration mode, save turn values to EEPROM
						//Run FINISH sequence
						
						if (robot_state == CALIBRATION) {
							save = 1;
							turns[current_turn] = 0;
							selection = RUNNING_HALFSPEED;
						} else selection = robot_state;
						
						robot_state = FINISH;
						break;
					}
				}
				//Set last white seen time to current time
				last_right_sensor_count_time = get_time();
			}
			
			switch (check_sensor_states(reflected_light_values)) {
			case NORMAL: //Normal Operation				
				//set_leds(current_turn);
			
				if (current_turn != slow_marker) time0 = -1; // Robot not lost

				
				//Calculate correction value.
				double correction_value = calculate_PID_turn_value(reflected_light_values, dt);
				filtered_turn_value = filtered_turn_value * 0.99 + 0.01 * correction_value;
				
				set_differential_power(current_speed, correction_value + offset);
				break;
			case WHITE: //White line crossover
			
				//if (sin(get_time() * 150) > 0) set_leds(0b1111);
				//else (set_leds(0));
			
				if (time0 == -1) time0 = get_time();
				else if (get_time() - time0 < 0.2) set_differential_power(current_speed, 0);
				else if (get_time() - time0 < 0.4) set_differential_power(-0.1, 0);
				else if (get_time() - time0 < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - time0 > 0.6) robot_state = 0;
				break;
			case BLACK: //All black, robot lost.
			
				//if (sin(get_time() * 150) > 0) set_leds(0b1111);
				//else (set_leds(0));
			
				if (time0 == -1) time0 = get_time(); //Set lost time to current time
				else if (get_time() - time0 < 0.4 && get_time() - time0 > 0.2) set_differential_power(-0.1, 0);
				else if (get_time() - time0 < 0.6) set_motor_power_LR(0, 0); //Stop Motors
				else if (get_time() - time0 > 0.6) robot_state = 0;
				break;
			}
			
			break;
		
		case FINISH:
			if (time0 == -1) time0 = get_time();
			if (get_time() - time0 < 0.15) {
				set_differential_power(max_speed, 0);
			}
			else if (get_time() - time0 < 0.25) set_differential_power(-0.2, 0);
			else{ 
				time0 = -1;
				set_motor_power_LR(0, 0);
				
				robot_state = RESTARTING;
				
				if (save) {
					
					save = 0;
					
					//Write turns
					eeprom_write_byte((uint8_t*) 0, current_turn);
					eeprom_write_byte((uint8_t*) 1, slow_marker);
					eeprom_write_block((void*) turns, (uint16_t*) 2, 200);
					
					int i;
					for (i = 0; i< 10; i++) {
						set_leds(0b1111);
						_delay_ms(20);
						set_leds(0b0000);
						_delay_ms(20);
					}
				}
			}
			
			break;
		
		case STOPPED:
			time0 = -1;
			set_motor_power_LR(0, 0);
			if (selection > 0) set_leds(1 << selection - 1); else set_leds(0);
			break;
			
		case RESTARTING:
			if (time0 == -1) time0 = get_time();
			
			if (get_time() - time0 < 2) {
				set_leds(0b1010);
				robot_state = COUNTDOWN;
				time0 = -1;
			}
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