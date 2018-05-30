#ifndef PID_LINE_CODE_H
#define PID_LINE_CODE_H

#include <stdio.h>

typedef enum SensorStates {
	NORMAL,
	WHITE,
	BLACK
} SensorStates;

void set_pid_constants(double new_kP, double new_kI, double new_kD);
double calculate_PID_turn_value(uint16_t * reflected_light, double dt);
SensorStates check_sensor_states(uint16_t * reflected_light);
double calculate_error(uint16_t * reflected_light);

#endif