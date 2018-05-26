#ifndef PID_LINE_CODE_H
#define PID_LINE_CODE_H

#define KP 0.5
#define KI 2.9
#define KD 1.35

#include <stdio.h>

typedef enum SensorStates {
	NORMAL,
	WHITE,
	BLACK
} SensorStates;

double calculate_PID_turn_value(uint16_t * reflected_light, double dt);
SensorStates check_sensor_states(uint16_t * reflected_light);
double calculate_error(uint16_t * reflected_light);

#endif