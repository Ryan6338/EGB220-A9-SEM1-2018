#define LIGHT_THRESHOLD_HIGH 800 //Black
#define CROSSOVER_AVG_THRESHOLD 600 //White (Only used for crossover)
#define SENSOR_COUNT 10

#include "PID_Line_Code.h"

int16_t sensor_offset[] = {-75, -35, -25, -15, -5, 5, 15, 25, 35, 75};

static double integral = 0;
static double last_error = 0;
static double last_derivate = 0;

double calculate_PID_turn_value(uint16_t * reflected_light, double dt) {
		//Error value (between -1 and 1)
	double error = calculate_error(reflected_light);

	//Exponential filter on derivative to reduce erratic behaviour
	double derivative = ((error - last_error) / 0.01) * 0.99 + last_derivate * 0.01;
	last_derivate = derivative;
	last_error = error;

	integral = integral * 0.999 + error * 0.01;
	
	if (integral * error < 0) integral = 0;

	return error * KP + integral * KI + derivative * KD;
}

//Function scales light value to be linear with distance from line
double adjust_light(uint16_t r) {
	return ((double) r - 138) * 0.01383;
}

SensorStates check_sensor_states(uint16_t * reflected_light) {
	int i;
	uint32_t average = 0;
	uint16_t min = 1023;
	for (i = 0; i < SENSOR_COUNT; i++) {
		average += (uint32_t) reflected_light[i];
		min = reflected_light[i] < min ? reflected_light[i] : min;
	}
	
	average = average / SENSOR_COUNT;
	
	if (average < CROSSOVER_AVG_THRESHOLD) return WHITE;
	else if (min > LIGHT_THRESHOLD_HIGH) return BLACK;
	else return NORMAL;
}

double calculate_error(uint16_t * reflected_light) {
	
	double error = 0;
	uint8_t useful_sensors = 0;
	
	int i;
	for (i = 1; i < 9; i++) {
		
		//Loop through sensors, check if they are within threshold. If so, linearize and add to average error.
		
		//If sensor is within threshold values
		if (reflected_light[i] <= LIGHT_THRESHOLD_HIGH) {
			
			useful_sensors++;
			
			//If sensor is last sensor
			if (i == 8) {
				//Last sensor senses less light ? left of line : right of line
				if (reflected_light[8] < reflected_light[7]) {
					error = error - adjust_light(reflected_light[i]) - sensor_offset[i];
				} else {
					error = error + adjust_light(reflected_light[i]) - sensor_offset[i];
				}
				
			} else { //If not last sensor
				//Next sensor senses more light ? Left of line : Right of line
				if (reflected_light[i] > reflected_light[i+1]) {
					error = error - adjust_light(reflected_light[i]) - sensor_offset[i];
				} else {
					error = error + adjust_light(reflected_light[i]) - sensor_offset[i];
				}
			}
		}
	}
	
	if (useful_sensors > 0) error = error / useful_sensors;
	
	//Divide error by maximum possible value
	return error / (adjust_light(LIGHT_THRESHOLD_HIGH) + sensor_offset[SENSOR_COUNT-2]);
}