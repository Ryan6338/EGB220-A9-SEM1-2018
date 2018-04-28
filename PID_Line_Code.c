#define LIGHT_THRESHOLD_HIGH 920 //Black
#define LIGHT_THRESHOLD_LOW 60 //White
#define LIGHT_THRESHOLD_MID (LIGHT_THRESHOLD_LOW + (LIGHT_THRESHOLD_HIGH - LIGHT_THRESHOLD_LOW) / 2)
#define SENSOR_COUNT 8

#include "PID_Line_Code.h"

float line_edge_value;
float kP, kI, kD;

int32_t error_integral = 0;

const int16_t sensor_offset[] = {-33, -24, -15, -5, 5, 15, 24, 33};

//Function scales light value to be linear with distance from line
double adjust_light(uint16_t r) {
	return ((double) r - 12.4290) / 46.7460;
}

double calculate_error(uint16_t * reflected_light) {
	
	double error = 0;
	uint8_t useful_sensors = 0;
	
	int i;
	for (i = 0; i < SENSOR_COUNT; i++) {
		
		//Loop through sensors, check if they are within threshold. If so, linearize and add to average error.
		
		//If sensor is within threshold values
		if (reflected_light[i] <= LIGHT_THRESHOLD_HIGH) {
			
			useful_sensors++;
			
			//If sensor is last sensor
			if (i == SENSOR_COUNT - 1) {
				//Last sensor reflects less light ? left of line : right of line
				if (reflected_light[SENSOR_COUNT] < reflected_light[SENSOR_COUNT - 1]) {
					error = error - adjust_light(reflected_light[i]) - sensor_offset[i];
				} else {
					error = error + adjust_light(reflected_light[i]) - sensor_offset[i];
				}
				
			} else { //If not last sensor
				//Next sensor reflects more light ? Left of line : Right of line
				if (reflected_light[i] > reflected_light[i+1]) {
					error = error - adjust_light(reflected_light[i]) - sensor_offset[i];
				} else {
					error = error + adjust_light(reflected_light[i]) - sensor_offset[i];
				}
			}
		}
	}
	
	if (useful_sensors > 0) error = error / useful_sensors;
	
	return error;
}