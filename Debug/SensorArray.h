#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include <avr/io.h>

void initialise_sensors();
uint16_t read_sensor(uint16_t sensor);
void get_reflected_light_values(uint16_t * reflected_light);

#endif