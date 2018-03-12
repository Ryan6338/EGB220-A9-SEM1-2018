#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include <avr/io.h>

uint16_t reflected_light[8];

void initialise_sensors();
uint16_t read_sensor(uint8_t sensor);
uint16_t * get_reflected_light_values();

#endif