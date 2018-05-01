#ifndef PID_LINE_CODE_H
#define PID_LINE_CODE_H

#include <stdlib.h>
#include <stdio.h>

double calculate_error(uint16_t * reflected_light);
uint8_t check_sensor_states(uint16_t * reflected_light);

#endif