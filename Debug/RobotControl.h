#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <avr/io.h>
#include <math.h>

void initialise_registers();
void set_leds(uint8_t format);
void set_motor_power_LR(double b, double a);
void set_differential_power(double speed, double turn_value);

#endif