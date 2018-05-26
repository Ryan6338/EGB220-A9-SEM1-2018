#ifndef EEPROM_H
#define EEPROM_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

uint8_t EEPROM_read_next();
uint8_t EEPROM_read(uint16_t addr);
void EEPROM_write(uint16_t addr, uint8_t data);
void EEPROM_write_next(uint8_t data);

#endif