#include "EEPROM.h"

static uint16_t current_addr;

void EEPROM_write_next(uint8_t data) {
	
	current_addr++;
	EEPROM_write(current_addr, data);
	
}

void EEPROM_write(uint16_t addr, uint8_t data) {
	
	current_addr = addr;
	
	//Wait for completion of previous write
	while(EECR & (1<<EEPE));
	
	//Set up address and data registers
	EEARH = (addr >> 8) & 0xFF;
	EEARL = addr & 0xFF;
	EEDR = data;
	
	cli();
	//Write 1 to EEMPE
	EECR |= (1 << EEMPE);
	
	//Start EEPROM write by setting EEPE
	EECR |= (1 << EEPE);
	sei();
}

uint8_t EEPROM_read_next() {
	
	current_addr++;
	return EEPROM_read(current_addr);
	
}

uint8_t EEPROM_read(uint16_t addr) {
	current_addr = addr;
	
	//Wait for previous write to finish
	while (EECR & (1<<EEPE));
	
	//Configure address register
	EEARH = (addr >> 8) & 0xFF;
	EEARL = addr & 0xFF;
	
	//Start EEPROM read
	EECR |= (1 << EEPE);
	
	return EEDR;
}