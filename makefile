TARGETS = \
	SensorArray.o\
  PID_Line_code.o\
	Serial.o\
	main.o

all: $(TARGETS) FLASH.bin

clean:
	for f in $(TARGETS); do \
		if [ -f $$f.elf ]; then rm $$f.elf; fi; \
		if [ -f $$f.o ]; then rm $$f.o; fi; \
	done

rebuild: clean all

%.o : %.c %.h
	avr-gcc -O1 -lm -Werror -Wl,-u -mmcu=atmega32u4 -c $< -o $@

FLASH.bin : $(TARGETS)
	avr-gcc $(TARGETS) -mmcu=atmega32u4 -o out.elf
	avr-objcopy -O binary out.elf $@
