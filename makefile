TARGETS = \
	main.o \
	SensorArray.o \
	Serial.o

all: $(TARGETS) FLASH.bin

clean:
	for f in $(TARGETS); do \
		if [ -f $$f.elf ]; then rm $$f.elf; fi; \
		if [ -f $$f.o ]; then rm $$f.o; fi; \
	done

rebuild: clean all

%.o : %.c %.h
	avr-gcc -O1 -lprintf_flt -lm -Werror -Wl,-u,vfprintf -lprintf_min -mmcu=atmega32u4 -c $< -o $@

FLASH.bin : $(TARGETS)
	avr-gcc $(TARGETS) -mmcu=atmega32u4 -o out.elf
	avr-objcopy -O binary out.elf $@
