TARGETS = \
	main.o \
	SensorArray.o

all: $(TARGETS) FLASH.bin

clean:
	for f in $(TARGETS); do \
		if [ -f $$f.elf ]; then rm $$f.elf; fi; \
		if [ -f $$f.o ]; then rm $$f.o; fi; \
	done

rebuild: clean all

%.o : %.c %.h
	avr-gcc -O1 -mmcu=atmega32u4 -c $< -o $@

FLASH.bin : $(TARGETS)
	avr-gcc $(TARGETS) -o out.elf
	avr-objcopy -O binary out.elf $@
